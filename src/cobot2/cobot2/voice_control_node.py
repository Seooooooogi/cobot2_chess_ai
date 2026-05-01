import os
import io
import time
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from openai import OpenAI
from openwakeword.model import Model
from dotenv import load_dotenv

from .miccontroller import MicController, MicConfig


# =========================
# 설정 상수 (클래스 밖)
# =========================
ENV_PATH = ".env"

WAKEWORD_MODEL_PATH = "/home/kyb/cobot_ws/src/cobot2/models/hello_rokey_8332_32.tflite"
WAKEWORD_THRESHOLD = 0.4

VOICE_COMMAND_TOPIC = "voice_command"
VOICE_STATUS_TOPIC = "voice_status"

# main -> voice_control_node (복귀 신호)
WAKE_UP_SIGNAL = "WAKE_UP"
PASS_SIGNAL = "pass"

# voice_control_node -> main -> UI
VOICE_UI_STATUS_TOPIC = "voice_ui_status"
UI_MSG_WAKEWORD_WAIT = "Hello rokey로 불러주세요"
UI_MSG_PASS_LISTENING = "pass 감지 중..."
UI_MSG_PASS_DETECTED = "감지!"

WHISPER_MODEL = "whisper-1"
WHISPER_LANGUAGE = "en"
WHISPER_PROMPT = "The user is playing chess and wants to 'Pass' the turn."

SPIN_TIMEOUT_SEC = 0.01

MIC_FLUSH_CHUNKS = 30
WAKEWORD_COOLDOWN_SEC = 0.7


class VoiceControlNode(Node):
    def __init__(self):
        super().__init__("voice_control_node")

        load_dotenv(ENV_PATH)

        self.STATE_WAKEWORD = "WAKEWORD_MODE"
        self.STATE_COMMAND = "COMMAND_MODE"
        self.STATE_SLEEPING = "SLEEPING_MODE"
        self.current_state = self.STATE_WAKEWORD

        self._cooldown_until = 0.0

        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY is not set in environment")

        self.client = OpenAI(api_key=api_key)

        self.oww_model = Model(wakeword_models=[WAKEWORD_MODEL_PATH])
        self.model_name = list(self.oww_model.models.keys())[0]

        self.mic = MicController(MicConfig())
        self.mic.open_stream()

        self.voice_command_pub = self.create_publisher(String, VOICE_COMMAND_TOPIC, 10)
        self.ui_status_pub = self.create_publisher(String, VOICE_UI_STATUS_TOPIC, 10)
        self.status_sub = self.create_subscription(String, VOICE_STATUS_TOPIC, self.status_callback, 10)

        # 시작 상태: wakeword 대기 메시지
        self._publish_ui_status(UI_MSG_WAKEWORD_WAIT)

        self.get_logger().info(f"VoiceControlNode started. state={self.current_state}")

    def status_callback(self, msg: String):
        if (msg.data or "").strip() == WAKE_UP_SIGNAL:
            self._flush_mic_buffer()
            self._cooldown_until = time.time() + WAKEWORD_COOLDOWN_SEC

            self.current_state = self.STATE_WAKEWORD
            self._publish_ui_status(UI_MSG_WAKEWORD_WAIT)

            self.get_logger().info("WAKE_UP received. Back to wakeword listening.")

    def _publish_ui_status(self, text: str):
        m = String()
        m.data = text
        self.ui_status_pub.publish(m)

    def _flush_mic_buffer(self):
        try:
            if self.mic is None or self.mic.stream is None:
                return
            for _ in range(MIC_FLUSH_CHUNKS):
                _ = self.mic.stream.read(self.mic.config.chunk, exception_on_overflow=False)
        except Exception as e:
            self.get_logger().warn(f"Mic flush failed: {e}")

    def run_detection(self):
        # sleeping 상태에서는 아무것도 안 함 (main이 WAKE_UP 줄 때까지)
        if self.current_state == self.STATE_SLEEPING:
            return

        if self.current_state != self.STATE_WAKEWORD:
            return

        if time.time() < self._cooldown_until:
            return

        try:
            raw_data = self.mic.stream.read(self.mic.config.chunk, exception_on_overflow=False)
            audio_data = np.frombuffer(raw_data, dtype=np.int16)

            prediction = self.oww_model.predict(audio_data)
            confidence = prediction[self.model_name]

            if confidence > WAKEWORD_THRESHOLD:
                self.get_logger().info(f"Wakeword detected. score={confidence:.2f}")

                # wakeword 감지됨 → pass 듣는 상태 표시
                self._publish_ui_status(UI_MSG_PASS_LISTENING)

                self.current_state = self.STATE_COMMAND
                self.process_whisper_command()

        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")

    def process_whisper_command(self):
        try:
            self.get_logger().info("Listening command (3s)...")
            wav_data = self.mic.record_command()

            audio_file = io.BytesIO(wav_data)
            audio_file.name = "input.wav"

            transcript = self.client.audio.transcriptions.create(
                model=WHISPER_MODEL,
                file=audio_file,
                language=WHISPER_LANGUAGE,
                prompt=WHISPER_PROMPT,
            )

            text = (transcript.text or "").strip().lower()
            self.get_logger().info(f"Recognized: '{text}'")

            if any(kw in text for kw in ["pass", "패스", "path"]):
                # pass 감지 완료
                self._publish_ui_status(UI_MSG_PASS_DETECTED)

                self.get_logger().info("Pass confirmed. Publishing 'pass' and entering sleeping mode.")
                self.voice_command_pub.publish(String(data=PASS_SIGNAL))

                self._flush_mic_buffer()
                self.current_state = self.STATE_SLEEPING

            else:
                self.get_logger().warn("Invalid command. Back to wakeword.")
                self._flush_mic_buffer()
                self.current_state = self.STATE_WAKEWORD
                self._cooldown_until = time.time() + 0.2
                self._publish_ui_status(UI_MSG_WAKEWORD_WAIT)

        except Exception as e:
            self.get_logger().error(f"Whisper error: {e}")
            self._flush_mic_buffer()
            self.current_state = self.STATE_WAKEWORD
            self._cooldown_until = time.time() + 0.3
            self._publish_ui_status(UI_MSG_WAKEWORD_WAIT)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()

    try:
        while rclpy.ok():
            node.run_detection()
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_SEC)
    finally:
        try:
            node.mic.close_stream()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

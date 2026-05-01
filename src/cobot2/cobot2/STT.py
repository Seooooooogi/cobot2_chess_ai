from openai import OpenAI
import sounddevice as sd
import scipy.io.wavfile as wav
import numpy as np
import tempfile
import os
from dotenv import load_dotenv

# .env 파일에서 환경변수 로드
load_dotenv()

# OpenAI API Key 불러오기
openai_api_key = os.getenv("OPENAI_API_KEY")


class STT:
    def __init__(self, openai_api_key):
        # OpenAI 클라이언트 생성 (Whisper STT용)
        self.client = OpenAI(api_key=openai_api_key)

        # 녹음 길이 (초)
        self.duration = 3

        # 샘플레이트 (Whisper 권장: 16kHz)
        self.samplerate = 16000

    def speech2command(self):
        # ===== 음성 녹음 시작 =====
        print("음성 녹음을 시작합니다. (3초)")

        # 마이크로부터 음성 녹음
        audio = sd.rec(
            int(self.duration * self.samplerate),  # 전체 샘플 수
            samplerate=self.samplerate,
            channels=1,        # 모노
            dtype="int16",     # Whisper 호환 포맷
        )

        # 녹음이 끝날 때까지 대기
        sd.wait()

        # ===== 임시 WAV 파일 생성 =====
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            # 녹음한 오디오를 WAV 파일로 저장
            wav.write(temp_wav.name, self.samplerate, audio)

            # ===== Whisper STT 호출 =====
            with open(temp_wav.name, "rb") as f:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1",  # OpenAI Whisper 모델
                    file=f,             # 음성 파일
                    language="ko"       # 한국어 인식 명시
                )

        # STT 결과 텍스트 정리
        text = transcript.text.strip().lower()
        print("인식된 텍스트:", text)

        # ===== 명령어 판별 =====

        # "pass" 관련 명령
        if any(w in text for w in ["pass", "패스", "파스"]):
            return "PASS"

        # "최선의 수" 관련 명령
        if "최선의 수" in text:
            return "BEST_MOVE"

        # 인식은 되었지만 명령이 아님
        return "UNKNOWN"


if __name__ == "__main__":
    # STT 객체 생성
    stt = STT(openai_api_key)

    # 음성 → 명령 변환
    command = stt.speech2command()

    # ===== 명령 처리 =====
    if command == "PASS":
        print("턴을 넘깁니다")
        # chess_engine.pass_turn()

    elif command == "BEST_MOVE":
        print("최선의 수를 계산합니다")
        # move = chess_engine.get_best_move()
        # chess_engine.make_move(move)

    else:
        print("명령을 인식하지 못했습니다")

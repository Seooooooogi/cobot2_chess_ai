import pyaudio
import wave
import io
import sys

class MicConfig:
    chunk: int = 1280
    rate: int = 16000
    channels: int = 1
    record_seconds: int = 3 # 3초 동안 명령어를 녹음합니다.
    fmt: int = pyaudio.paInt16
    device_name_keyword: str = "pulse" 

class MicController:
    def __init__(self, config: MicConfig = MicConfig()):
        self.config = config
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.sample_width = self.audio.get_sample_size(self.config.fmt)
        self.target_index = self.find_device_index()

    def find_device_index(self):
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            if self.config.device_name_keyword in info['name'] and info['maxInputChannels'] > 0:
                print(f"🎯 마이크 발견: {info['name']} (Index: {i})")
                return i
        return None

    def open_stream(self):
        try:
            self.stream = self.audio.open(
                format=self.config.fmt,
                channels=self.config.channels,
                rate=self.config.rate,
                input=True,
                input_device_index=self.target_index,
                frames_per_buffer=self.config.chunk
            )
            print(f"✅ 스트림 오픈 완료")
        except Exception as e:
            print(f"❌ 오픈 실패: {e}")
            sys.exit(1)

    # 이 함수가 빠져있어서 에러가 났었습니다. 다시 추가합니다.
    def record_command(self) -> bytes:
        """설정된 시간(3초) 동안 음성을 녹음하여 반환합니다."""
        print(f"🎙️ [{self.config.record_seconds}초] 명령을 말씀해 주세요...")
        frames = []
        # 초당 필요한 청크 개수 계산
        num_chunks = int(self.config.rate / self.config.chunk * self.config.record_seconds)
        
        for _ in range(num_chunks):
            data = self.stream.read(self.config.chunk, exception_on_overflow=False)
            frames.append(data)

        wav_io = io.BytesIO()
        with wave.open(wav_io, 'wb') as wf:
            wf.setnchannels(self.config.channels)
            wf.setsampwidth(self.sample_width)
            wf.setframerate(self.config.rate)
            wf.writeframes(b''.join(frames))
        
        return wav_io.getvalue()

    def close_stream(self):
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()


# import pyaudio
# import wave
# import io
# import sys

# class MicConfig:
#     chunk: int = 1280
#     rate: int = 16000
#     channels: int = 2
#     record_seconds: int = 3
#     fmt: int = pyaudio.paInt16
#     # 하드웨어 인덱스를 직접 지정 (C270 웹캠: 10)
#     device_index: int = 5 

# class MicController:
#     def __init__(self, config: MicConfig = MicConfig()):
#         # config 값 대신 직접 번호를 박아서 테스트
#         self.config = config
#         self.config.device_index = 5  # <--- 여기서 강제로 5번 설정
#         self.config.channels = 2      # <--- 내장 마이크 채널 2개
        
#         self.audio = pyaudio.PyAudio()
#         self.stream = None
#         self.sample_width = self.audio.get_sample_size(self.config.fmt)
        
#         # [디버그] 실제로 어떤 번호로 시도하는지 출력
#         print(f"DEBUG: 실제 연결 시도 인덱스: {self.config.device_index}")
# # class MicController:
# #     def __init__(self, config: MicConfig = MicConfig()):
# #         self.config = config
# #         self.audio = pyaudio.PyAudio()
# #         self.stream = None
# #         self.sample_width = self.audio.get_sample_size(self.config.fmt)

#     def open_stream(self):
#         """실시간 감지를 위해 스트림을 엽니다."""
#         target_idx = self.config.device_index
        
#         try:
#             # 장치 정보 확인 로그
#             info = self.audio.get_device_info_by_index(target_idx)
#             print(f"🎤 [Attempt] 장치 연결 시도: {info['name']} (Index: {target_idx})")
            
#             # C270 웹캠 등 실제 마이크는 보통 1채널입니다.
#             # 만약 장치가 2채널만 지원한다면 자동으로 맞춰서 시도합니다.
#             max_in_channels = int(info['maxInputChannels'])
#             use_channels = min(self.config.channels, max_in_channels)
            
#             if max_in_channels == 0:
#                 print(f"❌ 에러: 인덱스 {target_idx} 장치는 마이크(입력) 기능이 없습니다!")
#                 sys.exit(1)

#             self.stream = self.audio.open(
#                 format=self.config.fmt,
#                 channels=use_channels,
#                 rate=self.config.rate,
#                 input=True,
#                 input_device_index=target_idx,
#                 frames_per_buffer=self.config.chunk,
#             )
#             print(f"✅ [Success] 스트림 오픈 완료: {info['name']}")
            
#         except Exception as e:
#             print(f"❌ [Failure] 마이크 오픈 실패: {e}")
#             print("💡 TIP: 'ros2 run' 실행 전 'python3 -c ...' 스크립트로 인덱스를 재확인하세요.")
#             raise e

#     def close_stream(self):
#         if self.stream:
#             try:
#                 self.stream.stop_stream()
#                 self.stream.close()
#             except: pass
#         self.audio.terminate()

#     def record_command(self) -> bytes:
#         """3초간 명령어를 녹음하여 WAV 데이터를 반환합니다."""
#         print("🎙️ 명령을 말씀해 주세요 (3초)...")
#         frames = []
#         num_chunks = int(self.config.rate / self.config.chunk * self.config.record_seconds)
        
#         for _ in range(num_chunks):
#             try:
#                 data = self.stream.read(self.config.chunk, exception_on_overflow=False)
#                 frames.append(data)
#             except Exception as e:
#                 print(f"⚠️ 녹음 누락: {e}")

#         wav_io = io.BytesIO()
#         with wave.open(wav_io, 'wb') as wf:
#             wf.setnchannels(self.config.channels)
#             wf.setsampwidth(self.sample_width)
#             wf.setframerate(self.config.rate)
#             wf.writeframes(b''.join(frames))
        
#         return wav_io.getvalue()
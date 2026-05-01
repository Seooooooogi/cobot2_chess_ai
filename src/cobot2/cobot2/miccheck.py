import pyaudio

p = pyaudio.PyAudio()
print("=== 사용 가능한 오디오 장치 목록 ===")
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    if info['maxInputChannels'] > 0:
        print(f"Index {i}: {info['name']} (Max Channels: {info['maxInputChannels']})")
p.terminate()
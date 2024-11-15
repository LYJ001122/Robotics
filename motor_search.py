from dynamixel_sdk import *  # Dynamixel SDK 라이브러리 임포트

# 포트 설정
DEVICENAME = '/dev/ttyUSB0'  # 실제 사용 중인 포트로 변경
BAUDRATE = 57600

# 프로토콜 버전 (MX-64는 1.0 또는 2.0 사용 가능)
PROTOCOL_VERSION = 2.0

# 포트와 패킷 핸들러 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# 포트 열기
if portHandler.openPort():
    print("포트를 열었습니다.")
else:
    print("포트를 여는데 실패했습니다.")
    quit()

# 통신 속도 설정
if portHandler.setBaudRate(BAUDRATE):
    print("통신 속도를 설정했습니다.")
else:
    print("통신 속도 설정에 실패했습니다.")
    quit()

# ID 검색
for motor_id in range(0, 15):
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, motor_id)
    if dxl_comm_result == COMM_SUCCESS:
        print(f"모터 ID {motor_id}를 찾았습니다. 모델 번호: {dxl_model_number}")
    elif dxl_comm_result == COMM_RX_TIMEOUT:
        print(f"모터 ID {motor_id}에서 응답이 없습니다.")
        continue  # 응답 없음 (모터 없음)
    else:
        pass
        # print(f"모터 ID {motor_id}에서 오류 발생: {packetHandler.getTxRxResult(dxl_comm_result)}")

# 포트 닫기
portHandler.closePort()

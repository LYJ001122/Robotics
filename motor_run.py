from dynamixel_sdk import *                    # Dynamixel SDK 모듈 임포트
import time

# 통신 설정
DEVICENAME = '/dev/ttyUSB_dynamixel'                    # 통신 포트 설정 (시스템에 맞게 변경)
BAUDRATE = 57600                               # 보레이트 설정
PROTOCOL_VERSION = 2.0                         # 프로토콜 버전 설정
ADDR_TORQUE_ENABLE = 64                        # Torque Enable 주소
ADDR_GOAL_POSITION = 116                        # 목표 위치 주소
ADDR_PRESENT_POSITION = 132                     # 현재 위치 주소
ADDR_OPERATING_MODE = 11                       # Operating Mode 주소 (특정 모델에서 지원)
TORQUE_ENABLE = 1                              # Torque On
TORQUE_DISABLE = 0                             # Torque Off
POSITION_CONTROL_MODE = 3                      # 위치 제어 모드 값
DXL_MIN_POSITION = 100                         # 최소 위치값
DXL_MAX_POSITION = 1000                        # 최대 위치값
MOVEMENT_DELAY = 0.5                           # 회전 시 각 목표 위치 간의 지연 시간

# 사용하려는 모터 ID 설정
MOTOR_IDS = [6]                          # 모터 ID 리스트

# 포트 핸들러 초기화
port_handler = PortHandler(DEVICENAME)
packet_handler = PacketHandler(PROTOCOL_VERSION)

# 포트 열기
if not port_handler.openPort():
    print("포트를 열 수 없습니다.")
    exit()

# 보레이트 설정
if not port_handler.setBaudRate(BAUDRATE):
    print("보레이트 설정에 실패했습니다.")
    exit()



# 모터 초기화 및 위치 제어 모드 설정
for motor_id in MOTOR_IDS:
    # 모터 Torque 비활성화 (모드 설정을 위해)
    packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    # 위치 제어 모드로 설정
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"모터 {motor_id} 위치 제어 모드 설정 오류: {packet_handler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"모터 {motor_id} 오류: {packet_handler.getRxPacketError(dxl_error)}")
    else:
        print(f"모터 {motor_id} 위치 제어 모드로 설정 완료")

    # 모터 Torque 활성화
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"모터 {motor_id} Torque 활성화 오류: {packet_handler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"모터 {motor_id} 오류: {packet_handler.getRxPacketError(dxl_error)}")
    else:
        print(f"모터 {motor_id} Torque 활성화 성공")

for motor_id in MOTOR_IDS:
    dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, motor_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"모터 {motor_id} 현재 위치 읽기 오류: {packet_handler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"모터 {motor_id} 오류: {packet_handler.getRxPacketError(dxl_error)}")
    else:
        print(f"모터 {motor_id} 현재 위치: {dxl_present_position}")

#! 모터 회전 - 목표 위치를 반복적으로 변경하여 회전 동작 구현
goal_positions = [DXL_MIN_POSITION, DXL_MAX_POSITION]  # 목표 위치 리스트 (회전 범위)
for i in range(5):  # 반복 횟수 설정
    for goal_position in goal_positions:
        for motor_id in MOTOR_IDS:
            # 목표 위치 설정
            dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, motor_id, ADDR_GOAL_POSITION, goal_position)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"모터 {motor_id} 목표 위치 설정 오류: {packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"모터 {motor_id} 오류: {packet_handler.getRxPacketError(dxl_error)}")
            else:
                print(f"모터 {motor_id} 목표 위치 {goal_position}로 설정 완료")
        
        # 목표 위치에 도달하기 전 대기
        time.sleep(MOVEMENT_DELAY)

# 현재 위치 읽기 (예시)
for motor_id in MOTOR_IDS:
    dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, motor_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"모터 {motor_id} 현재 위치 읽기 오류: {packet_handler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"모터 {motor_id} 오류: {packet_handler.getRxPacketError(dxl_error)}")
    else:
        print(f"모터 {motor_id} 현재 위치: {dxl_present_position}")

# 포트 닫기
port_handler.closePort()

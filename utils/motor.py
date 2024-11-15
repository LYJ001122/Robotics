from dynamixel_sdk import *                    # Dynamixel SDK 모듈 임포트
import numpy as np

class Motor():
    def __init__(self):
        self.DEVICENAME = '/dev/dynamixel'                    # 통신 포트 설정 (시스템에 맞게 변경)
        self.BAUDRATE = 57600                               # 보레이트 설정
        self.PROTOCOL_VERSION = 2.0                         # 프로토콜 버전 설정
        self.ADDR_TORQUE_ENABLE = 64                        # Torque Enable 주소
        self.ADDR_GOAL_POSITION = 116                        # 목표 위치 주소
        self.ADDR_PRESENT_POSITION = 132                     # 현재 위치 주소
        self.ADDR_OPERATING_MODE = 11                       # Operating Mode 주소 (특정 모델에서 지원)
        self.TORQUE_ENABLE = 1                              # Torque On
        self.TORQUE_DISABLE = 0                             # Torque Off
        self.POSITION_CONTROL_MODE = 3                      # 위치 제어 모드 값
        self.EXTENDED_POSITION_CONTROL_MODE = 4  # Extended Position Control Mode의 모드 값
        self.DXL_MIN_POSITION = 100                         # 최소 위치값
        self.DXL_MAX_POSITION = 1000                        # 최대 위치값
        self.MOVEMENT_DELAY = 0.5                           # 회전 시 각 목표 위치 간의 지연 시간
        self.ADDR_MIN_POSITION_LIMIT = 52                    # mx-28의 최소 위치 제한 주소
        self.ADDR_MAX_POSITION_LIMIT = 48                   # mx-28/64 의 최대 위치 제한 주소
        self.ADDR_MOVING_SPEED = 32


        self.MOTOR_NUM = 12
        self.MOTOR_IDS = range(1, self.MOTOR_NUM+1)                          # 모터 ID 리스트

        self.port_handler = PortHandler(self.DEVICENAME)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        self.init_pose = {
            1: 2048,
            2: 2048,
            3: 2903,
            4: 2048,
            5: 2048,
            6: 3986,
            7: 2048,
            8: 2048,
            9: 2138,
            10: 2048,
            11: 2048,
            12: 2377,
        }

        if not self.port_handler.openPort():
            print("포트를 열 수 없습니다.")
            exit()
        else:
            print(f"포트를 열었습니다.: {self.DEVICENAME}")

        if not self.port_handler.setBaudRate(self.BAUDRATE):
            print("보레이트 설정에 실패했습니다.")
            exit()
        else:
            print(f"보레이트를 설정했습니다.: {self.BAUDRATE}")

        for motor_id in self.MOTOR_IDS:
            #! 모터 Torque 비활성화 (모드 설정을 위해)
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

            #! Position Control Mode로 설정
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.ADDR_OPERATING_MODE, self.POSITION_CONTROL_MODE)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"모터 {motor_id} 위치 제어 모드 설정 오류: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"모터 {motor_id} 오류: {self.packet_handler.getRxPacketError(dxl_error)}")
            else:
                print(f"모터 {motor_id} 위치 제어 모드로 설정 완료")

            #! 모터 Torque 활성화
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"모터 {motor_id} Torque 활성화 오류: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"모터 {motor_id} 오류: {self.packet_handler.getRxPacketError(dxl_error)}")
            else:
                print(f"모터 {motor_id} Torque 활성화 성공")

        print("========================== 모터 초기화 완료 ==========================")

    # def get_init_pose(self):
    #     # 초기 위치 설정
    #     init_pose = {
    #         1: 2536,
    #         2: 3964,
    #         3: 2868,
    #         4: 138,
    #         5: 212,
    #         6: 3992,
    #         7: 1169,
    #         8: 2146,
    #         #todo: initial pose? 9: ,
    #         10: 6839,
    #         11: 7555,
    #         12: 2419,
    #     }

    #     return init_pose
    

    # # todo: safe pose check when moving motion generation
    # def get_safe_pose_left_big(self, init_angle):
    #     min_value = init_angle - 990
    #     max_value = init_angle - 448
    #     safe_angle = np.clip(init_angle, min=min_value, max=max_value)
    #     return safe_angle

    # def get_safe_pose_left_small(self, init_angle):
    #     min_value = init_angle - 430
    #     max_value = init_angle + 0
    #     safe_angle = np.clip(init_angle, min=min_value, max=max_value)
    #     return safe_angle

    # def get_safe_pose_right_big(self, init_angle):
    #     min_value = init_angle + 448
    #     max_value = init_angle + 990
    #     safe_angle = np.clip(init_angle, min=min_value, max=max_value)
    #     return safe_angle

    # def get_safe_pose_right_small(self, init_angle):
    #     min_value = init_angle + 0
    #     max_value = init_angle + 430
    #     safe_angle = np.clip(init_angle, min=min_value, max=max_value)
    #     return safe_angle

    def get_step_angle_delta(self):
        angle_deltas = []
        motor_direction = [1, 1, -1, -1]
        motor_deltas = [[100, 200, 300, -200, 0], [150, 500, 500, -420, 0], [0, 0, 0, 0, 0],
                        [0, 220, 220, -200, -200], [0, 300, -420, -420, -200], [0, 0, 0, 0, 0], 
                        [0, -220, -220, 200, 200], [0, -300, 420, 420, 200], [0, 0, 0, 0, 0],
                        [-100, -200, -300, 200, 0], [-150, -500, -500, 420, 0], [0, 0, 0, 0, 0]
                        ]
        
        # for i in range(4):
        #     for j in range(3):
        #         angle_deltas.append([x * motor_direction[i] for x in motor_deltas[j]])
        angle_deltas = motor_deltas

        return angle_deltas
    
    def get_step_angle(self):
        angle_deltas = self.get_step_angle_delta()
        self.angles = []
        for i in range(12):
            self.angles.append([self.init_pose[i+1]+x for x in angle_deltas[i]])

        return self.angles
        

    def read_angle(self, motor_id):
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, self.ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"모터 {motor_id} 현재 위치 읽기 오류: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"모터 {motor_id} 오류: {self.packet_handler.getRxPacketError(dxl_error)}")
        else:
            print(f"모터 {motor_id} 현재 위치: {dxl_present_position}")

        return dxl_present_position
    
    def run_motor(self, motor_id, goal_position):
        #! 모터 위치 제어
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, self.ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"모터 {motor_id} 위치 제어 오류: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"모터 {motor_id} 오류: {self.packet_handler.getRxPacketError(dxl_error)}")
        else:
            print(f"모터 {motor_id} 위치 제어 성공")

        return dxl_comm_result==COMM_SUCCESS and dxl_error==0
    
    def run_motor_step(self, motor_ids, step, delay = 0):
        for motor_id in motor_ids:
            self.run_motor(motor_id, self.angles[motor_id - 1][step])
        time.sleep(delay)

    # def safe_small(init_angle):
    #     return np.clip(init_angle, min=, max=)

    # def safe_big(init_angle):
    #     return np.clip(init_angle, min=, max=)

    # def rotate(self, motor_id, angle):
    #     # 현재 위치 읽기
    #     current_position = self.read_angle(motor_id)

    #     # 목표 위치 계산
    #     goal_position = (current_position + angle) % 4096
    #     if goal_position < 0:
    #         goal_position += 4096  # 음수일 경우 4096 범위로 보정

    #     # 목표 위치로 이동할 때 최단 경로 선택
    #     clockwise_distance = (goal_position - current_position) % 4096
    #     counter_clockwise_distance = (current_position - goal_position) % 4096

    #     # 회전 방향 결정
    #     if clockwise_distance <= counter_clockwise_distance:
    #         # 시계 방향 이동
    #         final_goal_position = (current_position + clockwise_distance) % 4096
    #     else:
    #         # 반시계 방향 이동
    #         final_goal_position = (current_position - counter_clockwise_distance) % 4096

    #     # 모터 이동
    #     success = self.run_motor(motor_id, int(final_goal_position))
    #     if success:
    #         print(f"모터 {motor_id}가 {angle}도 만큼 회전하였습니다.")
    #     else:
    #         print(f"모터 {motor_id} 회전 실패.")

    def set_position_limits(self, motor_id, min_limit, max_limit):
        # 최소 및 최대 위치 제한을 설정
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, self.ADDR_MIN_POSITION_LIMIT, min_limit)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"모터 {motor_id} 최소 위치 제한 설정 오류: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"모터 {motor_id} 오류: {self.packet_handler.getRxPacketError(dxl_error)}")
        
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, self.ADDR_MAX_POSITION_LIMIT, max_limit)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"모터 {motor_id} 최대 위치 제한 설정 오류: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"모터 {motor_id} 오류: {self.packet_handler.getRxPacketError(dxl_error)}")
        else:
            print(f"모터 {motor_id} 위치 제한 설정 완료: {min_limit} ~ {max_limit}")

    def initialize_position_limits(self):
        # 각 모터에 대해 설정할 제한값 딕셔너리
        position_limits = {
            1: (1648, 2448),
            2: (1598, 2548),
            4: (1648, 2448),
            5: (1628, 2348),
            7: (1648, 2448),
            8: (1748, 2468),
            10: (1648, 2448),
            11: (1548, 2498),
            # 나머지 모터별로 원하는 제한 범위를 추가
            # 예시로 3부터 12번까지 추가 가능
        }

        for motor_id, (min_limit, max_limit) in position_limits.items():
            self.set_position_limits(motor_id, min_limit, max_limit)

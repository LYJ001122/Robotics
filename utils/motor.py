from dynamixel_sdk import *                    # Dynamixel SDK 모듈 임포트
import numpy as np
from numpy import pi as PI
from numpy import cos as COS
from numpy import sin as SIN
from numpy import arctan as ARCTAN
from numpy import arccos as ARCCOS

class Motor():
    def __init__(self):
        self.DEVICENAME = '/dev/ttyUSB_dynamixel'           # '/dev/dynamixel'                    # 통신 포트 설정 (시스템에 맞게 변경)
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
        self.ADDR_PROFILE_VELOCITY = 112

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


        self.angles_forward = self.convert_positions_to_angles(self.get_step_position_forward())
        self.angles_right = self.convert_positions_to_angles(self.get_step_position_right())
        self.angles_left = self.convert_positions_to_angles(self.get_step_position_left())

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

        #! 모터 속도 설정
        for motor_id in self.MOTOR_IDS:
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, self.ADDR_PROFILE_VELOCITY, data=30)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Error setting velocity: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"Error detected: {self.packet_handler.getRxPacketError(dxl_error)}")

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
    
    def run_motor_mode(self, motor_ids, mode, step = None, delay = 0):
        if mode == 0:  # forward
            angles = self.angles_forward
        elif mode == 1:  # right
            angles = self.angles_right
        elif mode == 2:  # left
            angles = self.angles_left
        
        if step is None:
            for i in range(angles.shape[1]):
                for motor_id in motor_ids:
                    self.run_motor(motor_id, angles[motor_id - 1][i])
                time.sleep(delay)
        else:
            for motor_id in motor_ids:
                self.run_motor(motor_id, self.angles[motor_id - 1][step])
            time.sleep(delay)

    def move_front(self, motor_ids, step=None, delay=0):
        if step is None:
            for i in range(self.angles_forward.shape[1]):
                for motor_id in motor_ids:
                    self.run_motor(motor_id, self.angles_forward[motor_id - 1][i])
                time.sleep(delay)
        else:
            for motor_id in motor_ids:
                self.run_motor(motor_id, self.angles_forward[motor_id - 1][step])
            time.sleep(delay)

    def turn_right(self, motor_ids, step=None, delay=0):
        if step is None:
            for i in range(self.angles_right.shape[1]):
                for motor_id in motor_ids:
                    self.run_motor(motor_id, self.angles_right[motor_id - 1][i])
                time.sleep(delay)
        else:
            for motor_id in motor_ids:
                self.run_motor(motor_id, self.angles_right[motor_id - 1][step])
            time.sleep(delay)

    def turn_left(self, motor_ids, step=None, delay=0):
        if step is None:
            for i in range(self.angles_left.shape[1]):
                for motor_id in motor_ids:
                    self.run_motor(motor_id, self.angles_left[motor_id - 1][i])
                time.sleep(delay)
        else:
            for motor_id in motor_ids:
                self.run_motor(motor_id, self.angles_left[motor_id - 1][step])
            time.sleep(delay)
            
    '''
        motor_vertical_distance = [[14.405, 14.604, 13.193, 10.283, 14.142],    # motor_idx: 1(left_back_big), 2(left_back_small)
                                   [14.142, 13.807, 5.738, 10.283, 13.482],     # motor_idx: 4(left_front_big), 5(left_front_small)
                                   [14.142, 13.807, 5.738, 10.283, 13.482],     # motor_idx: 7(right_front_big), 8(right_front_small)
                                   [14.405, 14.604, 13.193, 10.283, 14.142]]    # motor_idx: 10(right_back_big), 11(right_back_small)
    '''
    def get_step_position_forward(self):   
        top, mid, bot, push = 3, 0, -3, 2
        #! go forward
        motor_vertical_distance = [[0, 0, 0, 0, 0, top, top, top, 0,              0, bot, bot, bot, 0, push, bot, 0, 0],
                                   [0, push, bot, 0, 0, bot, bot, bot, 0,        0, top, top, top, 0, 0, 0, 0, 0],
                                   [0, top, top, top, 0, 0, 0, 0, 0,              0, push, bot, 0, 0, bot, bot, bot, 0],
                                   [0, bot, bot, bot, 0, push, bot, 0, 0,        0, 0, 0, 0, 0, top, top, top, 0]]

        motor_horizontal_distance = [[0, 0, 0, 0, 0, 0, 0, 0, 0,            -4, -4, -4, -4, -4, -4, 0, 4, 4],
                                     [-4, -4, 0, 4, 4, 4, 4, 4, 4,          0, 0, 0, 0, 0, 0, 0, 0, 0],
                                     [0, 0, 0, 0, 0, 0, 0, 0, 0,            -4, -4, 0, 4, 4, 4, 4, 4, 4],
                                     [-4, -4, -4, -4, -4, -4, 0, 4, 4,      0, 0, 0, 0, 0, 0, 0, 0, 0]]
        
        motor_horizontal_distance = np.array(motor_horizontal_distance)
        motor_vertical_distance = np.array(motor_vertical_distance)

        a = 0.5
        motor_horizontal_distance[0] = motor_horizontal_distance[0] - a
        motor_horizontal_distance[3] = motor_horizontal_distance[3] - a
        motor_horizontal_distance[1] = motor_horizontal_distance[1] + a
        motor_horizontal_distance[2] = motor_horizontal_distance[2] + a

        return motor_vertical_distance, motor_horizontal_distance

    def get_step_position_right(self):   
        top, mid, bot, push = 2, 0, -2, 1
        #! rotate left
        motor_vertical_distance = [[0, top, top, top, 0, bot, bot, bot, 0, push, push],
                                   [0, bot, bot, bot, 0, top, top, top, 0, 0, 0],
                                   [0, 0, 0, 0, 0, push, bot, 0, 0, bot, bot],
                                   [0, push, bot, 0, 0, 0, 0, 0, 0, top, top]]

        motor_horizontal_distance = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                     [-4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4],
                                     [0, 0, 0, 0, 0, 0, 3, 6, 6, 6, 0],
                                     [-4, -4, 0, 2, 2, 2, 2, 2, 2, 2, -4]]
        
        motor_horizontal_distance = np.array(motor_horizontal_distance)
        motor_vertical_distance = np.array(motor_vertical_distance)

        a = 0.5
        motor_horizontal_distance[0] = motor_horizontal_distance[0] - a
        motor_horizontal_distance[3] = motor_horizontal_distance[3] - a
        motor_horizontal_distance[1] = motor_horizontal_distance[1] + a
        motor_horizontal_distance[2] = motor_horizontal_distance[2] + a

        return motor_vertical_distance, motor_horizontal_distance

    def get_step_position_left(self):   
        top, mid, bot, push = 2, 0, -2, 1
        #! rotate left
        motor_vertical_distance = [[0, push, bot, 0, 0, 0, 0, 0, 0, top, top],
                                   [0, 0, 0, 0, 0, push, bot, 0, 0, bot, bot],
                                   [0, bot, bot, bot, 0, top, top, top, 0, 0, 0],
                                   [0, top, top, top, 0, bot, bot, bot, 0, push, push]
                                   ]

        motor_horizontal_distance = [[-4, -4, 0, 2, 2, 2, 2, 2, 2, 2, -4],
                                     [0, 0, 0, 0, 0, 0, 3, 6, 6, 6, 0],
                                     [-4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4],
                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]        
        
        motor_horizontal_distance = np.array(motor_horizontal_distance)
        motor_vertical_distance = np.array(motor_vertical_distance)

        a = 0.5
        motor_horizontal_distance[0] = motor_horizontal_distance[0] - a
        motor_horizontal_distance[3] = motor_horizontal_distance[3] - a
        motor_horizontal_distance[1] = motor_horizontal_distance[1] + a
        motor_horizontal_distance[2] = motor_horizontal_distance[2] + a

        return motor_vertical_distance, motor_horizontal_distance

        
        
        
    def convert_positions_to_angles(self, positions=None):
        vertical_positions, horizontal_positions = positions
        vertical_positions = vertical_positions + 14.142
        _, step_num = horizontal_positions.shape
        motor_angle_list = np.zeros((12, step_num), dtype=float)

        for leg_idx in range(4):
            for step in range(step_num):
                vertical_distance = vertical_positions[leg_idx][step]
                horizontal_distance = horizontal_positions[leg_idx][step]

                big_motor_angle, small_motor_angle = self.distance_to_angle(motor_id=leg_idx*3+1, vertical_distance=vertical_distance, horizontal_distance=horizontal_distance)
                motor_angle_list[leg_idx*3][step] = big_motor_angle
                motor_angle_list[leg_idx*3+1][step] = small_motor_angle
        # float to int
        motor_angle_list = motor_angle_list.astype(int)
        
        return motor_angle_list
    

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
            4: (1548, 2448),
            5: (1628, 2400),
            7: (1648, 2548),
            8: (1700, 2468),
            10: (1648, 2448),
            11: (1548, 2498),
            # 나머지 모터별로 원하는 제한 범위를 추가
            # 예시로 3부터 12번까지 추가 가능
        }

        for motor_id, (min_limit, max_limit) in position_limits.items():
            self.set_position_limits(motor_id, min_limit, max_limit)

    def angle_to_distance(self, motor_id, raw_angle1, raw_angle2):
        # 모터 각도를 이용한 변환식
        alpha = raw_angle1 / 4096 * 2 * PI
        beta = raw_angle2 / 4096 * 2 * PI

        if motor_id == 7:                          # right_front
            alpha = alpha - 0.75*PI
            beta = beta - 0.5*PI
            
            x = 10*COS(alpha) - 10*COS(0.75*PI-beta)
            y = 10*SIN(alpha) + 10*SIN(0.75*PI-beta)

            vertical_distance = y
            horizontal_distance = -x
            print()
        elif motor_id == 4:                        # left_front
            alpha = alpha - 0.25*PI
            beta = beta - 0.5*PI

            x = 10*COS(alpha) + 10*COS(beta-0.25*PI)
            y = 10*SIN(alpha) + 10*SIN(beta-0.25*PI)

            vertical_distance = y
            horizontal_distance = x
            print()
        elif motor_id == 10:                        # right_back
            alpha = alpha - 0.75*PI
            beta = 1.5*PI - beta

            x = 10*COS(alpha) - 10*COS(beta-0.25*PI)
            y = 10*SIN(alpha) + 10*SIN(beta-0.25*PI)

            vertical_distance = y
            horizontal_distance = -x
            print()
        elif motor_id == 1:                         # left_back
            alpha = alpha - 0.25*PI
            beta = 1.5*PI - beta
            
            x = 10*COS(alpha) + 10*COS(0.75*PI-beta)
            y = 10*SIN(alpha) + 10*SIN(0.75*PI-beta)

            vertical_distance = y
            horizontal_distance = x
            print()
        else:
            print("모터 ID 오류")
            return None
        
        return vertical_distance, horizontal_distance
    
    def distance_to_angle(self, motor_id, vertical_distance, horizontal_distance):
        if motor_id == 7 or motor_id == 10:                          # right_front, right_back
            alpha = ARCTAN(horizontal_distance/vertical_distance) + 0.5*ARCCOS(1-(horizontal_distance**2+vertical_distance**2)/200)
            beta = 0.75*PI - 0.5*ARCCOS(1-(horizontal_distance**2+vertical_distance**2)/200) + ARCTAN(horizontal_distance/vertical_distance)

            # 기준점 -> 180도 init_pose
            alpha = alpha + 0.75*PI
            beta = beta + 0.5*PI
        elif motor_id == 4 or motor_id == 1:                        # left_front, left_back
            alpha = ARCTAN(horizontal_distance/vertical_distance) + 0.5*ARCCOS(1-(horizontal_distance**2+vertical_distance**2)/200)
            beta = 0.75*PI - 0.5*ARCCOS(1-(horizontal_distance**2+vertical_distance**2)/200) + ARCTAN(horizontal_distance/vertical_distance)
            
            # 기준점 -> 180도 init_pose
            alpha = alpha + 0.75*PI
            beta = beta + 0.5*PI

            # 반대쪽다리는 360-각도
            alpha = 2*PI - alpha
            beta = 2*PI - beta
        else:
            print("모터 ID 오류")
            return None
        
        #todo: 0-4095 range check
        raw_angle1 = int(alpha / 2 / PI * 4095) + 1
        raw_angle2 = int(beta / 2 / PI * 4095) +1

        return raw_angle1, raw_angle2
    
    
from utils.motor import Motor
from utils.yolo import YOLO


class RobotStateMachine:
    def __init__(self, robot, yolo):
        self.robot = robot
        self.yolo = yolo
        self.state = "init"
        self.rotate_direction = "super_left"
        self.screen_center_x = 320  # 화면 중앙 값, 화면의 가로 해상도에 따라 조정 필요

    def update(self):
        if self.state == "init":
            self.robot.move_front([1, 2, 4, 5, 7, 8, 10, 11], step=0, delay=0.5)
            self.robot.turn_start_left([1,2,4,5,7,8,10,11], delay=0.5)
            self.state = "detect"

        elif self.state == "detect":
            detection = self.yolo.detect()
            if detection:
                self.target_point = detection
                self.state = "adjust_rotation"
            else:
                # 타겟을 탐지할 때까지 회전
                if self.rotate_direction == "right":
                    self.robot.turn_right()
                else:
                    self.robot.turn_super_left()

        elif self.state == "adjust_rotation":
            detection = self.yolo.detect()  # 상태 갱신을 위한 새로운 탐지
            if detection:
                self.target_point = detection
                center_x = self.target_point
                if abs(center_x - self.screen_center_x) > 80:
                    if center_x < self.screen_center_x:
                        self.robot.turn_left()
                    else:
                        self.robot.turn_right()
                else:
                    self.state = "move_forward"
            else:
                self.state = "detect"  # 타겟을 잃어버렸을 경우 다시 탐지 상태로 돌아감

        elif self.state == "move_forward":
            detection = self.yolo.detect()  # 상태 갱신을 위한 새로운 탐지
            if detection:
                self.target_point = detection
                center_x = self.target_point
                # 타겟이 화면 중앙에 유지되도록 조정
                if abs(center_x - self.screen_center_x) > 160:
                    self.state = "adjust_rotation"
                else:
                    self.robot.move_front()
            else:
                self.state = "detect"  # 타겟을 잃어버렸을 경우 다시 탐지 상태로 돌아감

    def run(self):
        while True:
            self.update()

'''
Init : 초기 상태로 돌림
Detect : 현재 카메라에서 물체를 탐지해서 탐지 되면 Center Point를 줌, 탐지 안되면 None을 줌.
Adjust Rotation : 지정된 범위 안에 타겟이 오도록 회전함
Move Forward : 앞으로 이동함

Init → Detect

Detect → Adjust Rotation (if target detected)
Detect → Rotate Left/Right (if no target detected, continue in Detect)

Adjust Rotation → Move Forward (if target centered)
Adjust Rotation → Detect (if no target detected)

Move Forward → Adjust Rotation (if target deviates from center)
Move Forward → Detect (if no target detected)

'''

# 로봇 인스턴스를 생성하고 상태 머신 실행
robot = Motor()  # Robot 클래스는 실제 로봇의 API를 연결하는 코드를 포함해야 합니다.
yolo = YOLO()
state_machine = RobotStateMachine(robot, yolo)
state_machine.run()

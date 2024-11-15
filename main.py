from utils.motor import Motor
import time
import cv2

def main():
    motor = Motor()
    angles = motor.get_step_angle()
    motor.initialize_position_limits()
    print(angles)
    motor.run_motor_step([1, 7, 2, 8], 0)
    motor.run_motor_step([4, 10, 5, 11], 1)


    time.sleep(1)
    while True:
        # motor.run_motor_step([4, 5], 2, delay=0.1)
        # motor.run_motor_step([4, 5], 3, delay=0.1)
        # motor.run_motor_step([4, 5], 4, delay=0.3)
        # motor.run_motor_step([11, 10], 2, delay=0.1)
        # motor.run_motor_step([11, 10], 3, delay=0.1)
        # motor.run_motor_step([11, 10], 4, delay=0.3)
        motor.run_motor_step([4, 5, 11, 10], 2, delay=0.1)
        motor.run_motor_step([4, 5, 11, 10], 3, delay=0.1)
        motor.run_motor_step([4, 5, 11, 10], 4, delay=1)

        motor.run_motor_step([1, 2, 7, 8], 1)
        motor.run_motor_step([4, 5, 10, 11], 0, delay=1)

        # motor.run_motor_step([2, 1], 2, delay=0.1)
        # motor.run_motor_step([2, 1], 3, delay=0.1)
        # motor.run_motor_step([2, 1], 4, delay=0.3)
        # motor.run_motor_step([7, 8], 2, delay=0.1)
        # motor.run_motor_step([7, 8], 3, delay=0.1)
        # motor.run_motor_step([7, 8], 4, delay=0.3)
        motor.run_motor_step([2, 1, 7, 8], 2, delay=0.1)
        motor.run_motor_step([2, 1, 7, 8], 3, delay=0.1)
        motor.run_motor_step([2, 1, 7, 8], 4, delay=1)

        motor.run_motor_step([1, 2, 7, 8], 0)
        motor.run_motor_step([4, 5, 10, 11], 1, delay=1)
        
        


if __name__ == '__main__':
    main()
from utils.motor import Motor
import time
import cv2

def main():
    motor = Motor()
    angles = motor.convert_positions_to_angles()
    # motor.initialize_position_limits()
    print(angles)
    


    # while True:
    #     motor.run_motor_step([4, 5], 2, delay=0.1)
    #     motor.run_motor_step([4, 5], 3, delay=0.1)
    #     motor.run_motor_step([4, 5], 4, delay=1)
    #     motor.run_motor_step([10, 11], 2, delay=0.1)
    #     motor.run_motor_step([10, 11], 3, delay=0.1)
    #     motor.run_motor_step([10, 11], 4, delay=1)


    #     motor.run_motor_step([1, 2, 7, 8], 1)
    #     motor.run_motor_step([4, 5, 10, 11], 0, delay=1)


    #     motor.run_motor_step([7, 8], 2, delay=0.1)
    #     motor.run_motor_step([7, 8], 3, delay=0.1)
    #     motor.run_motor_step([7, 8], 4, delay=1)
    #     motor.run_motor_step([2, 1], 2, delay=0.1)
    #     motor.run_motor_step([2, 1], 3, delay=0.1)
    #     motor.run_motor_step([2, 1], 4, delay=1)


    #     motor.run_motor_step([1, 2, 7, 8], 0)
    #     motor.run_motor_step([4, 5, 10, 11], 1, delay=1)
    while True:
        for i in range(18):
            motor.run_motor_step([1,2,4,5,7,8,10,11], i, delay=0.5)
        


if __name__ == '__main__':
    main()
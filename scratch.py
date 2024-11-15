from utils.motor import Motor
import time

def main():
    motor = Motor()
    motor_list = [1,2,3]
    while True:
        angle4 = motor.read_angle(motor_list[0])
        angle5 = motor.read_angle(motor_list[1])
        angle6 = motor.read_angle(motor_list[2])

        print(f"{motor_list[0]}: {angle4}, {motor_list[1]}: {angle5}, {motor_list[2]}: {angle6}")
        time.sleep(5)
    # motor.run_motor(6,500)
    # # motor.set_position_limits(6, 500, 1500)
    # motor.run_motor(6,-100)


if __name__ == '__main__':
    main()
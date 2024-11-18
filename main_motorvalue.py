from utils.motor import Motor
import time
import cv2

def main():
    motor = Motor()

    big_current_angle = 2048 + -200
    small_current_angle = 2048 + -420
    big_motor_id = 10
    
    print(f'========== Motor index : {big_motor_id} ===========')
    print(f'Big motor angle: {big_current_angle}, Small motor angle: {small_current_angle}')
    vertical_distance, horizontal_distance = motor.angle_to_distance(motor_id=big_motor_id, raw_angle1=big_current_angle, raw_angle2=small_current_angle)
    print(f'Vertical distance: {vertical_distance}, Horizontal distance: {horizontal_distance}')
    
    raw_angle1, raw_angle2 = motor.distance_to_angle(motor_id=big_motor_id, vertical_distance=vertical_distance, horizontal_distance=horizontal_distance)
        
    print(f'Big motor angle: {raw_angle1}, Small motor angle: {raw_angle2}')

if __name__ == '__main__':
    main()
from Drone import Drone
from djitellopy import Tello
import cv2
import numpy as np
import time

def move_default(tello, frame_read):
    print('move')
    drone = Drone(tello, frame_read)
    drone.start_auto_move_thread()     # 自動
    # drone.start_manual_move_thread() # 手動
    print('video')
    drone.show_video()

def move_linetrace(tello, frame_read):
    drone = Drone(tello, frame_read)
    drone.start_linetrace_move()

def main():
    tello = Tello()
    tello.connect()
    tello.streamon()
    frame_read = tello.get_frame_read()

    select_move = input('1.default 2.linetrace >')
    if select_move == '1':
        move_default(tello, frame_read)
    elif select_move == '2':
        move_linetrace(tello, frame_read)
    else:
        print('予期しない入力です')

    del tello

if __name__ == "__main__":
    print('started')
    main()
    print('finish')
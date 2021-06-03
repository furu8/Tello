from djitellopy import Tello
import time
from datetime import datetime
import csv
from threading import Thread

class Sensor:

    def __init__(self, tello, name):
        # インスタンス
        self.tello = tello

        # 実行中のコマンド
        self.status = 'cmd'
        self.state = 'command'
        self.ord_yaw = 0
        # header
        self.header = ['datetime', 'status', 'state', 'mpry1', 'mpry2', 'mpry3',
                    'mid', 'x', 'y', 'z', 'pitch', 'roll', 'yaw', 'ord_yaw',
                    'agx', 'agy', 'agz', 'vgx', 'vgy', 'vgz', 
                    'templ', 'temph', 'tof', 'h', 'bat', 'baro', 'time']

        self.sensor_dict = {} # 保存先辞書

        # センサ保存用スレッド
        self.save_thread = Thread(target=self.save_tello_sensor, args=(name,))
        self.save_thread.daemon = True
        self.save_thread.start()

    def save_tello_sensor(self, filename):
        """
        filename : 現在時刻
        """
        path = '../../data/raw/'+ filename + '.csv'
        flag = True
        
        while True:
            time.sleep(0.1)
            state_dict = self.tello.get_current_state()
            self.__adjust_tello_sensor(state_dict)
            self.__write_tello_sensor(path, self.header, self.sensor_dict, flag)
            if flag:
                flag = False

    def __adjust_tello_sensor(self, state_dict):
        """
        djitellopyから取得できるデータを整形している
        """
        now = str(datetime.now().strftime('%Y-%m-%d %H:%M:%S:%f')) # 現在時刻
        mpry_list = state_dict['mpry'].split(',') # mpry整形用

        # 日時,status,state,ord_yaw,mpryは先に辞書に登録
        self.sensor_dict['datetime'] = now
        self.sensor_dict['status'] = self.status
        self.sensor_dict['state'] = self.state
        self.sensor_dict['ord_yaw'] = self.ord_yaw
        for i, mpry in enumerate(mpry_list):
            self.sensor_dict['mpry'+str(i+1)] = mpry_list[i]

        # 残りのセンサを記録
        for state in state_dict:
            self.sensor_dict[state] = state_dict[state]
        
        # mpryを削除
        self.sensor_dict.pop('mpry')

    def __write_tello_sensor(self, path, header, value_dict, flag):
        with open(path, mode='a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=header)
            if flag:
                writer.writeheader()
            writer.writerow(value_dict)

# テスト
if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    tello.streamon()

    print('started')
    sensor = Sensor(tello, str(datetime.now().strftime('%Y-%m-%d_%H-%M-%S')))
    time.sleep(3)
    sensor.state = 'takeoff'
    tello.takeoff()
    sensor.state = 'forward'
    tello.move_forward(200)
    sensor.state = 'cw'
    tello.rotate_clockwise(180)
    sensor.state = 'forward'
    tello.move_forward(200)
    sensor.state = 'land'
    tello.land()
    time.sleep(1) # 保存用に1秒待つ
    print('finish')
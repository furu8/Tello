# coding=utf-8
from Sensor import Sensor
from Recorder import Recorder
from LineTrace import LineTrace
from datetime import datetime
from threading import Thread
import sys
# import termios # macでしか使えない
import time
import cv2


class Drone:

    def __init__(self, tello, frame_read):
        # 記録用時刻
        self.name = str(datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
        
        # Telloインスタンス
        self.tello = tello
        # ビデオフレーム取得
        self.frame_read = frame_read

        # Sensorインスタンス
        self.sensor = Sensor(tello, self.name)

        # Recorderインスタンス
        self.recorder = Recorder(tello, self.frame_read, self.name)

        # 強制着陸フラグ
        self.isforceland = False

        # すべての動作終了フラグ
        self.ismotion = False

    # オート飛行動作用スレッド
    def start_auto_move_thread(self):
        self.auto_move_thread = Thread(target=self.auto_move)
        self.auto_move_thread.daemon = True
        self.auto_move_thread.start()

    # マニュアル飛行動作用スレッド
    def start_manual_move_thread(self):
        self.manual_move_thread = Thread(target=self.manual_move)
        self.manual_move_thread.daemon = True
        self.manual_move_thread.start()

    # ライントレース飛行動作用スレッド
    def start_linetrace_move(self):
        print('linetrace')
        linetrace = LineTrace(self, self.tello, self.frame_read)
        linetrace.start_auto_linetrace_thread()
        print('video')
        linetrace.show_linetrace()

    # 飛行中のビデオ描画
    # cv2.imshowで表示するGUIがメインじゃないと怒られるため、mainで映像を呼び出し
    # https://teratail.com/questions/194282
    def show_video(self):
        while True:
            ret, frame = self.frame_read.grabbed, self.frame_read.frame
            key = cv2.waitKey(1)
            if ret:
                cv2.imshow('frame', frame)
            if key == ord('l'):
                self.isforceland = True
            if key == 27 or self.ismotion: 
                break

        cv2.destroyWindow('frame')
    
    # 飛行計画から動かす
    def auto_move(self):
        _ = 'key' # key使わないので適当に

        # 飛行計画読み込み
        commands = self.read_FlightPlan('FlightPlan/command_default.txt')

        # コマンドを判定して投げる
        for command in commands:
            # コマンドの合間以外のlandは受け付けない（forward中に強制着陸命令してもforwardが終わった後に着陸する）
            # もしforward中に強制着陸したいのであれば、rcコマンドを使って前進するように改良するしかない（linetraceと同じ要領）
            if self.isforceland:
                self.forced_land_command()
                break
            cmd = command.replace('\n','').split(' ')[0] # 改行文字を消して分割
            try:
                xr = int(command.split(' ')[1])
                self.separate_flight_command(cmd, _, xr, xr)
            except:
                # 例外はtakeoff、landなどがある
                self.separate_flight_command(cmd, _, 0, 0)

        time.sleep(2) # 少し待って動作を終了したいため
        self.ismotion = True
    
    # コマンドからいつでも強制着陸
    def forced_land_command(self):
        print('forced land')
        self.sensor.state = 'land'
        self.tello.land()                        # 着陸
    
    def manual_move(self):
        _ = 'cmd' # cmd使わないので適当に
        x = 50    # 20cm
        r = 20    # 20度

        while True:
            key = ord(self.get_key())
            if key == 27: # keyが27(ESC)だったらwhileループを脱出，プログラム終了
                break
            self.separate_flight_command(_, key, x, r)

    def separate_flight_command(self, cmd, key, x, r):
        if cmd == 'takeoff' or key == ord('t'): 
            self.sensor.state = 'takeoff'
            self.tello.takeoff()                     # 離陸
        elif cmd == 'land' or key == ord('l'):
            self.sensor.state = 'land'
            self.tello.land()                        # 着陸
        elif cmd == 'forward' or key == ord('w'):
            self.sensor.state = 'forward'
            self.tello.move_forward(x)               # 前進
        elif cmd == 'backward' or key == ord('s'):
            self.sensor.state = 'backward'
            self.tello.move_back(x)                  # 後進
        elif cmd == 'left' or key == ord('a'):
            self.sensor.state = 'left'
            self.tello.move_left(x)                  # 左移動
        elif cmd == 'right' or key == ord('d'):
            self.sensor.state = 'right'
            self.tello.move_right(x)                 # 右移動
        elif cmd == 'ccw' or key == ord('q'):
            self.sensor.state = 'ccw'
            self.tello.rotate_counter_clockwise(r)   # 左旋回
        elif cmd == 'cw' or key == ord('e'):
            self.sensor.state = 'cw'
            self.tello.rotate_clockwise(r)           # 右旋回
        elif cmd == 'up' or key == ord('r'):
            self.sensor.state = 'up'
            self.tello.move_up(x)                    # 上昇
        elif cmd == 'down' or key == ord('f'):
            self.sensor.state = 'down'
            self.tello.move_down(x)                  # 下降
        elif cmd == 'speed':
            self.tello.set_speed(x)

    def get_key(self):
        """
        注意
            CUI上の入力であること
            Macでしか使えない
        """
        # 標準入力のファイルディスクリプタを取得
        fd = sys.stdin.fileno()

        # fdの端末属性をゲットする。oldとnewには同じものが入る。
        old = termios.tcgetattr(fd) # newに変更を加えて、適応する
        new = termios.tcgetattr(fd) # oldは、後で元に戻すため

        # new[3]はlflags
        new[3] &= ~termios.ICANON # ICANON(カノニカルモードのフラグ)を外す
        new[3] &= ~termios.ECHO # ECHO(入力された文字を表示するか否かのフラグ)を外す

        try:
            # 書き換えたnewをfdに適応する
            termios.tcsetattr(fd, termios.TCSANOW, new)
            # キーボードから入力を受ける。lfalgsが書き換えられているので、エンターを押さなくても次に進む。echoもしない
            key = sys.stdin.read(1)
        finally:
            # fdの属性を元に戻す。具体的にはICANONとECHOが元に戻る
            termios.tcsetattr(fd, termios.TCSANOW, old)

        return key
    
    # 飛行計画読み込み
    def read_FlightPlan(self, path):
        with open(path, 'r') as f:
            commands = f.readlines()
        return commands

# テスト
if __name__ == "__main__":
    from djitellopy import Tello
    tello = Tello()
    tello.connect()
    tello.streamon()

    # print('started')
    # cv2.namedWindow("OpenCV Window") # waitkeyのためだけに書いてるので画面に何も出なくてよい
    drone = Drone(tello, tello.get_frame_read())
    # drone.auto_move()
    drone.manual_move()
    time.sleep(1) # 保存用に1秒待つ
    print('finish')
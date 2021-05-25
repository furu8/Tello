from djitellopy import Tello
from threading import Thread
from datetime import datetime
import time
import cv2

class Recorder:

    def __init__(self, tello, frame_read, name):
        # インスタンス
        self.tello = tello
        # ビデオフレーム
        self.frame_read = frame_read

        # ビデオ保存用スレッド
        self.record_thread = Thread(target=self.record_video, args=(name,))
        self.record_thread.daemon = True
        self.record_thread.start()

    def record_video(self, filename):
        """
        filename : 現在時刻
        """
        path = '../../../video/' + filename + '.avi'
        FPS = 30

        height, width, _ = self.frame_read.frame.shape
        video = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'XVID'), FPS, (width, height))

        while True:
            video.write(self.frame_read.frame)
            time.sleep(1 / 30)

        # 書き込み完了して閉じる
        video.release()

# テスト
if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    tello.streamon()

    print('started')
    re = Recorder(tello, tello.get_frame_read())
    tello.takeoff()
    tello.land()
    time.sleep(1) # 保存用に1秒待つ
    print('finish')
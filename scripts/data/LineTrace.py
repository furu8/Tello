# coding=utf-8
from djitellopy import Tello
from threading import Thread
import numpy as np
import cv2
import time
import sys
from decimal import Decimal, ROUND_HALF_UP

class LineTrace():

    def __init__(self, drone, tello, frame_read):
        # ドローンインスタンス
        self.drone = drone
        # Telloインスタンス
        self.tello = tello
        # ビデオフレーム取得
        self.frame_read = frame_read

        # イメージ類
        self.org_image = None      # オリジナル画像
        self.small_image = None    # 画像サイズを半分にリサイズした画像
        self.bgr_image = None      # 注目する領域(ROI)を(0,250)-(479,359)で切り取った画像
        self.hsv_image = None      # BGR画像をHSV画像に変換した画像
        self.bin_image = None      # 2値化した画像
        self.land_bin_image = None # 着陸用画像
        self.dilation_image = None # 膨張した画像
        self.erosion_image = None  # 圧縮した画像
        self.masked_image = None   # マスクをかけた画像

        # ウィンドウ生成
        cv2.namedWindow("OpenCV Window")

        # トラックバーの生成
        cv2.createTrackbar("H_min", "OpenCV Window", 30, 179, self.__change_hsv)
        cv2.createTrackbar("H_max", "OpenCV Window", 90, 179, self.__change_hsv)     # Hueの最大値は179
        cv2.createTrackbar("S_min", "OpenCV Window", 128, 255, self.__change_hsv)
        cv2.createTrackbar("S_max", "OpenCV Window", 255, 255, self.__change_hsv)
        cv2.createTrackbar("V_min", "OpenCV Window", 50, 255, self.__change_hsv)
        cv2.createTrackbar("V_max", "OpenCV Window", 255, 255, self.__change_hsv)

        # rcコマンドの初期値
        self.left_right_velocity = 0        # 原則値は変更しない
        self.forward_backward_velocity = 30 # ライントレースの速度に応じて調整
        self.up_down_velocity = 0           # 原則値は変更しない
        self.yaw_velocity = 0               # ラインに応じて適宜回転させる

        # ライントレース実行フラグ
        self.islinetrace = False

        # 着陸フラグ
        self.island = False      # 着陸マーカー検知
        self.isforceland = False # 強制着陸

        # すべての動作終了フラグ
        self.ismotion = False

    # イニシャライザ関係のメソッド-----------------------------------------------------------------------------
    
    def __change_hsv(self, _):
        """
        入力
        _：引数がないとエラーが出るため用意
        """
        h_min = cv2.getTrackbarPos("H_min", "OpenCV Window")
        h_max = cv2.getTrackbarPos("H_max", "OpenCV Window")
        s_min = cv2.getTrackbarPos("S_min", "OpenCV Window")
        s_max = cv2.getTrackbarPos("S_max", "OpenCV Window")
        v_min = cv2.getTrackbarPos("V_min", "OpenCV Window")
        v_max = cv2.getTrackbarPos("V_max", "OpenCV Window")
        # 辞書に記録
        self.__register_hsv_dict(h_min, h_max, s_min, s_max, v_min, v_max)

    def __register_hsv_dict(self, h_min, h_max, s_min, s_max, v_min, v_max):
        self.hsv_dict = {}
        self.hsv_dict['h_min'] = h_min
        self.hsv_dict['h_max'] = h_max
        self.hsv_dict['s_min'] = s_min
        self.hsv_dict['s_max'] = s_max
        self.hsv_dict['v_min'] = v_min
        self.hsv_dict['v_max'] = v_max

    # オート飛行動作用スレッド
    def start_auto_linetrace_thread(self):
        self.auto_linetrace_thread = Thread(target=self.auto_linetrace)
        self.auto_linetrace_thread.daemon = True
        self.auto_linetrace_thread.start()

    # 実行メソッド----------------------------------------------------------------------------

    # メインスレッドじゃないとGUI画面が表示されない?
    def show_linetrace(self):
        while True:
            # フレーム読み込み
            ret, frame = self.frame_read.grabbed, self.frame_read.frame
            if not ret:
                continue
            # keyが27(ESC)だったらwhileループを脱出，プログラム終了
            key = cv2.waitKey(1)
            if key == ord('l'):
                self.isforceland = True
            if key == 27 or self.ismotion: 
                break

            # トラックバーの値を取る
            self.__change_hsv('_')
            # 膨張画像を取得
            self.convert_img(frame)
            # 認識した画像をラベリング
            label_image = self.dilation_image # ここでイメージを自由に選ぶ（適宜変更する）
            land_label_image = self.land_dilation_image
            self.num_labels, label_image, self.stats, self.center = self.labeling_imgs(label_image)
            land_num_labels, land_label_image, land_stats, land_center = self.labeling_imgs(land_label_image) # 着陸検出
            # ラベリング結果書き出し用に画像を準備
            edit_output_image = self.bgr_image # ここでイメージを自由に選ぶ（適宜変更する）
            # 出力用画像編集
            if self.num_labels >= 1:
                edit_output_image = self.__edit_output_imgs(edit_output_image, self.stats, self.center)
                if land_num_labels >= 1:
                    self.island = True
                    edit_output_iamge = self.__edit_output_imgs(edit_output_image, land_stats, land_center)

            # 画像をPCの画面に表示
            self.show_img(edit_output_image)

    def auto_linetrace(self):
        pre_time = time.time()     # 定期コマンド送信用の時間を取得
        _ = 'key' # key使わないので適当に

        # 飛行計画読み込み
        commands = self.drone.read_FlightPlan('FlightPlan/command_linetrace.txt')

        for command in commands:
            print(command)
            # 強制着陸
            if self.isforceland:
                self.drone.forced_land_command()
                break

            # 飛行計画のコマンド
            cmd = command.replace('\n','').split(' ')[0] # 改行文字を消して分割
            try:
                xr = int(command.split(' ')[1])
                self.separate_linetrace_flight_command(cmd, _, xr, xr)
            except:
                # 例外はtakeoff、landなどがある
                self.separate_linetrace_flight_command(cmd, _, 0, 0)
                
            # ライントレース
            if self.islinetrace:
                while True:
                    # ラインを隠したときのデータと風のデータの差分の違いを明らかにする場合、try/exceptは不要
                    try: 
                        # マーカーで着陸
                        if self.island: 
                            time.sleep(0.001) # ここで少しだけ止めないと、なぜかmove_off_linetrace()が実行されない（受付限界TIME_BTW_RC_CONTROL_COMMANDSが0.001秒のせいっぽいから、0.001秒以上スリープすれば良さげ）
                            # ライントレース終了
                            self.move_off_linetrace()
                            print('finished linetrace') 
                            break
                        # ラインを検知してライントレース実行
                        elif self.num_labels >= 1:
                            time.sleep(0.1) # default_moveと差異が生まれている原因の一つかも
                            # 強制着陸
                            if self.isforceland:
                                self.move_off_linetrace()
                                break
                            # ライントレース実行
                            self.move_on_linetrace(self.stats, self.center)
                    except:
                        # 何らかの問題によるライントレース終了(マーカーとラインがどちらも検知できない)
                        self.move_off_linetrace() # これがないと次の実行時にtakeoff以降の命令を聞いてくれなくなる
                        print('except finished linetrace')
                        break

                self.islinetrace = False

        time.sleep(2) # 少し待って動作を終了したいため
        self.ismotion = True

    def manual_linetrace(self):
        """
        https://qiita.com/hsgucci/items/4a5aa35aa9fd036621a7
        """
        pre_time = time.time()     # 定期コマンド送信用の時間を取得
        _ = 'cmd' # cmd使わないので適当に
        x = 50    # 距離
        r = 180   # 角度

        while True:
            # フレーム読み込み
            ret, frame = self.frame_read.grabbed, self.frame_read.frame
            if not ret:
                continue
            # keyが27(ESC)だったらwhileループを脱出，プログラム終了
            key = cv2.waitKey(1)
            if key == 27: 
                break

            # トラックバーの値を取る
            self.__change_hsv('_')
            # 膨張画像を取得
            self.convert_img(frame)
            # 認識した画像をラベリング
            label_image = self.dilation_image # ここでイメージを自由に選ぶ（適宜変更する）
            num_labels, label_image, stats, center = self.labeling_imgs(label_image)
            # ラベリング結果書き出し用に画像を準備
            edit_output_image = self.masked_image # ここでイメージを自由に選ぶ（適宜変更する）
            # 出力用画像編集
            if num_labels >= 1:
                edit_output_image = self.__edit_output_imgs(edit_output_image, stats, center)

            # 画像をPCの画面に表示
            self.show_img(edit_output_image)

            # ライントレース前（手動）
            self.separate_linetrace_flight_command(_, key, x, r)
            # ライントレース
            if num_labels >= 1 and self.islinetrace:
                self.drone.sensor.status = 'linetrace'
                self.move_on_linetrace(stats, center)
                
            # 動作が停止しないように定期的にコマンドを送る
            pre_time = self.__send_constant_command(pre_time)

    # 画像認識関係のメソッド-----------------------------------------------------------------------------
    
    def convert_img(self, frame):
        """
        入力
        frame：変換したい画像
            https://qiita.com/hsgucci/items/e9a65d4fa3d279e4219e
        """
        LAND_H_MIN, LAND_H_MAX = 140, 179 # 着陸マーカーの検知範囲

        self.org_image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)           # OpenCV用のカラー並びに変換する
        self.small_image = cv2.resize(self.org_image, dsize=(480, 360))   # 画像サイズを半分に変更
        self.bgr_image = self.small_image[250:359, 0:479]                 # 注目する領域(ROI)を(0,250)-(479,359)で切り取る
        self.hsv_image = cv2.cvtColor(self.bgr_image, cv2.COLOR_BGR2HSV)  # BGR画像 -> HSV画像

        # inRange関数で範囲指定２値化
        self.bin_image = cv2.inRange(self.hsv_image, 
                                (self.hsv_dict['h_min'], self.hsv_dict['s_min'], self.hsv_dict['v_min']), 
                                (self.hsv_dict['h_max'], self.hsv_dict['s_max'], self.hsv_dict['v_max'])) # HSV画像なのでタプルもHSV並び

        # 赤色（ピンク色）を検知して着陸
        self.land_bin_image = cv2.inRange(self.hsv_image, 
                                (LAND_H_MIN, self.hsv_dict['s_min'], self.hsv_dict['v_min']), 
                                (LAND_H_MAX, self.hsv_dict['s_max'], self.hsv_dict['v_max'])) # HSV画像なのでタプルもHSV並び

        # ラインの膨張と圧縮
        self.dilation_image, self.erosion_image = self.__change_morphology(self.bin_image) # 必ずしも使う必要性はない
        self.land_dilation_image, self.land_erosion_image = self.__change_morphology(self.land_bin_image) # 必ずしも使う必要性はない

        # bitwise_andで元画像にマスクをかける -> マスクされた部分の色だけ残る
        mask_iamge = self.dilation_image # ここでイメージを自由に選ぶ（適宜変更する）
        self.masked_image = cv2.bitwise_and(self.hsv_image, self.hsv_image, mask=mask_iamge)

    def __change_morphology(self, mol_image):
        """
        入力
        mol_image：モルフォロジー変換したい画像
        isdelate：圧縮するかどうかboolフラグ
            http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
        """
        kernel = np.ones((15,15),np.uint8)  # 15x15カーネル

        dilation_image = cv2.dilate(mol_image, kernel, iterations=1) # 膨張
        erosion_image = cv2.erode(mol_image, kernel, iterations=1)   # 収縮

        return dilation_image, erosion_image

    def labeling_imgs(self, label_image):
        """
        入力
        label_image：ラベリングしたい画像
        出力
        ラベリングした画像の詳細情報（https://dronebiz.net/tech/opencv/labeling）
        """
        # 面積・重心計算付きのラベリング処理を行う
        num_labels, label_image, stats, center = cv2.connectedComponentsWithStats(label_image)

        # 最大のラベルは画面全体を覆う黒なので不要．データを削除
        num_labels = num_labels - 1
        stats = np.delete(stats, 0, 0)
        center = np.delete(center, 0, 0)

        return num_labels, label_image, stats, center

    def __edit_output_imgs(self, output_image, stats, center):
        # statsパラメータ取得
        x, y, w, h, s, mx, my = self.__get_stats_params(stats, center)

        # ラベルを囲うバウンディングボックスを描画
        cv2.rectangle(output_image, (x, y), (x+w, y+h), (255, 0, 255))

        # 検出したオブジェクトの中心点（赤点）
        cv2.circle(output_image, (mx, my), 5, (0,0,255), thickness=-1)

        # 画面の中心点描画（黒点）
        cv2.circle(output_image, (240, 55), 5, (0,0,0), thickness=-1)

        # 重心位置の座標を表示
        cv2.putText(output_image, "%d"%(s), (x, y+h+15), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0))

        return output_image

    def __get_stats_params(self, stats, center):
        # 面積最大のインデックスを取得
        max_index = np.argmax(stats[:,4])

        # 面積最大のラベルのx,y,w,h,面積s,重心位置mx,myを得る
        x = stats[max_index][0]
        y = stats[max_index][1]
        w = stats[max_index][2]
        h = stats[max_index][3]
        s = stats[max_index][4]
        mx = int(center[max_index][0])
        my = int(center[max_index][1])
        
        # print("(x,y)=%d,%d (w,h)=%d,%d s=%d (mx,my)=%d,%d"%(x, y, w, h, s, mx, my))
        return x, y, w, h, s, mx, my

    def show_img(self, image):
        cv2.imshow('OpenCV Window', image)

    # 飛行動作関係のメソッド-----------------------------------------------------------------------------

    def separate_linetrace_flight_command(self, cmd, key, x, r):
        if cmd == 'takeoff' or key == ord('t'): 
            self.drone.sensor.state = 'takeoff'
            self.drone.sensor.status = 'cmd'
            self.tello.takeoff()                     # 離陸
        elif cmd == 'land' or key == ord('l'):
            self.drone.sensor.state = 'land'
            self.drone.sensor.status = 'cmd'
            self.tello.land()                        # 着陸
        elif cmd == 'forward' or key == ord('w'):
            self.drone.sensor.state = 'forward'
            self.drone.sensor.status = 'cmd'
            self.tello.move_forward(x)               # 前進
        elif cmd == 'backward' or key == ord('s'):
            self.drone.sensor.state = 'backward'
            self.drone.sensor.status = 'cmd'
            self.tello.move_back(x)                  # 後進
        elif cmd == 'left' or key == ord('a'):
            self.drone.sensor.state = 'left'
            self.drone.sensor.status = 'cmd'
            self.tello.move_left(x)                  # 左移動
        elif cmd == 'right' or key == ord('d'):
            self.drone.sensor.state = 'right'
            self.drone.sensor.status = 'cmd'
            self.tello.move_right(x)                 # 右移動
        elif cmd == 'ccw' or key == ord('q'):
            self.drone.sensor.state = 'ccw'
            self.drone.sensor.status = 'cmd'
            self.tello.rotate_counter_clockwise(r)   # 左旋回
        elif cmd == 'cw' or key == ord('e'):
            self.drone.sensor.state = 'cw'
            self.drone.sensor.status = 'cmd'
            self.tello.rotate_clockwise(r)           # 右旋回
        elif cmd == 'up' or key == ord('r'):
            self.drone.sensor.state = 'up'
            self.drone.sensor.status = 'cmd'
            self.tello.move_up(x)                    # 上昇
        elif cmd == 'down' or key == ord('f'):
            self.drone.sensor.state = 'down'
            self.drone.sensor.status = 'cmd'
            self.tello.move_down(x)                  # 下降
        elif cmd == 'speed':
            self.drone.sensor.state = 'adj_spped'
            self.drone.sensor.status = 'cmd'
            self.tello.set_speed(x)                  # 速度調整
        elif cmd == 'linetrace' or key == ord('1'): 
            self.islinetrace = True                      # ライントレースモードON
        elif key == ord('2'):
            self.islinetrace = False                     # ライントレースモードOFF
            self.move_off_linetrace()                    
        elif key == ord('y'):           
            self.change_forward_backward_velocity(True)  # 前進速度をキー入力で速度上昇
        elif key == ord('h'):
            self.change_forward_backward_velocity(False) # 前進速度をキー入力で速度下降

    def move_on_linetrace(self, stats, center):
        """
        左右旋回のself.yaw_velocityだけが変化する．
        前進速度のself.forward_backward_velocityはキー入力で変える
        （P制御）https://www.yukisako.xyz/entry/pid_control
        """
        # statsパラメータ取得
        _x, _y, _w, _h, _s, mx, _my = self.__get_stats_params(stats, center)

        # 回転角度を計算
        self.yaw_velocity = self.__calc_vec_angle_line(mx, _my)
        
        # 旋回方向のリミッタ±50を超えないように(ソフトウェアリミッタは±100)
        self.yaw_velocity =  100 if self.yaw_velocity >  100.0 else self.yaw_velocity
        self.yaw_velocity = -100 if self.yaw_velocity < -100.0 else self.yaw_velocity

        # statusとstateの更新
        self.drone.sensor.status = 'linetrace' 
        if self.yaw_velocity < -1: # ±1程度なら回転命令を与えなくても発生し得る
            self.drone.sensor.state = 'forward,ccw'
            self.drone.sensor.ord_yaw = self.yaw_velocity
        elif self.yaw_velocity > 1: # ±1程度なら回転命令を与えなくても発生し得る
            self.drone.sensor.state = 'forward,cw'
            self.drone.sensor.ord_yaw = self.yaw_velocity
        else:
            self.drone.sensor.state = 'forward'

        # rcコマンド
        self.tello.send_rc_control(int(self.left_right_velocity), int(self.forward_backward_velocity), int(self.up_down_velocity), int(self.yaw_velocity))

    def move_off_linetrace(self):
        self.tello.send_rc_control(0, 0, 0, 0)

    def change_forward_backward_velocity(self, isforward):
        if isforward:
            self.forward_backward_velocity = self.forward_backward_velocity + 10
        else:
            self.forward_backward_velocity = self.forward_backward_velocity - 10

    def __send_constant_command(self, pre_time):
        # 5秒おきに'command'を送って、死活チェックを通す
        current_time = time.time()           # 現在時刻を取得
        if current_time - pre_time > 5.0:    # 前回時刻から5秒以上経過しているか
            self.tello.connect()             # 'command'送信
            pre_time = current_time          # 前回時刻を更新

        return pre_time

    # 角度計算のメソッド-----------------------------------------------------------------------------

    # ラインの角度を計算
    def __calc_angle_line(self, org_mx, org_my, mx, my):
        # プラスマイナスを計算（角度の左右判定）
        plus_minus_sign = self.__calc_plus_minus_sign(org_mx, mx)

        my = my + 250 # org_mx,org_myとmx,myの原点を揃える処理（トリミング前と後で同じ原点にする処理）
        width = np.abs(org_mx - mx)
        hight = np.abs(org_my - my)
        tan_theta = hight / width
        theta = np.rad2deg(np.arctan(tan_theta))

        return self.__calc_round_off(theta * plus_minus_sign) # 符号を乗算して四捨五入

    # ラインの方向に向くために二点の角度計算
    def __calc_vec_angle_line(self, img_mx, img_my):
        """
        img_mx：画像のラインの重心点のX座標
        img_my：画像のラインの重心点のY座標（現状使っていない。厳密に計算するときには必要）
        """
        # 以下4つのパラメータはドローンが高さ30cmのときとする
        REAL_MY = 58
        REAL_WIDTH = 67
        IMAGE_WIDTH = 480
        IMAGE_CENTER = IMAGE_WIDTH / 2 # 画面の中点のx座標
        
        # プラスマイナスを計算（角度の左右判定）
        plus_minus_sign = self.__calc_plus_minus_sign(img_mx, IMAGE_CENTER)
        
        real_mx = REAL_WIDTH * (IMAGE_CENTER - img_mx) / IMAGE_WIDTH
        real_mx = self.__calc_round_off(real_mx) # 四捨五入

        point_calculated_vec = np.array([real_mx, REAL_MY]) # 計算して求めた座標
        point_center_vec = np.array([0, REAL_MY])           # 画面の中心点の座標（原点はドローンの位置（厳密にはドローンから真下に下ろした直線と床の接点の座標））
        
        # 成す角計算
        inner_product = np.inner(point_calculated_vec, point_center_vec) # 内積
        norm_product = np.linalg.norm(point_calculated_vec) * np.linalg.norm(point_center_vec) # 各ベクトルのノルム積
        cos_theta = inner_product / norm_product
        theta = np.rad2deg(np.arccos(np.clip(cos_theta, -1.0, 1.0))) # cosθからθを算出
        
        return self.__calc_round_off(theta * plus_minus_sign) # 符号を乗算して四捨五入

    # 引数のparamから正負判定する
    def __calc_plus_minus_sign(self, param1, param2):
        if (param1 - param2) >= 0:
            plus_minus_sign = 1
        else:
            plus_minus_sign = -1
        
        return plus_minus_sign
    
    # 四捨五入を計算
    def __calc_round_off(self, target_num):
        try:
            return int(Decimal(str(target_num)).quantize(Decimal('0'), ROUND_HALF_UP))
        except ValueError:
            print('ValueError')
            # target_numがNaNのとき
            return 0

if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    tello.streamon()
    frame_read = tello.get_frame_read()

    linetrace = LineTrace(tello, frame_read)
    linetrace.start_auto_linetrace_thread()
    # linetrace.manual_linetrace()
    linetrace.show_linetrace()
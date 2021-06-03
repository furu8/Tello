import numpy as np
import cv2
import time

class LineTrace4AR():
    def __init__(self, cap):
        self.cap = cap

        self.count = 0

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
    
    def show_linetrace(self):
        while True:
            # フレーム読み込み
            ret, frame = self.cap.read()
            print(ret)
            if not ret:
                self.count += 1
                if self.count > 5:
                    break
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
            time.sleep(1/60)

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

if __name__ == "__main__":
    video_path = '..//video/flight_from_another_video/2021-03-31_10-42-22.avi'
    
    cap = cv2.VideoCapture(video_path)

    linetrace = LineTrace4AR(cap)
    linetrace.show_linetrace()

    """メモ
    ・そのまま実行すると、映像が流れるのが早い
    ・これで飛行したときどんな挙動をするか確かめる
    """
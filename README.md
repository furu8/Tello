# Tello

## フォルダ構成
------------

    ├── README.md             <- The top-level README for developers using this project.
    ├── data                  <- Tello sensor data
    │   └── raw               <- The original, immutable data dump.
    │
    ├── requirements.txt      <- The requirements file for reproducing the tello flight environment
    │
    ├── scripts               <- Source code for use in this project.
    │   ├── data              <- Scripts to collect tello sensor data
    │   │   ├── djitellopy    <- https://github.com/damiafuentes/DJITelloPy
    │   │   ├── FlightPlan    <- Flight plan for tello
    │   │   ├── Drone.py
    │   │   ├── LineTrace.py
    │   │   ├── Main.py
    │   │   ├── Recorder.py
    │   │   └── Sensor.py
    │── video                 <- Tello video data
--------

## 環境構築

1. [1]からAnaconda3-2020.11-Windows-x86=64.exeをダウンロードする
2. インストーラーをNext押し続け（pathを通さないことを勧める）
3. タスクバーの虫眼鏡からAnaconda Promptと検索し、 起動する（以下、ターミナル=Anaconda Prompt）

    (なお、base環境はanaconda3の基底環境なので基本この環境で実行しない)

    ```
    (base) C:\Users\furuhama\OneDrive\document\Research\ADforDrone>
    ```

4. [2]などを参考に、ターミナルで`conda create -n tello python=3.7.9`として、Tello用の仮想環境を構築する
5. `conda activate tello`で仮想環境に入る
6. `pip install -r requirements.txt`を実行し、必要なライブラリ等を入れる
7. 以上でTello飛行用環境は準備完了

    ```
    (tello) C:\Users\furuhama\OneDrive\document\Research\ADforDrone>
    ```
8. なお、仮想環境を抜けるには`concda deactivate`で良い

## 実行方法

1. Telloを起動し、Wi-Fiの欄にある**Tello-5A8B98**を選ぶ（Telloの機種によってはハイフン以降変わるかも）
2. ターミナルで`python Main.py`で実行
3. 標準入力で1か2かを求められる
4. コマンドから飛行する簡易プログラムは1、ライントレース飛行プログラムは2を選ぶ（基本2）
5. 2を実行した時点で、ライントレースプログラムが開始される

    - プログラムの詳細はNASの`\\133.92.145.47\public\70. NCOS\共同研究資料\古浜\報告資料\2020-12-03\2020-12-03_Tello飛行プログラム_古濵_V2.pptx`にある
    - Telloに関して詳しく知りたい場合以下参照
      - https://qiita.com/hsgucci/items/3327cc29ddf10a321f3c


## 参考

- [1] Anaconda installer archive
  - https://repo.anaconda.com/archive/

- [2]【初心者向け】Anacondaで仮想環境を作ってみる
  - https://qiita.com/ozaki_physics/items/985188feb92570e5b82d

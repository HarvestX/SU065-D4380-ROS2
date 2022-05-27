* 説明用README

* ROS2→USBtoRS485モジュール→ドライバ を担うモジュール

* (本来は hardware_controller等を用いて書くべきなので時間があれば書き直す)

* API
  * 制御用
  * comnode.py
  * rs422node
    * /wheel_order_vel harvestx_odm モジュールで計算した回転数等を指示
    * subしているデータ
      * Int32MultiArray
        * self.agv.rightwheel.vel_cmd_f=msg.data[0]
          self.agv.leftwheel.vel_cmd_f=msg.data[1]
    * /wheel_read_vel
    * こちらからpubしてるデータ
      * Int32MultiArray
      * self.pubdata.data[0] = self.agv.rightwheel.speed
        self.pubdata.data[1] = self.agv.leftwheel.speed
        self.pubdata.data[2] = self.agv.drv_error
        self.pubdata.data[3] = self.agv.drv_status
      self.pubdata.data[4] = self.agv.drv_bat
    * 基本的には harvestx_odm ノード内部からpubされたトピックをsubしている.
    * また制御周りはまとめて harvestx_odm パッケージ内部で
    * ros2 launch harvestx_odm robot_launch.py
    * としている。

  * パラメータ書き換え用
      * paramkey.py
      * parammode.py
      * ２つのシェルで
        * ros2 run harvestx_com rs422_paramnode
        * ros2 run harvestx_com rs422_keynode
        * と起動し、keynodeの方での指示通りにパラメタを打ち込む
        * パラメタについてはドライバのマニュアル(/Driver_instruction内)参照



  * Odometryっぽいことをしているモジュール

  * harvestx_odm.py
  * robocon ノード
    * Twist である /cmd_vel を受け取って各輪の輪速に変換して司令
    * /wheel_order_vel を harvestx_com の　rs422comノードに対して指示
    * /wheel_read_vel 　を　上記のノードから受け取り、それから Odometry型である /nav_odm を robot_localization モジュールに指示
    *
    * ros2 launch harvestx_odm robot_launch.py
    * で起動
    * 車輪幅はself.widthにベタ書きでしかも正しくないので直してくださるとありがたいです。


    * ros2 launch harvestx_odm robot_launch.py
    * ros2 launch harvestx_joy joy_launch.py
    * でjoyconから動かせる。


* ロボットの動かし方
  * 赤いバッテリー用のスイッチを入れる（48V ON）
  * スイッチング電源に電源を投入する(SU065-D4380ドライバがONになり通信が始まる)
  * 白いドライバスイッチを入れる(モーターへの入力受付開始(これを先に入れるとたまに動かなくなることがある))
  * 通信用のノードを起動し通信を初めて入力する

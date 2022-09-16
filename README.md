# rt_usb_9axisimu_test

このリポジトリは株式会社アールティが販売している[USB 出力 9 軸 IMU センサモジュール](https://www.rt-net.jp/products/9axisimu2/)を ROS から使用するための[ドライバ](https://github.com/rt-net/rt_usb_9axisimu_driver)の動作を確認するためのものです．

動作確認は Ubuntu 20.04 上の ROS2 Foxy で行っており，[rt_usb_9axisimu_driver](https://github.com/rt-net/rt_usb_9axisimu_driver)の版は foxy-devel (2f7f417d200a77439189288bb985a2f7f25557b5)です．
[issue](https://github.com/rt-net/rt_usb_9axisimu_driver/issues/34)を参考に，[imu_complementary_filter](http://wiki.ros.org/imu_complementary_filter)を用いて姿勢を推定し，rviz2 に描画させて動作を確認しました．

## インストールと起動

ドライバの他に imu_complementary_filter，rviz2, rqt-gui を用います．
imu_complementary_filter は`sudo apt install ros-foxy-imu-tools`でインストールしました．

デバイスを PC に接続した直後の状態では`[ERROR] Error opening sensor device, please re-check your devices. `が発生するので`sudo chmod 666 /dev/ttyACM0`しておきます．

```
mkdir -p imu_test/src
cd imu_test/src
git clone git@github.com:Actat/rt_usb_9axisimu_test.git
git clone -b foxy-devel git@github.com:rt-net/rt_usb_9axisimu_driver.git
rosdep install -r -y -i --from-paths .
cd ..
colcon build
source install/setup.bash
ros2 launch rt_usb_9axisimu_test launch.py
```

rviz と rqt-gui のウインドウが開きます．
rviz ではデバイスの姿勢を変えると画面中の座標軸の向きが対応するように変化する様子を観察できます．
rqt-gui の Node Graph を見てデバイスで計測した加速度と角速度のデータである`/imu/data_raw`と`/imu/mag`が使用されていることを確認できます．
また，Plugins->Visualization->Plot を選択して MatPlot を開き，Topic に/imu/data/angular_velocity や/imu/data/linear_acceleration や/imu/mag/magnetic_field を入力して緑色に変化した+のボタンをクリックすれば計測された値を確認できます．

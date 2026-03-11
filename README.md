# khr2026_team1_hwmc

ロボットハンド機構系の CAN ブリッジ ESP32 ファームウェア。
ROS 2 の `hand_control` / `hand_feedback` トピック（HandMessage）を受け取り、CAN 1 系統でサブボード（yagura_dcmd / ring_svmd / crub_igniter）に指令を転送する。

## robot_msgs を変えたときは

ヘッダファイルを生成する必要があるので、以下のコマンドの順に実行する。

```bash
rm -rf .pio/libdeps/esp32dev/micro_ros_platformio/libmicroros
pio run -e esp32dev
```

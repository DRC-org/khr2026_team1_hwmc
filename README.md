# khr2026_team1_hwmc

## robot_msgs を変えたときは

ヘッダファイルを生成する必要があるので、以下のコマンドの順に実行する。

```bash
rm -rf .pio/libdeps/esp32dev/micro_ros_platformio/libmicroros
pio run -e esp32dev
```

## m3508_gains の送受信（メモ）

```cpp
// 送信する場合（チューニング時）
msg.m3508_gains.size = 1;
msg.m3508_gains.data[0] = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f};

// 受信側でのチェック
if (msg.m3508_gains.size > 0) {
    apply_pid_gains(msg.m3508_gains.data[0]);
}
```

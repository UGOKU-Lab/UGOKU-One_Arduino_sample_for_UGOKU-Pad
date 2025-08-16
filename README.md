# ESP32_Arduino_for_UGOKU_Pad
ESP32をUGOKU Padで動かすArduinoのサンプルプログラムです。

### UGOKU Pad
https://ugoku-lab.github.io/ugokupad.html

Console内の「ESP32 Arduino Sample」を使用

<img src="https://github.com/user-attachments/assets/a0c7ed43-5082-4802-9647-cbb8cc861142" width="200">
<img src="https://github.com/user-attachments/assets/578605c3-9ea8-434b-b564-59bf12aa8233" width="200">

### できること
- デジタル出力の操作
- RCサーボの操作
- ローテーションサーボの操作
- PSD測距モジュール(GP2Y0A21YK)による距離表示（アナログ入力）
- BMI270(IMU)のサンプル読み取り（加速度・ジャイロ）

### ピン配置
| 機能 | ピン |
| ------------- | ------------- |
| デジタル出力  | 27 |
| 測距モジュール | 26 |
| RCサーボ | 14 |
| ローテーションサーボ | 12 |
| I2C SDA | 21 |
| I2C SCL | 22 |

### 使用ライブラリ
- Arduino標準ライブラリ
- ESP32servo
- Arduino_BMI270_BMM150（Arduino公式 IMU ライブラリ）

インストール方法: Arduino IDE の「ライブラリを管理…」で「Arduino_BMI270_BMM150」を検索してインストール。

### 動作確認
ESP32-WROOM-32E、ESP32-WROVER-Eで動作確認済み

## BMI270 サンプルについて
- 本リポジトリには `BMI270Sample.hpp` を追加し、`UGOKU-One_Arduino_sample_for_UGOKU-Pad.ino` 内でコンパイル時フラグ `USE_BMI270` により有効化しています（デフォルト: 1）。
- 有効時は、シリアルモニタ(115200bps)に加速度[g]とジャイロ[dps]が約200ms間隔で表示されます。
- BMI270/BMM150（I2C）の接続例: VCC->3.3V, GND->GND, SDA->GPIO21, SCL->GPIO22。



# UGOKU-One Arduino Sample for UGOKU Pad
UGOKU One を UGOKU Pad から操作するための Arduino サンプルです。

## このサンプルでできること
- BLE 接続
- RC サーボの制御＋ DIP で反転
- モータードライバ制御＋ DIP で反転
- LED の ON/OFF
- 加速度の計測

<img src="https://github.com/user-attachments/assets/f735c3af-c777-4a1e-8846-5607c272b291" width="600">


## 使用方法
#### Arduino IDE での準備
- BOARD MANAGER で ESP32 を検索して **Arduino ESP32 Boards** と **esp32** をインストールする  
<img src="https://github.com/user-attachments/assets/1f6a2303-91af-4abe-82d2-41801bee747c" width="350">

　
- LIBRARY MANAGER で **ESP32Servo** を検索してインストールする  
<img src="https://github.com/user-attachments/assets/cae88e9d-53ed-4a1c-bf38-bd7de3d8b462" width="350">  

　
- Select Other Board and Port で **ESP32 Dev Module** を検索して選択する 
<img src="https://github.com/user-attachments/assets/670f19c9-2996-4bbd-9af5-0fa0ecb96540" width="350">  


## UGOKU Pad の設定
UGOKU Padをインストール

[<img src="https://github.com/user-attachments/assets/73952bbe-7f89-46e9-9a6e-cdc7eea8e7c8" alt="Get it on Google Play" height="60">](https://play.google.com/store/apps/details?id=com.ugoku_lab.ugoku_console)　[<img src="https://github.com/user-attachments/assets/e27e5d09-63d0-4a2e-9e14-0bb05dabd487" alt="App Store" height="60">](https://apps.apple.com/jp/app/ugoku-pad/id6739496098)

参考: https://ugoku-lab.github.io/ugokupad.html

## ピン配置
| 機能 | ピン | 備考 |
| --- | --- | --- |
| サーボ1 | 14 | 50Hz, attach(500–2500μs) |
| サーボ2 | 27 | 〃 |
| LED1/2/3 | 2 / 4 / 13 | アクティブLOW（LOWで点灯） |
| DIP（サーボ反転） | 34 | INPUT（入力専用ピン） |
| DIP（モーター反転） | 35 | INPUT（入力専用ピン） |
| I2C SDA / SCL | 21 / 22 | BMI270 用 |

## UGOKU Pad チャンネル対応
- ch2/4/13: LED1/2/3 の ON/OFF（1=ON, 0=OFF 相当。内部はアクティブLOW）
- ch14/27: サーボ1/2 角度 0..180（DIPで反転可能）
- ch17/19: モーター MD1/2（スティック中央=停止。0..255を-1..+1にスケーリング）
- ch100/101/102: 加速度の計測値



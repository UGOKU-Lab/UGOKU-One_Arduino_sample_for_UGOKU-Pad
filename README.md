# UGOKU-One Arduino Sample for UGOKU Pad
UGOKU One を UGOKU Pad から操作するための Arduino サンプルです。現在のコード構成・挙動に合わせて README を更新しています。

## このサンプルでできること
- BLE 接続（UGOKU Pad の Console「ESP32 Arduino Sample」を使用）
- RC サーボ 2ch の制御（ch2, ch3）＋ DIP で反転
- 2ch モータードライバ制御（ch4, ch5 独立）＋ DIP で反転
- LED 3個の一括 ON/OFF（ch1）
- 汎用デジタル出力 2本の ON/OFF（ch6）
- PSD 測距の値を ch10 に送信（簡易換算）
- BMI270（加速度・ジャイロのみ）からロール/ピッチ/ヨー角を推定し、ch20/21/22 に 0..180 で送信（平置き=約90）

<img src="https://github.com/user-attachments/assets/b2da444f-e0e3-46c4-aa92-2031e2f38083" width="600">

## 使用方法
#### Arduino IDE での準備
- BOARD MANAGER で ESP32 を検索して **Arduino ESP32 Boards** と **esp32** をインストールする  
<img src="https://github.com/user-attachments/assets/34e671e7-9068-47e4-8431-86b137ea8c13" width="350">

　
- LIBRARY MANAGER で **ESP32Servo** を検索してインストールする  
<img src="https://github.com/user-attachments/assets/cae88e9d-53ed-4a1c-bf38-bd7de3d8b462" width="350">  

　
- Select Other Board and Port で **ESP32 Dev Module** を検索して選択する 
<img src="https://github.com/user-attachments/assets/670f19c9-2996-4bbd-9af5-0fa0ecb96540" width="350">  


## UGOKU Pad の設定
UGOKU Padをインストール

[<img src="https://github.com/user-attachments/assets/73952bbe-7f89-46e9-9a6e-cdc7eea8e7c8" alt="Get it on Google Play" height="60">](https://play.google.com/store/apps/details?id=com.ugoku_lab.ugoku_console)　[<img src="https://github.com/user-attachments/assets/e27e5d09-63d0-4a2e-9e14-0bb05dabd487" alt="App Store" height="60">](https://apps.apple.com/jp/app/ugoku-pad/id6739496098)

参考: https://ugoku-lab.github.io/ugokupad.html

## ピン配置（現在のスケッチに準拠）
| 機能 | ピン | 備考 |
| --- | --- | --- |
| サーボ1 | 14 | 50Hz, attach(500–2500μs) |
| サーボ2 | 27 | 〃 |
| アナログ入力 (PSD) | 33 | ADC1_CH3 |
| LED1/2/3 | 2 / 4 / 13 | アクティブLOW（LOWで点灯） |
| 汎用出力 OUT1/OUT2 | 23 / 25 | アクティブLOW（LOWでON想定） |
| DIP（サーボ反転） | 34 | INPUT（入力専用ピン） |
| DIP（モーター反転） | 35 | INPUT（入力専用ピン） |
| I2C SDA / SCL | 21 / 22 | BMI270 用 |

## チャンネル対応（Console「ESP32 Arduino Sample」想定）
- ch1: LED 3個の一括 ON/OFF（1=ON, 0=OFF 相当。内部はアクティブLOW）
- ch2: サーボ1 角度 0..180（DIPで反転可能）
- ch3: サーボ2 角度 0..180（DIPで反転可能）
- ch4: モーター MD1（スティック中央=停止。0..255を-1..+1にスケーリング）
- ch5: モーター MD2（同上）
- ch6: OUT1/OUT2 の ON/OFF（1=ON 相当。内部はアクティブLOW）
- ch10: PSD 距離の簡易換算値（cm目安。センサ固有の較正は未実装）
- ch20: Roll（X周り）0..180（平置き=約90）
- ch21: Pitch（Y周り）0..180（平置き=約90）
- ch22: Yaw（Z周り）0..180（簡易。地磁気未使用のためドリフトあり）

## ライセンス
このリポジトリのライセンスは `LICENSE` を参照してください。



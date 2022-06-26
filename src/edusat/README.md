# サンプルプログラムの構成

```txt
.
|- edusat.ino  # CanSatの制御プログラム
|- obniz.html  # ObnizのHTMLプログラム
```

サンプルプログラムの動かし方は「[13. サンプルプログラムを動かす](../../doc/md/13.md)」を参照。

# サンプルプログラムの動作

サンプルプログラムはObnizへ各センサのデータを送信して、Obnizの実行画面でそれらの値を描画するプログラムとなっています。

# サンプルプログラムの説明

## edusat.ino

Arduino IDEで開きます。

### 使用ライブラリ

|ライブラリ名|URL|
|:---|:---|
|MPU9250.h|https://github.com/hideakitai/MPU9250|
|obniz.h|https://obniz.com/ja/doc/reference/obnizos-for-esp32/plugin/|
|TinyGPSPlus.h|https://github.com/mikalhart/TinyGPSPlus|

### setup()

最初に一度だけ`setup`関数が実行されます。
この関数では主に初期化処理を行います。

`setup`関数で実行される関数やメソッド

|関数名|概要|
|:---|:---|
|Serial.begin()|シリアル通信を開始します。引数に渡したボーレート（ビット/秒）に設定されます|
|obniz_init()|Obniz OSの初期化処理|
|pin_init()|GPIOの初期化処理|
|sd_init()|SDカードの初期化|
|mpu_init()|9軸センサの初期化|
|gps_init()|GPSセンサの初期化|
|startUpdateMPUValTask()|9軸センサの値を更新するスレッドを起動|
|startUpdateGPSValTask()|GPSセンサの値を更新するスレッドを起動|
|startSendObnizTask()|Obnizにデータ送信するスレッドを起動|

### loop()

`setup`関数の次に実行されます。
この関数は繰り返し実行されます。

`loop`関数では`switch-case`文で状態遷移するようなプログラムにしています。

|状態|概要|
|:---|:---|
|ST_STAND_BY|待機状態|
|ST_DRIVE|目標地点へ走行する状態|
|ST_GOAL|目標地点へ到達した状態|

## obniz.html

Obnizの開発者コンソールで開きます。

CanSatから送信したデータを受け取って、ブラウザの画面に表示するプログラムとなっています。

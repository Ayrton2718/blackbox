# BlackBox
BlackBoxライブラリは、航空機のフライトレコーダーに着想を得たもので、ロボットのTopic、ログ、エラー、パラメータ、変数などの状態を記録し、後で分析することができます（リアルタイムでもできます）。ログデータはmcap形式で保存され、[FoxGlove](https://foxglove.dev/)や[rosbag2](https://github.com/ros2/rosbag2)と組み合わせることで、強力な分析ツールとして活用できます。

# FoxGloveについて
FoxGloveは、FoxGlove Studio（GUIツール）とFoxGlove bridge（ROS2の通信をWebSocketに変換するツール）から構成されています。FoxGlove Studioはmcapファイルを直接解析でき、Windows、Mac、スマホ（ブラウザ）で動作します。ROS2が起動しているLinux上でFoxGlove Bridgeを起動し、FoxGlove StudioでそのIPとポート番号を指定することで接続できます。

[FoxGlove Studioのインストール](https://foxglove.dev/download)  
[FoxGlove Bridgeのインストール](https://docs.foxglove.dev/docs/connecting-to-data/ros-foxglove-bridge/)

FoxGlove Studioは日本語に対応しており、設定から言語変更が可能です。また、フレームレートの調整も設定画面から行えます。

# このライブラリの使用方法
このライブラリを使用するには、BlackBoxNodeを継承する必要があります。rclcpp::Nodeの継承は避けてください。
```
class TestNode : public blackbox::BlackBoxNode
```

## Diagnostic
各モジュールの状態を監視するためのもの。rvizやFoxgloveを用いることで、可視化することができる。smbusのセンサやアクチュエータには既にDiagnosticが組み込まれているため、smbusを使用する際は、センサ一つ一つにDiagnosticを適用する必要はありません。センサの初期化やオフセットの成功状態などに適用することができます。

[GitHub](https://github.com/ros/diagnostics/tree/ros2)  
[FoxGlove Diagnostics Panel](https://docs.foxglove.dev/docs/visualization/panels/diagnostics/)  
[ROS1での仕様例](https://qiita.com/srs/items/46c8593dad23497902a8)  

```
blackbox::Diagnostic  _diag1;
_diag1.init(this, "diag1");                 // diagnosticのモジュール名を指定する
_diag1.ok("OK");                            // 状態とメッセージを設定
_diag1.warn("sometime lost connection");
_diag1.error("lost connection");
```

## Logger
実行中中のデバッグメッセージをレコードすることができる。ログのごとにログの重要度ととタグ名を設定することができる。`FoxGlove Studio`のLogパネルを用いることで、ログの出力を確認することができます。ログは、`/tagger/namespace/node_name`に格納されています。

[FoxGlove Log Panel](https://docs.foxglove.dev/docs/visualization/panels/log) 

```
blackbox::Logger  _log_info;
_log_info.init(this, blackbox::INFO, "log1");   // ログのレベル、ログのタグ
TAGGER(_log1, "status : OK");
```

ノードの宣言時に`logger_mode`を`DEBUG`に設定することで、そのノードのすべてのLoggerのログがコンソール上に出力されるようになる。（blackbox::LogRecorder::mode_t::RELEASE）
```
this->logger_mode(blackbox::LogRecorder::mode_t::DEBUG);
```

## Record
パッケージ内の内部変数等をレコードするライブラリです。数値型を持つメッセージであれば、`FoxGlove Studio`のPlotパネルと連携させることで、グラフを表示することができ、実行中は`FoxGlove Bridge`経由で波形を監視し、実行後は生成されるmcapファイルから解析することができます。レコードしたデータは、`/record/namespace/node_name`に格納されています。

[FoxGlove Plot Panel](https://docs.foxglove.dev/docs/visualization/panels/plot)  
FoxGlove Plotを使う際は、Rangeを設定すると見やすくなります。

```
blackbox::Record<std_msgs::msg::Float32MultiArray, true, true>   _rec1;　// メッセージの型, Publishするか、レコードするか
_rec1.init(this, "rec1", 10, 0);  // トピック名、qos、何回に一回ドロップ（レコードしない）するか
std_msgs::msg::Float32MultiArray msg;
msg.data = {0.342, 0.234, 342.3};
_rec1.record(msg);
```

## Topic
Topicするデータをレコードすることができます。Topicは通常のPublisherをレコードすることが目的で、Recordはノードの内部変数を監視することが目的で作られています。。（Recordと機能的に違う部分としては、topic名をこちらが設定できる点のみです）

```
blackbox::Topic<std_msgs::msg::String, true>    _topic1;    // メッセージの型, レコードするか
_topic1.init(this, "topic", 10, 0); // トピック名、qos、何回に一回ドロップ（レコードしない）するか

std_msgs::msg::String;
str.data = "lol";
_topic1.publish(str);
```


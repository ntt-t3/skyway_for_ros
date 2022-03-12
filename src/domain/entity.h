// このROSパッケージで利用するEntityを定義する
// 基本的に全てRust側で実施するので、ここではROS Topic関係だけ定義する

#ifndef SKYWAY_ENTITY_H
#define SKYWAY_ENTITY_H

#include <fruit/fruit.h>

#include <boost/asio.hpp>
#include <unordered_map>

using boost::asio::ip::udp;

// ========== Source ==========

// エンドユーザプログラムからデータを受信し、
// WebRTC GWに流し込むオブジェクトの定義
// 実装はInfra層で行う
class Source {
 private:
 public:
  virtual ~Source() = default;
  virtual void Start() {}
  virtual void Stop() {}
  // Topic名を取得
  virtual std::string TopicName() { return ""; }
};

using SourceFactory = std::function<std::unique_ptr<Source>(
    std::string, udp::endpoint, udp::endpoint)>;

fruit::Component<SourceFactory> getSourceComponent();

// ========== Destination ==========

// DataChannelから得られたデータをエンドユーザに返すオブジェクトの定義
// 実装はInfra層で行う
class Destination {
 public:
  virtual ~Destination() = default;
  // WebRTC GWからのデータ受信とPublishを開始する
  // 重複コールは許容するが、stop後の再開はサポートしない(未定義動作)
  virtual void Start() {}
  // WebRTC GWからのデータ受信とPublishを停止する
  // 重複コールは許容する
  virtual void Stop() {}
  // 内部で保持しているソケットのポート番号を取得する
  virtual unsigned short Port() { return 0; }
  // Topic名を取得
  virtual std::string TopicName() { return ""; }
};

using DestinationFactory =
    std::function<std::unique_ptr<Destination>(std::string)>;

fruit::Component<DestinationFactory> getDestinationComponent();

#endif  // SKYWAY_ENTITY_H

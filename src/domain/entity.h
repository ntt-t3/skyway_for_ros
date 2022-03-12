// このROSパッケージで利用するEntityを定義する
// 基本的に全てRust側で実施するので、ここではROS Topic関係だけ定義する

#ifndef SKYWAY_ENTITY_H
#define SKYWAY_ENTITY_H

#include <fruit/fruit.h>

#include <boost/asio.hpp>
#include <unordered_map>

using boost::asio::ip::udp;

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

#endif  // SKYWAY_ENTITY_H

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

using SourceFactory =
    std::function<std::unique_ptr<Source>(std::string, udp::endpoint)>;

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
    std::function<std::unique_ptr<Destination>(std::string, udp::endpoint)>;

fruit::Component<DestinationFactory> getDestinationComponent();

// ========== DataTopicContainer ==========

// SourceとDestinationをDataConnectionIdと紐づけて管理するオブジェクト
class DataTopicContainer {
 private:
  // DataConnectionIdとSourceの紐づけ
  std::unordered_map<std::string, std::unique_ptr<Source>> source_map_{};
  // DataConnectionIdとDestinationの紐づけ
  std::unordered_map<std::string, std::unique_ptr<Destination>>
      destination_map_{};

 public:
  virtual ~DataTopicContainer() = default;
  // DataConnectionId, Source, Destinationを登録
  // 登録成功時にSourceとDestinationをstartする
  virtual bool CreateData(std::string data_connection_id,
                          std::unique_ptr<Source> source,
                          std::unique_ptr<Destination> destination) {
    return false;
  }
  // DataConnectionIdに紐付いているSource, Destinationを破棄
  virtual bool DeleteData(std::string data_connection_id) { return false; }
  // DataConnectionIdに紐付いているDestinationのportを取得
  virtual unsigned short DestinationPort(std::string data_connection_id) {
    return 0;
  }
  // DataConnectionIdに紐付いているSourceのTopic名を取得
  virtual std::string SourceTopicName(std::string data_connection_id) {
    return "";
  }
  // DataConnectionIdに紐付いているDestinationのTopic名を取得
  virtual std::string DestinationTopicName(std::string data_connection_id) {
    return "";
  }
};

class DataTopicContainerImpl : public DataTopicContainer {
 private:
  // DataConnectionIdとSourceの紐づけ
  std::unordered_map<std::string, std::unique_ptr<Source>> source_map_{};
  // DataConnectionIdとDestinationの紐づけ
  std::unordered_map<std::string, std::unique_ptr<Destination>>
      destination_map_{};

 public:
  INJECT(DataTopicContainerImpl()) {}
  // DataConnectionId, Source, Destinationを登録
  // 登録成功時にSourceとDestinationをstartする
  virtual bool CreateData(std::string data_connection_id,
                          std::unique_ptr<Source> source,
                          std::unique_ptr<Destination> destination) override;
  // DataConnectionIdに紐付いているSource, Destinationを破棄
  virtual bool DeleteData(std::string data_connection_id) override;
  // DataConnectionIdに紐付いているDestinationのportを取得
  virtual unsigned short DestinationPort(
      std::string data_connection_id) override {
    return destination_map_.at(data_connection_id)->Port();
  }
  // DataConnectionIdに紐付いているSourceのTopic名を取得
  virtual std::string SourceTopicName(std::string data_connection_id) override {
    return source_map_.at(data_connection_id)->TopicName();
  }
  // DataConnectionIdに紐付いているDestinationのTopic名を取得
  virtual std::string DestinationTopicName(
      std::string data_connection_id) override {
    return destination_map_.at(data_connection_id)->TopicName();
  }
};

fruit::Component<DataTopicContainer> getDataTopicContainerComponent();

#endif  // SKYWAY_ENTITY_H

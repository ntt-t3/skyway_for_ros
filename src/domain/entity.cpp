//
// Created by nakakura on 2022/03/12.
//

#include "entity.h"

bool DataTopicContainerImpl::CreateData(
    std::string data_connection_id, std::unique_ptr<Source> source,
    std::unique_ptr<Destination> destination) {
  // data_connection_idが既に存在する場合は登録は行わない
  if (source_map_.find(data_connection_id) != source_map_.end() ||
      destination_map_.find(data_connection_id) != destination_map_.end()) {
    return false;
  }

  source_map_.emplace(data_connection_id, std::move(source));
  source_map_.at(data_connection_id)->Start();
  destination_map_.emplace(data_connection_id, std::move(destination));
  destination_map_.at(data_connection_id)->Start();

  return true;
}

bool DataTopicContainerImpl::DeleteData(std::string data_connection_id) {
  if (source_map_.find(data_connection_id) == source_map_.end() &&
      destination_map_.find(data_connection_id) == destination_map_.end()) {
    return false;
  }

  source_map_.at(data_connection_id)->Stop();
  source_map_.erase(data_connection_id);
  destination_map_.at(data_connection_id)->Stop();
  destination_map_.erase(data_connection_id);
  return true;
}

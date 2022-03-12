#include "../../src/domain/entity.h"

#include <fruit/fruit.h>
#include <gtest/gtest.h>

#include <iostream>
#include <string>

#include "../stub/destination_stub.h"
#include "../stub/source_stub.h"

TEST(TestSuite, data_topic_container_create) {
  int source_flag = 0;
  auto f = [&](int x) { source_flag += x; };
  Injector<SourceStubFactory> source_injector(getSourceStubComponent);
  SourceStubFactory sourceStubFactory(source_injector);
  auto source = sourceStubFactory(f);

  int destination_flag = 0;
  auto df = [&](int x) { destination_flag += x; };
  Injector<DestinationStubFactory> destination_injector(
      getDestinationStubComponent);
  DestinationStubFactory destinationStubFactory(destination_injector);
  auto destination = destinationStubFactory(df);

  DataTopicContainerImpl container;
  bool flag =
      container.CreateData("hoge", std::move(source), std::move(destination));

  // 登録に成功
  ASSERT_TRUE(flag);
  // Sourceがstartされている。破棄はされていない
  ASSERT_EQ(source_flag, 1);
  // Destinationがstartされている。破棄はされていない
  ASSERT_EQ(destination_flag, 1);

  int source_flag_2 = 0;
  auto f_2 = [&](int x) { source_flag_2 += x; };
  Injector<SourceStubFactory> source_injector_2(getSourceStubComponent);
  SourceStubFactory sourceStubFactory_2(source_injector_2);
  auto source_2 = sourceStubFactory_2(f_2);

  int destination_flag_2 = 0;
  auto df_2 = [&](int x) { destination_flag_2 += x; };
  Injector<DestinationStubFactory> destination_injector_2(
      getDestinationStubComponent);
  DestinationStubFactory destinationStubFactory_2(destination_injector_2);
  auto destination_2 = destinationStubFactory_2(df_2);

  bool flag_2 = container.CreateData("hoge", std::move(source_2),
                                     std::move(destination_2));

  // 登録に失敗
  ASSERT_FALSE(flag_2);
  // Sourceがstartされずに破棄されている
  ASSERT_EQ(source_flag_2, 2);
  // Destinationがstartされずに破棄されている
  ASSERT_EQ(destination_flag_2, 2);
}

TEST(TestSuite, data_topic_container_create_and_delete) {
  int source_flag = 0;
  int destination_flag = 0;
  {
    auto f = [&](int x) { source_flag += x; };
    Injector<SourceStubFactory> source_injector(getSourceStubComponent);
    SourceStubFactory sourceStubFactory(source_injector);
    auto source = sourceStubFactory(f);

    auto df = [&](int x) { destination_flag += x; };
    Injector<DestinationStubFactory> destination_injector(
        getDestinationStubComponent);
    DestinationStubFactory destinationStubFactory(destination_injector);
    auto destination = destinationStubFactory(df);

    DataTopicContainerImpl container;
    bool flag =
        container.CreateData("hoge", std::move(source), std::move(destination));
  }

  // sourceがstartされている
  // container object破棄時にsourceも破棄されている
  ASSERT_EQ(source_flag, 3);
  // destinationがstartされている
  // ontainer object破棄時にdestinationも破棄されている
  ASSERT_EQ(destination_flag, 3);
}

// DeleteDataに成功するケース
TEST(TestSuite, data_topic_delete_data) {
  int source_flag = 0;
  int destination_flag = 0;

  auto f = [&](int x) { source_flag += x; };
  Injector<SourceStubFactory> source_injector(getSourceStubComponent);
  SourceStubFactory sourceStubFactory(source_injector);
  auto source = sourceStubFactory(f);

  auto df = [&](int x) { destination_flag += x; };
  Injector<DestinationStubFactory> destination_injector(
      getDestinationStubComponent);
  DestinationStubFactory destinationStubFactory(destination_injector);
  auto destination = destinationStubFactory(df);

  DataTopicContainerImpl container;
  bool flag =
      container.CreateData("hoge", std::move(source), std::move(destination));
  container.DeleteData("hoge");

  // sourceがstartされている
  // DeleteData時にsourceが破棄されている
  ASSERT_EQ(source_flag, 3);
  // destinationがstartされている
  // DeleteData時にDestinationが破棄されている
  ASSERT_EQ(destination_flag, 3);
}

// DeleteDataに失敗するケース
TEST(TestSuite, data_topic_delete_data_failed) {
  DataTopicContainerImpl container;
  bool flag = container.DeleteData("hoge");

  // DeleteDataに失敗
  ASSERT_FALSE(flag);
}

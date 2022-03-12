#ifndef SKYWAY_SOURCE_IMPL_TEST_H
#define SKYWAY_SOURCE_IMPL_TEST_H

void publish_data(std::string topic_name, int data_len);
void udp_recv(unsigned short port, int recv_data_len, bool &is_received,
              std::vector<std::vector<char>> &recv_data_array);

#endif  // SKYWAY_SOURCE_IMPL_TEST_H

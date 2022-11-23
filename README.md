# SkyWay for ROS

ROSからSkyWayを利用するためのパッケージです。

端末間でP2P接続を行うための規格としてWebRTCが標準化されています。
SkyWayはWebRTCプラットフォームであり、SkyWayを利用することで、ブラウザ・iOS・Androidなどの環境で相互にWebRTC接続を簡単に行うことができます。
WebRTCは映像と音声を双方向通信するためのMediaStreamと、データを転送するためのDataChannelが定義されています。
Skyway for ROSを利用することで、MediaStreamとDataChannelをROSパッケージから利用することができます。

SkyWay for ROSでは2つのサービスが提供されており、これらを利用し操作を行います。
 
- [SkyWayControl](./srv/SkyWayControl.srv)
- [SkyWayEvents](./srv/SkyWayEvents.srv)


各サービスへのアクセス方法は以下の通りです。

- [PeerObjectの生成](./doc/peer_create.md)
- [PeerObjectの状態確認](./doc/peer_create.md)

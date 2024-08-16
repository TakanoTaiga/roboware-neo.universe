# mcu_ros_driver

Motor Control Unitを制御するモジュール。論理層を担当。ros_udp_python_driverと同時に使う.

## Package URL

repo url: [control/mcu_ros_driver](https://github.com/hakoroboken/roboware-neo.universe/tree/main/control/mcu_ros_driver)

## Launch

```bash
ros2 launch roboware_neo_launch mcu.launch.xml
```

## Topics

| Name               | Type                                   | Description                                       |
|--------------------|----------------------------------------|---------------------------------------------------|
| output/udp_packet  | rw_common_msgs::msg::UDPPacketBinary   | UDPパケットのバイナリデータを配信するパブリッシャー  |
| output/device_list | rw_common_msgs::msg::MCUDeviceLists    | MCUデバイスリストを配信するパブリッシャー            |
| input/udp_packet   | rw_common_msgs::msg::UDPPacketBinary   | UDPパケットのバイナリデータを受信するサブスクライバー |
| input/mcu_order    | rw_common_msgs::msg::MCUDeviceLists    | MCUデバイスの指令データを受信するサブスクライバー     |
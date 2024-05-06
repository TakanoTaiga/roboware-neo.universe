# joy_to_twist

ゲームコントローラの入力を受け取る`joy_node`からパプブリッシュされるデータをTwist型に変換するノード。

## Package URL

repo url: [control/joy_to_twist](https://github.com/hakoroboken/roboware-neo.universe/tree/main/control/joy_to_twist)

## Launch

`joy.launch.xml`を実行すると`joy_node`と`joy_to_twist`が起動します。コントローラを接続すると`Twist`がパブリッシュされます。

```bash
ros2 launch roboware_neo_launch joy.launch.xml
```

## Topics

| Name | Type | Description |
|--|--|--|
| output/twist | geometry_msgs::msg::Twist | 変換されたTwistデータ |
| input/joy | sensor_msgs::msg::Joy | ゲームコントローラのデータを受け取る |

## Parameters

ありません
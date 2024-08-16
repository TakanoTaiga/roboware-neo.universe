# wp2wp_planner
mission managerの命令を受取ポイントtoポイントのパスプランニングをおこなう。

## Launch
```bash
ros2 launch roboware_neo_launch global_path_planning.launch.xml
```
## Topics

| Name                 | Type                                   | Description                                    |
|----------------------|----------------------------------------|------------------------------------------------|
| input/pose/current    | geometry_msgs::msg::PoseStamped        | 現在のPoseデータを受信するサブスクライバー         |
| input/pose/goal       | geometry_msgs::msg::PoseStamped        | 目標のPoseデータを受信するサブスクライバー         |
| input/task_action     | rw_planning_msg::msg::TaskAction       | タスクアクションを受信するサブスクライバー         |
| output/global_plan_path | nav_msgs::msg::Path                 | グローバルプランされた経路を配信するパブリッシャー  |
| debug/area            | visualization_msgs::msg::Marker        | デバッグ用のエリアマーカーを配信するパブリッシャー   |
| debug/robot           | visualization_msgs::msg::Marker        | デバッグ用のロボットマーカーを配信するパブリッシャー |

## Parameters

| Name          | Type   | Default Value | Description                          |
|---------------|--------|---------------|--------------------------------------|
| map.plypath   | string | "kill"        | マップのPLYファイルのパス               |
| robot.plypath | string | "kill"        | ロボットのPLYファイルのパス             |
| max_speed     | double | 1.0           | ロボットの最大速度                      |


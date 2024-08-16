# path_follower

PurePursuit的なので経路追従を行う。

### Launch

```bash
ros2 launch roboware_neo_launch path_follower.launch.xml
```

### Topics

| Name                | Type                                  | Description                                           |
|---------------------|---------------------------------------|-------------------------------------------------------|
| input/current_pose  | geometry_msgs::msg::PoseStamped       | 現在のロボットのPoseを受信するサブスクライバー           |
| input/nav_path      | nav_msgs::msg::Path                   | ナビゲーションパスを受信するサブスクライバー             |
| input/task_action   | rw_planning_msg::msg::TaskAction      | タスクアクションを受信するサブスクライバー               |
| output/action_result| rw_planning_msg::msg::ActionResult    | アクション結果を配信するパブリッシャー                  |
| output/cmd_pose     | geometry_msgs::msg::Pose              | 指令されたPoseを配信するパブリッシャー                   |
| debug/target_pose   | geometry_msgs::msg::PoseStamped       | ターゲットPoseのデバッグ情報を配信するパブリッシャー     |

### Parameters

| Name               | Type   | Default Value | Description                              |
|--------------------|--------|---------------|------------------------------------------|
| position_tolerance | double | 0.01          | 位置の許容誤差                            |
| angle_tolerance    | double | 1.0           | 角度の許容誤差                            |
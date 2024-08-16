# mission_manager

Mermid interpreter。ロボットのmissionプランニングを行う。

## Launch

```bash
ros2 launch roboware_neo_launch mission_manager.launch.xml
```

## Topics

| Name               | Type                                        | Description                                         |
|--------------------|---------------------------------------------|-----------------------------------------------------|
| output/task_action | rw_planning_msg::msg::TaskAction             | タスクアクションのデータを配信するパブリッシャー           |
| input/action_result| rw_planning_msg::msg::ActionResult           | アクション結果を受信するサブスクライバー                  |

## Parameters

| Name      | Type    | Default Value | Description                               |
|-----------|---------|---------------|-------------------------------------------|
| graphpath | string  | "./graph.md"  | グラフの初期化に使用するファイルのパス             |
| endmode   | string  | "kill"        | エンドモードの指定（"kill" または "safe"）        |


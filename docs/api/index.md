# API Document

## Mission Planner Strategy API

### 既定クラス

下記のクラスを継承してオーバーロードすることでStrategyを定義することができます。
```c++
class RWStrategy{
public:
    virtual ~RWStrategy(){}
    virtual void update(node_bin& node, debug_info& info) = 0;

protected:
    rclcpp::Publisher<rw_planning_msg::msg::TaskAction>::SharedPtr pub_task_action_;
    rw_planning_msg::msg::ActionResult action_result;
    std::string strategy_label;
};
```

作成例：
```c++
class StartStrategy : public RWStrategy
{
public:
    StartStrategy();
    void update(node_bin& node, debug_info& info) override;
};
```

**イニシャライザ**

イニシャライザは下記を必ず記載してください。

```c++
// 命令句をここで指定します。
strategy_label = "HOGE";
```

`SETPOSE.1(SETPOSE:x=-2.0,y=2.0,z=270.0)`このような構文に対応させたい場合は下記のように記載します。

```c++
strategy_label = "SETPOSE";
```

**Update**

update
関数では下記のAPIにアクセスできます。

- rclcpp::Publisher<rw_planning_msg::msg::TaskAction>::SharedPtr pub_task_action_;
    - 外部にTaskActionメッセージをパブリッシュできます。
- action_result
    - アクション結果を取得することができます。この変数はタスクIDに一致するように自動的に与えられておりプライベートに使用できます。

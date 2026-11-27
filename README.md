# 大作业：CyberDog 2v2 足球比赛系统

面向第五届机器狗开发大赛复赛规则的 2v2 足球控制程序，基于光学动捕（VRPN）与 ROS2 伺服运动控制实现，采用普通多文件 Python 工程结构，无需 `colcon build`。

## 目录结构

```
CYBERDOG_FINAL/
├── start.sh                # 比赛启动脚本（规则 6.1/6.2：左右双终端启动 1/2 号狗）
├── run_dog.py              # 单狗入口，处理命令行参数与 Ctrl+C 结束帧
├── soccer/
│   ├── __init__.py
│   ├── config.py           # 场地、刚体名、名字空间与控制参数（现场只改这里）
│   ├── geometry.py         # 二维向量、角度归一化、四元数转偏航角
│   ├── field.py            # 10m×5.5m 场地、球门、0.8m 死区、9 个发球点、坐标换算
│   ├── model.py            # Pose / World / Command 数据结构
│   ├── roles.py            # 两狗动态分工（1 号发布、2 号跟随，断线自动降级）
│   ├── game.py             # 进球确认、15s 暂停、被进球方开球等待
│   ├── motion.py           # 人工势场避让、防冲撞减速、机身速度转换、卡住脱困
│   ├── brain.py            # 单狗决策（前锋绕球推射、后卫盯防/门前解围、死区预站位）
│   └── ros_node.py         # ROS2 节点（VRPN 订阅、自动起身、运动发布）
├── tools/
│   └── simulate.py         # 无 ROS 的 2v2 完整比赛仿真（上下半场各 4 分钟）
├── test/
│   └── test_soccer.py      # 纯逻辑单元测试
├── 大作业报告.pdf          # 大作业实践报告（策略、代码、收获与建议）
└── README.md
```

## 复赛规则对应设计

| 复赛规则 | 程序实现 |
| --- | --- |
| 场地 10 m × 5.5 m，球门宽 1.5 m，死球线距边线 0.8 m | `soccer/field.py` 严格按规则图 1 建模，含 4 个出发点与 9 个发球点 |
| 起始趴在出发点，裁判启动后起身（规则 3.2/3.3） | `ros_node.py` 启动时调用 `/<ns>/motion_result_cmd`（`motion_id=111`）起立后再进入比赛 |
| 进球暂停 15 s、被进球方开球（规则 3.4） | `game.py` 连续 3 帧确认球完全入网后发结束帧暂停 15 s；我方进球后等待对方踢动球（最多 5 s） |
| 禁止故意冲撞对方机器狗（规则 4.1） | `motion.py` 人工势场在接触半径内将朝向对方狗的速度清零，且正前方 0.6 m 有狗时限速 0.08 m/s |
| 死区/出界就近发球（规则 4.2/4.4） | 球入死区时不贴墙硬抢，前锋提前站到最近发球点后方，后卫守门 |
| 卡住边界/顶牛 8 s 判死球（规则 4.3/4.4） | `StuckDetector` 检测到有速度指令但 2.5 s 位移小于 0.1 m 时，主动后退转身脱困 1.2 s |
| 启动命令 `./start.sh`，左右双终端 1/2 号狗（规则 6.1/6.2） | `./start.sh` 自动开左右终端；`./start.sh 1` 或 `./start.sh 2` 支持单狗被抱起后独立重启 |

## 现场部署与运行

### 1. 修改配置

编辑 `soccer/config.py`：
- `DOGS[1]`、`DOGS[2]`：填入 FZMotion 刚体名、机器狗 ROS2 名字空间、SSH 地址（如使用远程模式）。
- `BALL_RIGID_BODY`：球的刚体名（默认 `ball`）。
- `FIELD.origin_x / origin_y / rotation`：若动捕世界坐标系原点不在场地左下角，填入换算参数。

### 2. 启动动捕 VRPN

按第四讲课件启动 `vrpn_listener`，确认 `/vrpn/<rigid_body>/pose` 有数据。

### 3. 预览与实机运行

实机启动时，只有起身服务返回 `result=True` 且 `code=0` 才进入比赛。起身失败或超时后，该狗会停留在等待重启状态。查看终端错误并排查原因，再按 Ctrl+C 退出并单独重启该狗。

```bash
# 预览（不发运动指令，不起身，检查坐标和方向）
./start.sh 1 --dry-run
./start.sh 2 --dry-run

# 比赛正式启动（规则 6.1/6.2：自动打开左右两个终端）
./start.sh

# 单狗独立启动（例如 1 号狗被抱起放回出发点后，裁判重新启动）
./start.sh 1 --side left --kickoff us
```

通过环境变量可切换本机/SSH 启动方式：
```bash
RUN_ON=ssh DOG1_HOST=mi@192.168.1.101 DOG2_HOST=mi@192.168.1.102 ./start.sh
```

## 无 ROS 测试与仿真

```bash
# 运行单元测试
python3 -m unittest discover -s test -t . -v

# 运行完整 2v2 仿真（上下半场各 4 分钟）
python3 tools/simulate.py --opponent chaser --seed 1
python3 tools/simulate.py --opponent same --seed 1
python3 tools/simulate.py --opponent static --seed 1
```

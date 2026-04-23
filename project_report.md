# Auto-Multilift_simulation 项目整体报告（代码现状版）

## 1. 项目定位与目标

`Auto-Multilift_simulation` 是一个基于 ROS2 + PX4 + Isaac Sim 的多无人机协同吊运仿真项目。  
核心目标是实现多机分布式 MPC（含并行求解能力）用于载荷协同运输，并支持从起飞、切换 MPC 到任务结束的完整流程。

当前分支处于“功能快速迭代中”状态，已经引入：

- 预计算轨迹管理（`PrecomputedTrajectoryManager`）
- YAML 参数管理（`ParameterManager`）
- 共享内存 + 多进程并行 QMPC（`parallel_mpc`）
- Isaac Sim 端载荷状态高频发布与初始化话题

---

## 2. 代码结构与模块职责

### 2.1 核心目录

- `src/px4-offboard/px4_offboard/`
  - 控制主逻辑：`multilift_sync_node.py`（中心/同步节点）、`multilift_quad_node.py`（单机节点）
  - 模型与求解：`Dynamics.py`、`Robust_Flight_MPC_acados.py`
  - 新增能力：`PrecomputedTrajectoryManager.py`、`config/`、`parallel_mpc/`
- `src/sitl_sim/sitl_sim/`
  - 仿真脚本：`sitl_stable.py`（生成 rope/payload/drone，并发布 payload pose/twist）
- `src/mpc_msgs/`
  - 分布式 MPC 通信消息/服务定义

### 2.2 运行链路（按你当前 `takeoff.md`）

1. `MicroXRCEAgent` 启动 DDS bridge。  
2. Isaac Sim 脚本 `sitl_stable.py` 启动仿真与 PX4 实例，并发布 `/payload_pose`、`/payload_twist`。  
3. ROS2 启动 `ros2 launch px4_offboard multilift_mpc.launch.py`，拉起可视化、SyncNode、多个 QuadNode。  

参考文件：
- `takeoff.md`
- `src/sitl_sim/sitl_sim/sitl_stable.py`
- `src/px4-offboard/launch/multilift_mpc.launch.py`

---

## 2.3 三终端在系统中的职责

### Terminal 0: `MicroXRCEAgent udp4 -p 8888`

职责是桥接 PX4 uORB 与 ROS2 DDS 通信。  
如果该进程未正常启动，`/fmu/out/*` 与 `/fmu/in/*` 相关链路会失效，QuadNode/SyncNode 无法拿到飞控状态或下发控制指令。

### Terminal 1: `ISAACSIM_PYTHON src/sitl_sim/sitl_sim/sitl_stable.py`

职责是启动 Isaac Sim 场景、生成 payload+rope+multi-quad，并发布仿真端状态话题：

- `/payload_pose` (`PoseStamped`)
- `/payload_twist` (`TwistStamped`)
- `/drone_i_init_pos`（初始化位姿，供可视化/初始化使用）

同时每个无人机实例通过 Pegasus/PX4 backend 启动 PX4 SITL。

### Terminal 2: `ros2 launch px4_offboard multilift_mpc.launch.py`

职责是启动控制系统：

- `multilift_sync_node`（中心同步与 LMPC）
- `multilift_quad_node` x `nq`（每架机体节点，几何起飞 + QMPC执行）
- `visualizer`（可视化）

---

## 2.4 端到端工作方式（状态机视角）

### 阶段 A: 上电与时钟同步

`multilift_sync_node` 周期发布 `/sync_time`。  
每个 `multilift_quad_node` 订阅 `/sync_time` 更新 `timestamp_us`，并持续发送 `OffboardControlMode`。

### 阶段 B: ARM 与起飞（几何控制）

QuadNode 状态从 `ST_INIT -> ST_ARMING -> ST_TAKEOFF`：  
当飞控进入 OFFBOARD 且 ARMED 后，执行 `generate_takeoff_trajectory + geom_publish_command`，将机体抬升到目标高度并收敛到参考初始点附近。

### 阶段 C: 进入分布式 MPC

当 SyncNode 检测 payload 达到高度条件后：

1. 启动 `mpc_forward_main`（更新 LMPC/QMPC迭代变量）
2. 启动 `broadcast_distributed_mpc`（发布 `/broadcast_mpc`）
3. `MPC_TRAJ=True`，系统转入分布式 MPC 闭环

### 阶段 D: 迭代求解与控制执行

QuadNode 订阅 `/broadcast_mpc`，在 `REC_TEMP_i=False` 时求解本机 QMPC，上传临时轨迹到中心（topic 模式）或由中心共享内存并行求解（parallel 模式）。  
中心节点聚合全部临时轨迹后，更新全局 `xq/uq/xl/ul`，再广播下一轮。  
QuadNode 按最新 `xq/uq` 提取 `xi_ctrl/ui_ctrl`，发布位置或姿态推力指令到 PX4。

### 阶段 E: 任务结束

`time_traj >= T_end` 后 SyncNode 设置 `MPC_TRAJ=False`，QuadNode 转 `ST_DONE`，停止 QMPC 前向求解定时器，保持末端控制或等待退出。

---

## 2.5 数据流向（消息与控制流）

### 仿真状态输入流

1. IsaacSim -> ROS2
- `/payload_pose`, `/payload_twist`

2. PX4 -> ROS2
- `/<px4_i>/fmu/out/vehicle_odometry`
- `/<px4_i>/fmu/out/vehicle_status_v1`
- `/<px4_i>/fmu/out/vehicle_local_position_v1`

### 中心广播流

SyncNode 发布：

- `/sync_time` (`Int64`)
- `/broadcast_mpc` (`BroadcastMPC`)

其中 `BroadcastMPC` 关键字段：
- `mpc_traj`: 任务启停标志
- `time_traj`: 当前控制时间
- `rec_temp`: 每机是否已返回临时解
- `xq_traj/uq_traj/xl_traj/ul_traj`: 分布式 MPC 当前迭代轨迹

### 局部回传流（非并行模式）

QuadNode 发布：

- `/quad_i/QMPC_temp` (`QuadReturnMPC`)

字段含义：
- `idx`: 机体编号
- `xi_temp/ui_temp`: 本机临时状态/控制轨迹
- `max_viol_i`: 本机迭代违反度

### 控制下行流（ROS2 -> PX4）

QuadNode 发布到 PX4 输入 topic：

- `offboard_control_mode`
- `vehicle_command`（切模式、arm）
- `vehicle_attitude_setpoint_v1`（几何控制/姿态推力）
- `trajectory_setpoint`（MPC位置控制路径）

---

## 2.6 并行与非并行两条 QMPC 路径

### 非并行（topic 回传）

每个 QuadNode 独立求解 QMPC -> 发 `QMPC_temp` -> SyncNode 聚合。  
优点是路径直观，便于调试；缺点是 ROS 消息序列化/调度开销更高。

### 并行（共享内存 + worker）

SyncNode 聚合各机状态并调用 `ParallelMPCCoordinator`，由 worker 进程并行求解后直接回写共享内存。  
优点是减少消息传输开销；风险点是超时、worker 异常、fallback策略不足。

---

## 2.7 坐标系与数据变换主线

- PX4 odometry 默认 NED/FRD。  
- 控制内部大量计算使用 ENU/world 表达。  
- `vehicle_odometry_callback` 与 `parallel_vehicle_odometry_callback` 中执行 NED->ENU、局部->world 平移与四元数变换。  

这是当前系统最关键的数据一致性环节之一，任何符号或轴顺序错误都会直接体现为轨迹偏移/姿态异常。

---

## 3. 当前代码状态评估

## 3.1 总体结论

项目已经具备完整链路雏形，且新并行架构已接入主流程；但配置一致性、参数来源统一性、并行鲁棒性与工程化规范仍未收敛。  
就“可研究、可演示”而言可继续推进；就“稳定复现实验”而言还需一轮系统性收口。

## 3.2 代码质量与可运行性

- 语法层面：对 `px4_offboard` 与 `sitl_sim` 的 Python 文件做过 `compileall`，未发现语法错误。
- 工作区层面：当前 `git status` 显示存在较多已修改/未跟踪文件，分支处于“开发中间态”，不适合直接作为可复现实验基线。
- 启动流程层面：文档与代码能对齐到三终端启动，但依赖环境要求严格（ROS2 humble、PX4 版本、acados 版本、IsaacSim 环境变量）。

---

## 4. 主要不完善点（按风险优先级）

## 4.1 高优先级（建议优先处理）

1. 参数源多头管理，存在不一致风险  
- `launch/multilift_mpc.launch.py` 中硬编码了 `uav_para/load_para/cable_para`；  
- `config/multilift_params.yaml` 也维护了同类参数；  
- `sitl_stable.py` 读取 YAML，但 `launch` 仍传固定值。  
这会导致仿真、控制、并行求解器参数不一致，出现“能跑但结果不可信”。

2. 启动时刻与时间同步机制耦合度高  
- QuadNode 依赖 `init_timestamp` 参数 + `/sync_time` 广播；  
- 若启动先后或时钟行为异常，状态机会卡在 `init/arming` 相关分支。  
目前缺少“时间源健康检查”和更清晰的降级策略。

3. 并行 QMPC 超时/失败回退策略不完整  
- `multilift_sync_node.py` 中并行失败时仅 warning + return false；  
- 缺少对持续超时、部分 worker 异常、自动切回 ROS topic QMPC 的状态管理。  
实际长时运行时这类问题会积累。

## 4.2 中优先级（影响维护与复现）

1. 打包与资源管理仍不统一  
- `setup.py` 已纳入部分 `config/`、`trained data/`，但仍依赖目录名和手工 glob；  
- 大量训练数据和备份目录直接放在包内，体积和版本管理成本高。

2. 日志体系混杂  
- ROS 节点内既有 `get_logger()` 也有大量 `print()`（尤其新增预计算模块）；  
- 中英文日志混用，且含 emoji，不利于批量日志分析与 CI 输出。

3. 注释债务与历史代码残留  
- 存在大量 `TODO/FIXME/XXX/pass`、注释掉的大段旧逻辑；  
- 会显著增加后续维护成本和误用概率。

## 4.3 低优先级（建议持续改进）

1. 测试覆盖不足  
- 现有多为模板化 lint 测试，缺少对状态机转换、并行超时处理、参数加载一致性的单测/集成测试。  

2. 依赖与版本约束未标准化  
- README 给出版本建议，但缺少 lock 方案（如环境导出文件、可执行 bootstrap 脚本）。

---

## 5. 建议的改进路线图

## 阶段 A（先稳住运行一致性）

1. 统一参数入口为 `multilift_params.yaml`，`launch` 不再硬编码动力学参数。  
2. 给 SyncNode 增加并行失败计数与自动降级策略（并行 -> ROS QMPC）。  
3. 增加启动前自检（YAML 可读、模型文件存在、`ACADOS_SOURCE_DIR`、topic ready）。

## 阶段 B（提升工程质量）

1. 统一日志接口：核心节点使用 ROS logger，工具脚本支持 `--verbose`。  
2. 清理无效注释与历史残留分支；保留最小必要注释。  
3. 拆分训练数据与运行包（数据外置或 artifact 管理）。

## 阶段 C（可复现实验）

1. 补齐最小回归测试集：参数一致性、状态机关键跳转、并行 worker 超时。  
2. 增加“一键复现实验脚本”（环境检查 + build + launch + 数据落盘）。

---

## 6. 对当前分支的建议结论

当前分支适合继续开发验证，不建议直接作为“稳定发布版”。  
如果近期目标是“论文/实验可复现”，建议先完成阶段 A，再冻结一版参数与运行脚本。

---

## 7. 关键参考文件

- `takeoff.md`
- `README.md`
- `src/px4-offboard/launch/multilift_mpc.launch.py`
- `src/px4-offboard/px4_offboard/multilift_sync_node.py`
- `src/px4-offboard/px4_offboard/multilift_quad_node.py`
- `src/px4-offboard/px4_offboard/config/multilift_params.yaml`
- `src/px4-offboard/px4_offboard/PrecomputedTrajectoryManager.py`
- `src/px4-offboard/px4_offboard/parallel_mpc/coordinator.py`
- `src/sitl_sim/sitl_sim/sitl_stable.py`

---

## 8. 建议补充的运行证据（用于最终定稿）

为把本报告从“静态代码分析版”升级为“实测版”，建议补充三段运行输出：

1. Terminal 0（MicroXRCEAgent）启动后前 20 行日志  
2. Terminal 2 执行 launch 后，`SyncNode` 与任一 `QuadNode` 的前 60 行日志  
3. `ros2 topic hz /sync_time`、`ros2 topic hz /broadcast_mpc`、`ros2 topic hz /payload_pose` 各 10 秒结果

有这三段数据后，可以精确验证：
- 时钟同步是否稳定
- 广播周期是否符合 `dt_broadcast`
- payload 状态输入是否连续
- 系统当前实际运行的是并行路径还是 topic 回传路径
---
| 类别 | 测量对象 | 目标频率 | 实测频率 / 耗时 | 实际含义 |
|---|---:|---:|---:|---|
| 时间同步 | `/sync_time` | `100Hz` | `~99-100Hz` | SyncNode 发布统一时间戳，供各 QuadNode 同步控制时间 |
| MPC 广播 | `/broadcast_mpc` | `100Hz` | `~100Hz` | 中心节点向各无人机广播 MPC 状态、轨迹、标志位 |
| 位置控制输出 | `/fmu/in/trajectory_setpoint` | -- | -- | 之前位置设定点路径的实测频率，目前已切换到姿态-推力 |
| 姿态-推力输出 | `/px4_i/fmu/in/vehicle_attitude_setpoint_v1` | 当前 `50Hz` | `~49.98-50Hz` | 当前实际使用的 PX4 控制输入频率 |
| 载荷状态反馈 | `/payload_pose` | 代码目标 `50Hz` | `~14Hz` | IsaacSim 发布 payload 位姿的实际频率，当前是状态反馈短板 |
| 并行 QMPC 平均耗时 | `[perf] mean` | 需小于 `20ms` | `~5.5ms` | 平均求解很快，50Hz 下有明显余量 |
| 并行 QMPC p95 耗时 | `[perf] p95` | 需小于 `20ms` | `~11.7-12.3ms` | 95% 周期内能稳定满足 50Hz |
| 并行 QMPC p99 耗时 | `[perf] p99` | 需小于 `20ms` | `~20.4ms` | 偶发周期接近或略超过 50Hz 周期 |
| 并行 QMPC 最大耗时 | `[perf] max` | 需小于 `20ms` | 最高 `~35ms` | 存在尖峰，说明 50Hz 不是强实时稳定上限 |
| 求解侧估计上限 | `[perf] est_ceiling p95` | - | `~80-85Hz`（50Hz实验后段） | 从 p95 求解耗时倒推的乐观上限 |
| 求解侧估计上限 | `[perf] est_ceiling p99` | - | `~48-49Hz`（50Hz实验后段） | 从 p99 求解耗时倒推的保守上限 |

结论：当前已经实测实现 `100Hz` MPC 广播和 `50Hz` 姿态-推力控制输出。并行 QMPC 在 50Hz 下 p95 可满足周期，但 p99 和 max 存在超时风险，因此推荐稳定控制频率约 `40Hz`，50Hz 可作为已验证的实验可达频率。
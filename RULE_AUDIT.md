# 比赛规则审计

- 比赛控制代码仅新增/修改 `src/swarm_rescue/solutions/blue_nav.py`、`blue_policy.py`、`my_drone_eval.py`。
- 未修改 `simulation/`、`maps/`、`launcher.py`、计分、传感器、规则或提交检查器。
- 控制器未调用 `true_position/true_angle/true_velocity/true_angular_velocity`，未访问 `_playground`、`map_data` 或 `walls_*`。
- 每架无人机状态均为实例字段；协作只经 `communicator.received_messages` 和 `define_message_for_all`，无跨无人机 Python 全局共享状态。
- 炸弹坐标只存在离线 `bench_cases/` 测试输入，控制器未加载这些文件或硬编码炸弹坐标。
- 控制输出经 `safe_command` 限制到 `forward/lateral/rotation ∈ [-1,1]`，`grasper ∈ {0,1}`。
- 位置/朝向来自 measured GPS、compass、odometer；地图来自 LiDAR；炸弹/中心/无人机检测来自 semantic sensor。
- 处置中心静态先验仅含公开地图尺寸、出生点与中心位置；不含墙体或炸弹真值。
- `E:\envs\swarm_rescue\python.exe scripts\check_submission.py .`：`Submission check passed.`
- 本审计不等于性能通过；性能明确未达标，详见 `TEST_REPORT.md`。


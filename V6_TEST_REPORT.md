# 蓝方 V6 官方仿真报告：用户固定坐标，各图一轮

测试日期：2026-11-16（Asia/Singapore）。将用户 `BLUE_V6_solutions_only.zip` 的三份源码部署到 `src/swarm_rescue/solutions/`。测试并报告原版 V6，不对候选控制器调参或挑选重跑成绩。官方 `simulation/`、`map_data/`、launcher、传感器、规则和计分逻辑未修改。

## 测试输入与范围

本轮首次使用用户 V6 完整包 `offline/make_eval_cases.py::FIXED` 表标注的**十图各十枚原始固定坐标**，从其源码提取数值并生成 `bench_cases_user_v6/MapXX_user_fixed.json`，供官方 launcher 离线输入。坐标没有进入飞控 `solutions`；每图坐标原样保留，未移动靠墙目标。十份输入的 SHA256 记录在 `bench_cases_user_v6/manifest.json`。该 FIXED 表是 V6 包中声明的用户原始表；本次未取得独立于该包的早期原始坐标文件作交叉核对。

每图只跑一轮，10 架无人机、10 枚炸弹；Map06 为无通信，Map08 为无 GPS，Map09 同时无通信和无 GPS。使用 `E:\envs\swarm_rescue\python.exe`、当前仓库 `src`，通过 `bench_tools/run_benchmarks.py` 启动官方 `python -m swarm_rescue.launcher --competition --headless`。**本次固定组与 V3–V5 的 `generated_fixed_seed0` 不是同一炸弹布局，不能直接比较得分归因。**

V6 包的离线几何检查对 100 个原样坐标给出若干 `<12 px` 净空警告：Map01、02、03、04、05、07、08、09 均至少一枚靠近墙、边界或特殊实体；其中 Map02 最小净空约 5 px、Map04 约 6 px。该检查是包中辅助脚本的几何近似，不等于官方可达性证明，也不能作为偷偷调整坐标的依据。Map06、Map07t4 最小净空约 13 px。

## 结论

十图共处置 **53/100** 枚，平均每图 **5.3/10、36.56 分**；仅 **Map02 1/10 张地图**达到 10/10，**0/10** 张达到 92 分。平均健康 **15.52%**，累计 **62 架次坠毁**。十轮进程退出码均为 0，无外部 300 秒超时；所有未全清地图均跑满官方步数上限。结果未达到目标，尤其 Map01 为 9/10、Map07t4 与 Map08 为 0/10。

| 地图 | 处置 | 得分 | 步数/上限 | 健康 | 坠毁 | 官方墙钟秒 | 拾取/释放事件 | 停滞对接观测 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Map01 | 9/10 | 67.9 | 2000/2000 | 69.6% | 0 | 37 | 10/10 | 12 |
| Map02 | 10/10 | 86.3 | 2072/5000 | 48.0% | 0 | 31 | 10/9 | 1 |
| Map03 | 4/10 | 25.4 | 5000/5000 | 7.2% | 7 | 76 | 13/5 | 301 |
| Map04 | 8/10 | 49.9 | 5000/5000 | 9.6% | 5 | 70 | 9/9 | 0 |
| Map05 | 9/10 | 54.0 | 7200/7200 | 0.0% | 10 | 47 | 10/10 | 7 |
| Map06（无通信） | 7/10 | 42.4 | 5400/5400 | 2.0% | 9 | 67 | 9/7 | 0 |
| Map07 | 1/10 | 6.6 | 5400/5400 | 3.2% | 8 | 99 | 10/1 | 140 |
| Map07t4 | 0/10 | 1.1 | 5400/5400 | 5.6% | 7 | 78 | 7/0 | 107 |
| Map08（无 GPS） | 0/10 | 1.8 | 5400/5400 | 8.8% | 7 | 88 | 7/0 | 0 |
| Map09（无通信、无 GPS） | 5/10 | 30.2 | 5400/5400 | 1.2% | 9 | 74 | 7/5 | 1 |

官方处置、分数、步数、健康、坠毁取自 launcher 回合汇总。拾取、释放是 `BLUE_V6_EVENT` 控制器日志，**不等同于官方处置**；Map01 有 10 条释放但官方只处置 9 枚，Map02 则官方 10/10 但仅 9 条释放。停滞对接观测是 `offline/diagnose_v6.py` 对稀疏日志中 `speed<1.2` 且 `|gap|<80` 的快照计数，**不是连续停滞步数**。逐轮 JSON、配置、原始日志和事件诊断分别见 `bench_results/v6_user_fixed_one_round/results.json`、`config_MapXX.yml`、`MapXX_user_fixed_r1.log`、`diagnostics.json`。

## 失败分析与可复核证据

1. **Map03、Map07、Map07t4 的对接死锁**：这三图分别有 415、199、184 条 `BLUE_V6_DOCK` 记录，四阶段对接均被触发；低速近中心观测 301、140、107 次。Map03 拾取 13、释放 5，Map07 为 10/1，Map07t4 为 7/0。日志中的多架携弹机长时间近中心低速，说明不是简单“没有发现炸弹”；仍需碰撞和运动轨迹分析才能把主因精确分为中心接触几何、规划或拥堵。
2. **Map08 主要卡在返航或进入中心前**：7 次拾取、0 次释放，但仅 3 条 `BLUE_V6_DOCK`，远少于 Map03/07/07t4。故不能把 Map08 的 0/10 全部归因为中心边缘扫掠；无 GPS 定位、远程返航与局部路径均仍是候选原因。本轮没有真值定位误差，无法量化漂移。
3. **Map01 和 Map05 的“释放即成功”假设不成立**：均记录 10 次释放，官方处置分别仅 9/10。夹爪由抓取转空并不能单独证明炸弹和处置中心发生了有效碰撞；应以官方 `rescued nb` 为准。
4. **碰撞生命反馈在真实仿真中未启用**：官方 `DroneAbstract.drone_health` 是 `@property`，返回本机整数生命值；V6 `blue_policy.py` 约第 616–627 行却执行 `own_health = getattr(...); hp = own_health() if callable(own_health) else None`。对整数属性 `callable` 为假，故十轮 `BLUE_V6` 日志的 `hp=None`，`BLUE_V6_DAMAGE` 事件总计 0；即使官方汇总有 62 架次坠毁，候选版“掉血驱动后撤/限速”也没有获得输入。离线测试把 `drone_health` 替换成 lambda，掩盖了这个接口类型错误。**这是具体代码与日志共同证实的实现缺陷，不是规则违规。** 本轮按用户要求测试原始 V6，未修改此代码。
5. **Pymunk 回调异常**：Map04、Map06、Map07t4 日志分别有 1 段 `Traceback`，是官方碰撞回调的 `get_entity_from_shape` 断言；进程仍以代码 0 完成。JSON 的 `crashed=true` 由日志出现 Traceback 触发，不代表进程崩溃，也不能凭共现证明其导致低分。未更改官方仿真。

## 合规与提交检查

- V6 三份源码 `py_compile` 通过；官方工作区 `scripts/check_submission.py .` 通过。
- 官方相关 pytest：距离传感器、处置计数、中心 guard，**23 passed**。
- V6 包的离线 `test_v3.py`–`test_v6.py` 链和 `test_diagnostics.py` 通过；它们是算法和模拟接口检查，不构成官方得分证明，且健康属性 stub 与真实接口不符。
- 静态审查未见预置炸弹坐标、完整墙/箱几何、测试文件读取、`true_*` 或仿真内部对象访问；飞控建图来自传感器，运行状态在无人机实例内，消息经官方 communicator。`CENTER_PRIORS` 保留公开起点/处置中心先验，是否完全符合最终组织方规则需由主办方确认。用户固定坐标仅保存在离线 `bench_cases_user_v6/` 和 `offline/make_eval_cases.py`，不进入 `solutions`。
- 用户提供的 `team006_final.zip` 复制到可写测试目录并保持官方文件名后，通过官方 ZIP 检查。仓库根目录 `team006_final.zip` 已用部署后的 V6 三源码重打包、保留原有 place 示例文件，也通过官方 ZIP 检查。检查通过不代表性能达标或规则最终认证。

## 复现

```powershell
$py='E:\envs\swarm_rescue\python.exe'
& $py bench_tools\run_benchmarks.py --repo . --cases bench_cases_user_v6 --output bench_results\v6_user_fixed_one_round --maps Map01 Map02 Map03 Map04 Map05 Map06 Map07 Map07t4 Map08 Map09 --kinds user_fixed --repeats 1 --timeout 300 --python $py
& $py scripts\check_submission.py .
& $py scripts\check_submission.py team006_final.zip
```

重跑会覆盖同名结果，随机初始姿态可能改变成绩。本报告只主张这十轮原样固定坐标实测，不声称多种子稳定性、随机边角鲁棒性、92 分或全部地图 10/10。

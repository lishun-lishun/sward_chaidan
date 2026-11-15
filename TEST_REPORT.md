# Swarm-Rescue 蓝方实测报告

日期：2026-11-15（Asia/Singapore）  
环境：Windows，`E:\envs\swarm_rescue\python.exe`，Arcade 2.6.17，Pymunk 6.4.0  
运行方式：全部有效回合均使用官方 `python -m swarm_rescue.launcher --competition --headless`。

## 结论

当前版本效果不达标，不建议以“10/10、92 分方案”名义提交。基线组 30 轮平均完成 5.07/10、平均 34.02 分；仅 3/30 轮完成 10/10，最高 83.2 分，没有任何一轮达到 92 分。复杂地图 Map07、Map07t4 和无 GPS 的 Map08 表现尤其差。

用户附件缺少 `offline/make_eval_cases.py`，因此也缺少说明中所称的 100 个固定炸弹坐标。为避免伪造，本报告把基于公开 `map_data`、seed 0 离线生成的组明确标为 `generated_fixed_seed0`，它不是用户原始固定组。边角随机矩阵原计划 90 轮，在用户中止后完成 10 轮；不把未执行回合写成成绩。

## 基线组汇总（每图 3 轮）

|地图|10/10 轮数|平均完成|平均分|最低/最高分|平均步数|平均健康%|坠毁总数|
|---|---:|---:|---:|---:|---:|---:|---:|
|Map01|0/3|5.67|46.10|36.6 / 59.9|2000|60.5|0|
|Map02|0/3|4.67|28.63|18.2 / 36.7|5000|3.1|27|
|Map03|0/3|5.67|35.93|24.2 / 45.8|5000|9.9|21|
|Map04|1/3|9.33|64.97|55.6 / 83.2|4043|17.5|19|
|Map05|2/3|9.67|65.10|55.8 / 79.0|5769|6.9|22|
|Map06|0/3|8.33|51.00|42.6 / 56.0|5400|4.9|27|
|Map07|0/3|0.67|5.17|1.9 / 7.4|5400|6.0|23|
|Map07t4|0/3|1.00|7.67|6.9 / 8.4|5400|8.3|21|
|Map08|0/3|0.00|0.53|0.0 / 1.3|5400|2.7|27|
|Map09|0/3|5.67|35.10|31.1 / 42.4|5400|5.6|25|
|合计|3/30|5.07|34.02|0.0 / 83.2|-|-|212|

完成的 10/10 回合：Map04 r1（83.2 分，2130 步，健康 34%，2 架坠毁）；Map05 r1（79.0 分，2943 步，健康 10.4%，5 架坠毁）；Map05 r3（60.5 分，7164 步，健康 1.6%，9 架坠毁）。

## 边角随机组（已完成部分）

Map01 完成 seeds 11/29/53 各 3 轮，共 9 轮：完成数为 `9,6,7 / 7,2,5 / 5,7,5`，平均 5.89/10，平均 47.09 分，0/9 达到 10/10。Map02 seed11 仅完成 r1：3/10、19.2 分、健康 6%、8 架坠毁。其余 80 轮因用户中止未运行。

逐轮机器可读数据见 `bench_results/final_matrix/results.json` 和 `bench_results/random_matrix/results.json`，原始日志同目录（Git 中以 force-add 保存）。测试输入见 `bench_cases/`。

## 故障分析

1. 碰撞与健康是主要失败源。除 Map01 外，大多数长图在跑满步数前已有 7–10 架坠毁；30 轮累计 212 架次坠毁。
2. 运输吞吐不足。多数回合能找到并抓住炸弹，但载弹 A*、局部避障和中心末端接触耗时过长；Map04/05 偶尔 10/10，但方差很大。
3. Map07/Map07t4 的狭窄拓扑与当前 24 px 栅格、未知空间代价、局部方向采样不匹配，平均仅 0.67/1.0 枚。
4. Map08 无 GPS 下航位推算漂移严重，三轮均 0/10；当前只有里程计积分和 GPS 恢复融合，没有 scan matching。
5. Map06 无通信下 claim/地图同步中断，虽然平均 8.33 枚，但三轮均有 9 架坠毁。
6. 40 个已完成正式矩阵日志中 16 个出现官方引擎 Pymunk CFFI callback 的 `AssertionError`（36 次），位置在 `simulation/gui_map/collision_handlers.py` / `playground.py`。进程均 exit 0 并输出官方分数；未修改引擎，报告原样保留该异常。

## 测试与检查

- 相关官方测试：22 passed（传感器输出、处置计数、处置中心 guard）。
- 工作区 submission check：passed。
- 有效矩阵：40/40 进程 exit 0，0 timeout；其中 16 个日志含上述被 CFFI 忽略的引擎 callback traceback。
- Conda 包装器在本机报 DLL relocation，实际测试直接调用环境 Python，并用 `PYTHONPATH=<当前仓库>/src` 防止旧 editable-install 污染。

## 复现

```powershell
$env:PYTHONPATH=(Resolve-Path src).Path
E:\envs\swarm_rescue\python.exe bench_tools\generate_eval_cases.py --repo . --out bench_cases --seeds 11 29 53
E:\envs\swarm_rescue\python.exe bench_tools\run_benchmarks.py --repo . --cases bench_cases --output bench_results\final_matrix --kinds generated_fixed_seed0 --repeats 3 --python E:\envs\swarm_rescue\python.exe
E:\envs\swarm_rescue\python.exe scripts\check_submission.py .
```


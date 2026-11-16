# 蓝方 V2 真实仿真报告

测试日期：2026-11-16（Asia/Singapore）。源码来自用户提供的 `blue_redesign_solutions.zip`，只替换 `src/swarm_rescue/solutions/` 内的 `blue_nav.py`、`blue_policy.py`、`my_drone_eval.py`；后者与原入口相同。仿真使用当前仓库的官方 launcher、`--competition --headless`、10 架无人机和每图 10 枚炸弹。`PYTHONPATH` 指向当前仓库 `src`，避免 Conda 环境中旧 editable install 干扰。

## 结论

V2 在十图各一轮中只完成 11/100 枚炸弹，平均每图 1.10/10、16.36 分，0/10 回合达到 10/10，也没有 92 分回合。平均健康 48.88%，累计 18 架次坠毁。相比旧版生成基线 30 轮平均 5.07/10、34.02 分，V2 的健康总体改善，但拆弹吞吐显著退化。两批运行的初始朝向随机，且轮次数不同，不能把差异解释为严格因果估计。

本次按用户追加指示每图只跑一轮。附件没有 `bench_tools/make_cases.py`、其 `FIXED` 坐标表或声称的 100 个用户固定坐标，所以本次使用仓库已有的 `bench_cases/MapXX_generated_fixed_seed0.json`。这个布局由离线生成器产生，**不是**用户固定组；三个边角随机种子本次没有重跑。不能据此宣称满足原始固定/随机验收矩阵。

## 官方逐图结果

|地图|拆弹|总分|步数/上限|健康分%|坠毁架数|仿真墙钟秒|终止原因|
|---|---:|---:|---:|---:|---:|---:|---|
|Map01|0/10|16.9|2000/2000|84.4|0|40|步数上限|
|Map02|2/10|21.5|5000/5000|47.6|2|88|步数上限|
|Map03|1/10|19.0|5000/5000|65.2|1|118|步数上限|
|Map04|1/10|18.7|5000/5000|63.6|1|96|步数上限|
|Map05|2/10|19.4|7200/7200|37.2|3|169|步数上限|
|Map06（无通信）|3/10|26.3|5400/5400|41.6|3|144|步数上限|
|Map07|1/10|15.6|4447/5400|48.0|2|180|墙钟上限|
|Map07t4|0/10|7.0|3939/5400|34.8|2|180|墙钟上限|
|Map08（无 GPS）|0/10|4.2|3738/5400|21.2|2|180|墙钟上限|
|Map09（无通信、无 GPS）|1/10|15.0|5400/5400|45.2|2|168|步数上限|

全部十轮进程退出码为 0，外部 300 秒超时为 0，原始日志未出现 traceback。所有回合未全部交付，故官方时间分均为 0。逐轮 JSON、原始 launcher 日志分别见 `bench_results/v2_pilot/`（Map07/08）和 `bench_results/v2_other_maps/`（其余八图）。

## 诊断

1. V2 速度量纲修订降低了撞毁：例如 Map02 此轮 2 架坠毁，而旧版同布局三轮均为 9 架；但只有 2/10 完成，安全与任务推进没有平衡。
2. Map01 无墙钟截断且健康 84.4%，却 0/10；这是抓取、寻路或交付路径未形成有效闭环的证据，不能归因于仿真时间不够。
3. Map07、Map07t4、Map08 的实际帧率分别约 24.7、21.9、20.8 步/秒，低于 5400 步在 180 秒内完成所需的 30 步/秒；因此规划/地图处理开销本身阻碍了完整回合。仍需进一步 profiling 才能定位具体热点。
4. 无 GPS 的 Map08 0/10，无通信的 Map06 3/10，二者均未验证所要求的鲁棒性。当前日志没有足够的逐机合法传感器轨迹，无法精确判定失锁误差或中心对接失败次数。
5. 用户提供的 V2 说明声称有专用单测与固定坐标生成器，但压缩包只有三份比赛源码；本次只能运行仓库已有的相关测试与生成布局。未执行附件中缺失的脚本。

## 合法性与提交检查

- `python -m py_compile` 三份比赛源码：通过。
- 官方工作区 `scripts/check_submission.py .`：通过。
- 相关官方 pytest：22 passed（传感器、处置计数、处置中心 guard）。
- 入口保留 place 模式原样例类；rescue 指向 V2 `MyDroneRescue`。
- 控制器仅使用测量、传感器、里程计和官方通信；未发现 `true_*` 调用、直接读取离线炸弹文件或跨无人机全局状态。`CENTER_PRIORS` 仍包含公开中心位置先验，人工规则合规性应由比赛组织方最终判断。
- 打包的 `team006_final.zip` 已运行官方 ZIP 提交检查，结果见提交记录。

## 复现命令

```powershell
$env:PYTHONPATH=(Resolve-Path src).Path
$py='E:\envs\swarm_rescue\python.exe'
& $py bench_tools\run_benchmarks.py --repo . --cases bench_cases --output bench_results\v2_pilot --maps Map07 Map08 --kinds generated_fixed_seed0 --repeats 1 --timeout 300 --python $py
& $py bench_tools\run_benchmarks.py --repo . --cases bench_cases --output bench_results\v2_other_maps --maps Map01 Map02 Map03 Map04 Map05 Map06 Map07t4 Map09 --kinds generated_fixed_seed0 --repeats 1 --timeout 300 --python $py
& $py scripts\check_submission.py .
```

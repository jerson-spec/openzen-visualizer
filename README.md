# OpenZen Visualizer

基于 [LP-Research OpenZen](https://bitbucket.org/lpresearch/openzen) 传感库（MIT License）二次开发的
IMU 可视化工具（Windows），用于 LPMS 系列传感器的连接、实时显示、姿态融合与磁力计校准实验。

上游 OpenZen 库保留在 `src/`、`include/`、`external/`、`cmake/`（仅保留本项目用到的部分：
核心库 + 依赖库，已移除 C#/Python 绑定、示例、测试、文档与 CI 配置）。

## 功能

- 蓝牙自动扫描连接（优先经典蓝牙 SPP，兼容 BLE），100 Hz IMU 数据流，
  Control Panel 实时显示链路类型与实测数据率
- 实时曲线：加速度 / 陀螺仪 / 磁力计（原始 + 校准 |B| 对比）/ 欧拉角
- 三个姿态融合滤波器并行运行与对比：Madgwick / 互补滤波 / EKF
- **磁力计椭球校准**（`visualizer/MagEllipsoidCalib.h`）：9 参数二次曲面线性最小二乘拟合，
  含 3D 点云视图、三正交平面投影、球面误差曲线、硬铁/软铁/各向异性等诊断指标、
  校准失效实时健康检测
- **Field Survey 网格磁扫描**（`visualizer/GuiRenderer.h`）：静止判稳（陀螺/加速度门控 +
  半窗收敛检验）+ 圆周平均航向 + 圈均值统计，一键记录 `field_survey.txt`，
  用于室内固定磁物体干扰场的逐格测量（后续磁场建图的数据来源）
- 数据记录：`imu_data_*.txt`（TSV，37 列，Excel 可直接打开）

## 编译

依赖：Visual Studio 2022、CMake 3.14+。

```bash
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release --target OpenZenVisualizer
```

产物：`build/visualizer/Release/OpenZenVisualizer.exe`。
首次配置时会从 LP-Research 的 Bitbucket 下载 Windows 串口/USB 二进制驱动库，需要联网。

日常使用请运行 **Release** 版（Debug 版渲染性能差 5-10 倍）。

## License

上游 OpenZen 部分遵循其原始 MIT 许可（见 [LICENSE](LICENSE)，Copyright LP-Research Inc.）；
`visualizer/` 目录为本项目新增代码。

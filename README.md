<p align="center">
  <img src="https://img.shields.io/badge/MATLAB-R2020a+-0076A8?style=flat" alt="MATLAB">
  <img src="https://img.shields.io/badge/Simulink-Control_Systems-FF6600?style=flat" alt="Simulink">
  <img src="https://img.shields.io/badge/Tutorials-21-blue?style=flat" alt="21 Tutorials">
  <img src="https://img.shields.io/badge/Language-ZH-red?style=flat" alt="Chinese">
  <img src="https://img.shields.io/badge/License-MIT-green?style=flat" alt="MIT License">
  <img src="https://img.shields.io/github/stars/xingd5478-ctrl/simulink-control-tutorial?style=flat" alt="Stars">
</p>

<h1 align="center">Simulink 控制工程教程</h1>
<p align="center"><strong>从零基础到嵌入式代码部署</strong></p>

---

## 效果预览

所有教程一键运行（`run_all_tutorials`）自动生成的结果图，更多见 [docs/images](docs/images)：

| PID 闭环控制 (t05) | LQR 最优控制 (t10) |
|:---:|:---:|
| ![PID](docs/images/t05_pid_control_fig1.png) | ![LQR](docs/images/t10_state_feedback_fig1.png) |
| **Kalman 滤波 (t12)** | **PMSM 矢量控制 FOC (t14)** |
| ![Kalman](docs/images/t12_kalman_filter_fig1.png) | ![FOC](docs/images/t14_foc_pmsm_fig1.png) |

---

## 教程总览

21 个教程，分三个阶段，覆盖从基础操作到企业级控制算法的完整学习路径。

### Phase 1 — Simulink 基础 (t01-t08)

| # | 教程 | 内容 | 模型 |
|:---|:---|:---|:---:|
| t01 | 信号基础 | 信号、增益、示波器 | ✅ |
| t02 | 数学运算 | 加法、乘法、Mux/Demux | ✅ |
| t03 | 一阶系统 | 传递函数、时间常数 | ✅ |
| t04 | 二阶系统 | 阻尼比、超调量 | ✅ |
| t05 | PID 控制 | P/I/D 各环节作用、反馈闭环 | ✅ |
| t06 | 信号源与输出 | 数据导入导出、多种信号源 | ✅ |
| t07 | 子系统 | 模型层次化封装 | ✅ |
| t08 | Mask 封装 | 参数化模块设计 | ✅ |

### Phase 2 — 控制理论 (t09-t12)

| # | 教程 | 核心理论 | 工程价值 |
|:---|:---|:---|:---|
| t09 | 状态空间模型 | ẋ=Ax+Bu, y=Cx+Du | 现代控制理论基础 |
| t10 | LQR 最优控制 | 极点配置、Riccati 方程、Q/R 调参 | 多变量系统设计 |
| t11 | 状态观测器 | Luenberger Observer、对偶性、(A-LC) | 无传感器控制 |
| t12 | Kalman 滤波 | lqe()、Q/R 噪声建模、Luenberger 对比 | 噪声环境最优估计 |
| t16 | 频域分析 | Bode图、Nyquist图、增益/相位裕度 | 频域稳定性判据 |
| t17 | 超前-滞后校正 | 根轨迹设计、Lead/Lag 补偿器 | 经典控制设计方法 |
| t18 | 系统辨识 | 阶跃响应法、最小二乘、模型验证 | 从实验数据到数学模型 |
| t19 | H∞ 鲁棒控制 | 混合灵敏度、hinfsyn、μ 分析 | 不确定性下的最优控制 |
| t20 | 模型预测控制 MPC | 滚动优化、QP 约束求解、显式 MPC | 带约束的多变量控制 |
| t21 | 滑模控制 SMC | 滑模面、抖振抑制、Super-Twisting | 非线性鲁棒控制 |

### Phase 3 — 机电系统 + 部署 (t13-t15)

| # | 教程 | 核心理论 | 工程价值 |
|:---|:---|:---|:---|
| t13 | DC 电机控制 | 电磁+机械耦合、级联 PI、LQR 对比 | 执行器建模 |
| t14 | PMSM + FOC | d-q 变换、Clarke/Park、矢量控制 | 无刷电机控制 |
| t15 | 代码生成 | c2d 离散化、dlqr、C 代码、FreeRTOS | 嵌入式部署 |

---

## 使用方法

每个教程是一个 `.m` 脚本，在 MATLAB 命令窗口运行：

```matlab
>> t01_signal_basics       % 基础
>> t10_state_feedback      % LQR 控制
>> t15_code_generation     % 生成 C 代码

>> run_all_tutorials       % 一键依次运行全部 21 课，图像自动保存到 docs/images/
```

脚本会自动创建 Simulink 模型（`.slx`）、运行仿真、生成结果图。

需要 **Control System Toolbox**（`place`, `lqr`, `lqe`, `ss2tf`, `c2d` 等函数）。

---

## 学习路径

```
t01-t08 基础操作 ──→ t09 状态空间 ──→ t10 LQR 控制
                     │    │               │
                     │    ├── t11 观测器 ←── t12 Kalman
                     │    │
                     ├── t16 频域分析 ──→ t17 校正器设计
                     │
                     ├── t18 系统辨识
                     │
                     ├── t19 H∞ 鲁棒控制
                     │
                     ├── t20 MPC 模型预测控制
                     │
                     └── t21 滑模控制 (SMC)
                         │
         ┌───────────────┘
         ↓
    t13 DC电机 ←── t14 PMSM/FOC ←── t15 代码生成 → 嵌入式部署
```

21 个教程学完，能做到：
- 从物理定律推导状态空间模型
- 设计 LQR/Kalman 最优控制器和观测器
- 连续→离散→C 代码，部署到 STM32
- 理解级联 PI、FOC 矢量控制的工业实践

---

## 配套项目

- [STM32-MPU6050-System](https://github.com/xingd5478-ctrl/STM32-MPU6050-System) — MEMS 陀螺仪 Allan 方差 + 自适应 Kalman 实物项目
- [ebpf-robot-safety](https://github.com/xingd5478-ctrl/ebpf-robot-safety) — eBPF 实时控制安全监控

---

## License

MIT — 可自由使用、修改、分发，详见 [LICENSE](LICENSE)。

如果这套教程对你有帮助，欢迎点个 Star，也欢迎提 Issue 反馈问题或建议新专题。

---

<p align="center">
  <sub>邢栋 · 2026</sub>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/MATLAB-R2020a+-0076A8?style=flat-square&logo=matlab" alt="MATLAB">
  <img src="https://img.shields.io/badge/Simulink-Control_Engineering-FF6600?style=flat-square" alt="Simulink">
  <img src="https://img.shields.io/badge/Tutorials-25_lessons-blue?style=flat-square" alt="25 Tutorials">
  <img src="https://img.shields.io/badge/License-MIT-green?style=flat-square" alt="MIT">
  <img src="https://img.shields.io/badge/Language-中文-red?style=flat-square" alt="Chinese">
  <img src="https://img.shields.io/github/stars/xingd5478-ctrl/simulink-control-tutorial?style=flat-square" alt="Stars">
  <img src="https://img.shields.io/github/last-commit/xingd5478-ctrl/simulink-control-tutorial?style=flat-square" alt="Last Commit">
  <img src="https://img.shields.io/github/actions/workflow/status/xingd5478-ctrl/simulink-control-tutorial/ci.yml?style=flat-square&label=CI" alt="CI">
</p>

<h1 align="center">Simulink 控制工程教程</h1>
<p align="center">
  <strong>从零基础到嵌入式代码部署 — 25 课系统学习路径</strong><br>
  覆盖经典控制、现代控制、鲁棒/非线性/MPC、自适应/模糊、物理建模到代码生成
</p>

---

## 目录

- [效果预览](#效果预览)
- [教程总览](#教程总览)
- [快速开始](#快速开始)
- [前置要求](#前置要求)
- [学习路径](#学习路径)
- [教程详解](#教程详解)
- [学完后能做什么](#学完后能做什么)
- [项目结构](#项目结构)
- [配套项目](#配套项目)
- [贡献指南](#贡献指南)
- [License](#license)

---

## 效果预览

所有教程一键运行（`run_all_tutorials`）自动生成的结果图，更多见 [`docs/images`](docs/images):

<table>
<tr>
  <td align="center"><b>PID 闭环控制 (t05)</b></td>
  <td align="center"><b>LQR 最优控制 (t10)</b></td>
</tr>
<tr>
  <td><img src="docs/images/t05_pid_control_fig1.png" alt="PID"></td>
  <td><img src="docs/images/t10_state_feedback_fig1.png" alt="LQR"></td>
</tr>
<tr>
  <td align="center"><b>Kalman 滤波 (t12)</b></td>
  <td align="center"><b>PMSM 矢量控制 FOC (t14)</b></td>
</tr>
<tr>
  <td><img src="docs/images/t12_kalman_filter_fig1.png" alt="Kalman"></td>
  <td><img src="docs/images/t14_foc_pmsm_fig1.png" alt="FOC"></td>
</tr>
</table>

---

## 快速开始

### 第一步：克隆项目

```matlab
git clone https://github.com/xingd5478-ctrl/simulink-control-tutorial.git
```

### 第二步：检查环境

在 MATLAB 中打开项目目录，运行环境检查脚本：

```matlab
>> check_setup
```

### 第三步：开始第一课

```matlab
>> t00_main_guide            % 查看全部 25 课总览
>> t01_signal_basics         % 第 1 课：正弦波 + 增益 + 示波器
```

### 第四步：按学习路径推进

```matlab
% 基础阶段 (t01-t08) — 不需要额外工具箱
% 经典控制 (t16, t17, t22) — 需要 Control System Toolbox
% 现代控制 (t09-t12) — 状态空间、LQR、观测器、Kalman
% 高级控制 (t18-t21, t23, t24) — 鲁棒、MPC、滑模、自适应、模糊
% 应用部署 (t13, t14, t25, t15) — 电机、Simscape、代码生成

% 也可以一键运行全部（比较耗时，建议学完基础后再试）
>> run_all_tutorials
```

每个脚本会**自动创建 Simulink 模型 (.slx) → 配置模块 → 运行仿真 → 生成结果图**，你不需要手动搭任何模型。

### 运行完一课后做什么

1. **双击 Scope 模块** — 查看仿真波形
2. **双击其他模块** — 看看参数是怎么设置的
3. **修改参数** — 比如把增益从 2 改成 5，重新运行看变化
4. **观察生成的 MATLAB Figure** — 里面的对比图帮你理解概念

---

## 前置要求

| 依赖 | 说明 | 阶段 |
|:---|:---|:---|
| MATLAB R2020a+ | 核心运行环境 | 全部 |
| Simulink | 模型搭建与仿真 | 全部 |
| Control System Toolbox | 状态空间、LQR、Kalman、频域分析 | Phase 2-5 |
| Robust Control Toolbox | H∞ / μ 分析 | t19 (可选) |
| Model Predictive Control Toolbox | MPC 设计 | t20 (可选) |
| Fuzzy Logic Toolbox | 模糊推理系统 | t24 (可选) |
| Simscape | 物理建模 | t25 (可选) |

> **新手提示**：t01-t08 只需 MATLAB + Simulink，不需要额外工具箱。安装 MATLAB 时勾选 Simulink 即可。运行 `check_setup` 可以一键检查你的环境是否满足要求。

---

## 常见问题 (FAQ)

<details>
<summary><b>Q: 我完全没学过控制理论，能学吗？</b></summary>

能。Phase 1 (t01-t08) 只教 Simulink 操作，不涉及控制理论。学完 Phase 1 你就能熟练搭建 Simulink 模型了。后续教程会在需要的地方解释控制概念。
</details>

<details>
<summary><b>Q: 运行脚本时报 "Undefined function or variable"？</b></summary>

确保 MATLAB 的**当前文件夹**是项目根目录（`simulink-control-tutorial/`）。在 MATLAB 的"当前文件夹"面板中导航到该目录即可。
</details>

<details>
<summary><b>Q: 仿真没有波形输出？</b></summary>

双击模型窗口里的 **Scope** 模块，如果波形是一条直线，检查 Step 模块的 `Step time` 参数（通常在 0.5s 处跳变，不是 0s）。
</details>

<details>
<summary><b>Q: 模型窗口太多，怎么关闭？</b></summary>

在 MATLAB 命令窗口输入：
```matlab
>> bdclose all     % 关闭所有 Simulink 模型
>> close all       % 关闭所有 Figure
```
</details>

<details>
<summary><b>Q: 我想重新生成某个教程的模型，怎么做？</b></summary>

直接重新运行该教程脚本即可，它会自动覆盖旧的 .slx 文件：
```matlab
>> t05_pid_control
```
</details>

---

## 教程总览

25 个教程，分五个阶段，按学习路径从基础到工业级应用循序递进。

### Phase 1 — Simulink 基础与系统动力学 (t01-t08)

| # | 教程 | 内容 | 模型 |
|:---:|:---|:---|:---:|
| t01 | 信号基础 | 信号、增益、示波器 | ✅ |
| t02 | 数学运算 | 加法、乘法、Mux/Demux 信号路由 | ✅ |
| t03 | 一阶系统 | 传递函数、时间常数 | ✅ |
| t04 | 二阶系统 | 阻尼比、超调量、固有频率 | ✅ |
| t05 | PID 控制 | P/I/D 各环节作用、反馈闭环 | ✅ |
| t06 | 信号源与输出 | 数据导入导出、多种信号源 | ✅ |
| t07 | 子系统 | 模型层次化封装 | ✅ |
| t08 | Mask 封装 | 参数化模块设计 | ✅ |

### Phase 2 — 经典控制设计 (t16, t17, t22)

| # | 教程 | 核心理论 | 工程价值 |
|:---:|:---|:---|:---|
| t16 | 频域分析 | Bode图、Nyquist图、增益/相位裕度 | 频域稳定性判据 |
| t17 | 超前-滞后校正 | Lead/Lag 补偿器、频率整形 | 经典控制设计方法 |
| t22 | 根轨迹法 | rlocus()、极点走向、补偿器设计 | 经典控制设计可视化 |

### Phase 3 — 现代控制理论 (t09-t12)

| # | 教程 | 核心理论 | 工程价值 |
|:---:|:---|:---|:---|
| t09 | 状态空间模型 | ẋ=Ax+Bu, y=Cx+Du | 现代控制理论基础 |
| t10 | LQR 最优控制 | 极点配置、Riccati 方程、Q/R 调参 | 多变量系统设计 |
| t11 | 状态观测器 | Luenberger Observer、对偶性、(A-LC) | 无传感器控制 |
| t12 | Kalman 滤波 | lqe()、Q/R 噪声建模、Luenberger 对比 | 噪声环境最优估计 |

### Phase 4 — 高级控制专题 (t18-t21, t23, t24)

| # | 教程 | 核心理论 | 工程价值 |
|:---:|:---|:---|:---|
| t18 | 系统辨识 | 阶跃响应法、最小二乘、模型验证 | 从实验数据到数学模型 |
| t19 | H∞ 鲁棒控制 | 混合灵敏度、hinfsyn、μ 分析 | 不确定性下的最优控制 |
| t20 | 模型预测控制 MPC | 滚动优化、QP 约束求解、显式 MPC | 带约束的多变量控制 |
| t21 | 滑模控制 SMC | 滑模面、抖振抑制、Super-Twisting | 非线性鲁棒控制 |
| t23 | MRAC 自适应控制 | MIT 规则、参考模型、参数自适应 | 参数时变系统的在线调节 |
| t24 | 模糊逻辑控制 | Sugeno FIS、隶属度函数、控制曲面 | 免模型专家经验控制 |

### Phase 5 — 机电系统与部署 (t13, t14, t25, t15)

| # | 教程 | 核心理论 | 工程价值 |
|:---:|:---|:---|:---|
| t13 | DC 电机控制 | 电磁+机械耦合、级联 PI、LQR 对比 | 执行器建模 |
| t14 | PMSM + FOC | d-q 变换、Clarke/Park、矢量控制 | 无刷电机控制 |
| t25 | Simscape 物理建模 | 物理元件连线、多域耦合、自动推导 | 免公式推导的建模方法 |
| t15 | 代码生成 | c2d 离散化、dlqr、C 代码、FreeRTOS | 嵌入式部署 |

---

## 学习路径

```
Phase 1 (基础)       Phase 2 (经典)      Phase 3 (现代)       Phase 4 (高级)       Phase 5 (应用)
                                                                                
t01-t08 基础操作 ──→ t16 频域分析 ──→ t09 状态空间 ──→ t18 系统辨识 ──→ t13 DC电机
                     │                  │                  │                  │
                     ├── t17 校正器     ├── t10 LQR       ├── t19 H∞鲁棒    ├── t14 FOC
                     │                  │                  │                  │
                     └── t22 根轨迹     ├── t11 观测器    ├── t20 MPC       ├── t25 Simscape
                                        │                  │                  │
                                        └── t12 Kalman    ├── t21 滑模       └── t15 代码生成
                                                          │
                                                          ├── t23 MRAC
                                                          │
                                                          └── t24 模糊控制
```

---

## 教程详解

<details>
<summary><b>t01-t08 Simulink 基础</b> — 点击展开</summary>

- **t01 信号基础**: 认识 Simulink 模块库，搭建第一个信号流模型，理解 Gain/Scope 的作用
- **t02 数学运算**: Sum/Product/Mux/Demux 模块，掌握信号的路由与运算
- **t03 一阶系统**: 传递函数建模，理解时间常数 τ 对响应速度的影响
- **t04 二阶系统**: 阻尼比 ζ 与超调量的关系，从时域波形反推系统参数
- **t05 PID 控制**: P/I/D 三环节独立验证 + 联合闭环调参，直观理解"为什么需要 D"
- **t06 信号源与输出**: 从 Workspace 导数据到 Simulink，仿真结果回存 Workspace
- **t07 子系统**: 将复杂模型折叠成黑盒，理解层次化建模思想
- **t08 Mask 封装**: 自定义参数对话框，让子系统可复用、可配置
</details>

<details>
<summary><b>t16, t17, t22 经典控制设计</b> — 点击展开</summary>

- **t16 频域分析**: Bode/Nyquist/Nichols 图，增益裕度与相位裕度，频域稳定性判据
- **t17 校正器**: Lead/Lag 补偿器设计，频率整形，改善瞬态与稳态性能
- **t22 根轨迹法**: 开环增益 K 从 0→∞ 时闭环极点在复平面的运动轨迹，rlocus() 自动绘制，sgrid 叠加设计约束
</details>

<details>
<summary><b>t09-t12 现代控制理论</b> — 点击展开</summary>

- **t09 状态空间**: 从传递函数到状态空间，理解"状态"的物理意义，能控/能观性
- **t10 LQR**: 极点配置 → LQR 最优控制，Riccati 方程，Q/R 权重矩阵调参策略
- **t11 观测器**: Luenberger 观测器设计，对偶原理，分离原理验证
- **t12 Kalman**: 过程噪声与测量噪声建模，lqe() 设计，与 Luenberger 对比
</details>

<details>
<summary><b>t18-t21, t23, t24 高级控制专题</b> — 点击展开</summary>

- **t18 系统辨识**: 阶跃响应法、最小二乘法，从实验数据拟合传递函数，模型验证
- **t19 H∞ 鲁棒**: 不确定性建模 (ureal/uss)，混合灵敏度 S/KS/T 整形，μ 分析
- **t20 MPC**: 预测模型、滚动优化、QP 约束求解，约束下的多变量控制
- **t21 滑模控制**: 滑模面设计、等效控制、抖振抑制（饱和函数/超螺旋算法）
- **t23 MRAC 自适应**: 参考模型指定理想行为，MIT 规则在线调节控制器参数，参数突变时自动恢复
- **t24 模糊控制**: Sugeno 型模糊推理系统，5×5 规则表，高斯隶属度函数，无模型专家经验控制
</details>

<details>
<summary><b>t13, t14, t25, t15 机电系统与部署</b> — 点击展开</summary>

- **t13 DC 电机**: 电磁转矩 + 机械负载耦合建模，级联 PI（电流环+转速环），LQR 对比
- **t14 PMSM/FOC**: Clarke → Park 坐标变换，d-q 解耦，SVPWM 原理，矢量控制全流程
- **t25 Simscape**: 质量-弹簧-阻尼系统物理建模，与传统 Transfer Fcn 对比验证，多物理域耦合概念
- **t15 代码生成**: 连续→离散 (c2d)，离散 LQR (dlqr)，生成 C 代码，FreeRTOS 集成框架
</details>

---

## 学完后能做什么

完成全部 25 课，你将具备以下能力：

- 从物理定律推导状态空间模型
- 设计 LQR/Kalman 最优控制器和观测器
- 在 Bode/Nyquist 图上分析稳定性并设计补偿器
- 从实验数据辨识系统模型
- 处理参数不确定性，设计 H∞ 鲁棒控制器
- 为带约束系统设计 MPC 控制器
- 实现滑模控制应对非线性/大扰动
- 用根轨迹法分析极点走向并设计补偿器
- 设计 MRAC 自适应控制器应对参数时变系统
- 搭建模糊逻辑控制系统，不依赖精确数学模型
- 使用 Simscape 进行物理建模，免公式推导
- 连续→离散→C 代码，部署到 STM32 等嵌入式平台
- 理解级联 PI、FOC 矢量控制的工业实践

---

## 项目结构

```
.
├── check_setup.m                 # 新手环境检查（运行开始前先跑这个）
├── t00_main_guide.m              # 教程索引（运行查看全貌）
├── t01-t25_*.m                   # 25 个教程主脚本
├── models/                       # Simulink 模型文件（脚本自动生成）
│   └── tutorial01-tutorial25.slx
├── utils/                        # 共享工具函数
│   └── getSimData.m
├── run_all_tutorials.m           # 一键运行全部教程
├── docs/images/                  # 自动生成的仿真结果图
├── .github/                      # Issue/PR 模板
├── README.md                     # 本文件（中文）
├── README_EN.md                  # English version
├── CHANGELOG.md                  # 更新日志
├── CONTRIBUTING.md               # 贡献指南
├── CODE_OF_CONDUCT.md            # 社区行为准则
├── SECURITY.md                   # 安全政策
└── LICENSE                       # MIT 协议
```

---

## 配套项目

- [STM32-MPU6050-System](https://github.com/xingd5478-ctrl/STM32-MPU6050-System) — MEMS 陀螺仪 Allan 方差 + 自适应 Kalman 实物项目
- [ebpf-robot-safety](https://github.com/xingd5478-ctrl/ebpf-robot-safety) — eBPF 实时控制安全监控

---

## 贡献指南

欢迎 Issue 和 PR！提 bug、建议新专题、改进文档都算贡献。

- 发现 bug → 使用 [Bug Report](https://github.com/xingd5478-ctrl/simulink-control-tutorial/issues/new?template=bug_report.md) 模板
- 新专题建议 → 使用 [Tutorial Request](https://github.com/xingd5478-ctrl/simulink-control-tutorial/issues/new?template=tutorial_request.md) 模板
- 改进代码/文档 → Fork → 修改 → PR

详见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## Star History

<a href="https://star-history.com/#xingd5478-ctrl/simulink-control-tutorial&Date">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=xingd5478-ctrl/simulink-control-tutorial&type=Date&theme=dark" />
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=xingd5478-ctrl/simulink-control-tutorial&type=Date" />
    <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=xingd5478-ctrl/simulink-control-tutorial&type=Date" />
  </picture>
</a>

---

## License

MIT — 可自由使用、修改、分发，详见 [LICENSE](LICENSE)。

如果这套教程对你有帮助，欢迎点个 ⭐ Star，也欢迎提 Issue 反馈问题或建议新专题。

---

<p align="center">
  <sub>邢栋 · 2026</sub>
</p>

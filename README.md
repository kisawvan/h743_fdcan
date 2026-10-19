# h743_fdcan

面向机器人控制计算的 STM32H743 工程，包含 FDCAN 通信示例、串口输出及机器人矩阵、运动学与动力学计算代码。

## 当前内容

- STM32 HAL 外设初始化：GPIO、FDCAN1、TIM3、TIM5、USART1。
- FDCAN FD 无比特率切换模式的发送封装、FIFO0 接收回调及心跳示例。
- C++ 主程序中的六自由度连杆模型，以及正运动学、雅可比、逆运动学和递归牛顿欧拉逆动力学调用。
- 基于 CMSIS-DSP 的矩阵与机器人学代码，以及 MATLAB 机器人模型脚本。

当前属于通信与计算集成实验。本文按仓库源码整理，未重新编译、下载固件或进行总线、机械臂实机验证。

## 目录导航

| 路径 | 内容 |
| --- | --- |
| [fdcan/MDK-ARM/fdcan.uvprojx](fdcan/MDK-ARM/fdcan.uvprojx) | Keil 工程入口 |
| [fdcan/fdcan.ioc](fdcan/fdcan.ioc) | CubeMX 配置 |
| [fdcan/Core/Src/main.cpp](fdcan/Core/Src/main.cpp) | 当前 Keil 工程选用的主程序 |
| [fdcan/Core/Src/main.c](fdcan/Core/Src/main.c) | 另一个 C 入口文件，勿与 main.cpp 同时加入构建 |
| [fdcan/Core/Src/fdcan.c](fdcan/Core/Src/fdcan.c) | FDCAN 初始化、收发与回调 |
| [src/](src/) | Keil 工程实际引用的矩阵、机器人学和工具源码 |
| [fdcan/toolbox/](fdcan/toolbox/) | 另一份算法源码，部分文件与 src 不同 |
| [fdcan/matlab/](fdcan/matlab/) | PUMA560、SCARA、UR 模型脚本 |
| [fdcan/Drivers/](fdcan/Drivers/) | HAL、CMSIS 及 DSP 相关文件 |

## 构建与查看入口

1. 用 Keil MDK 打开上述 `.uvprojx`，检查芯片包、编译器和 CMSIS-DSP 数学库路径。
2. 工程目标为 `STM32H743IITx`；CubeMX 记录为 `STM32H743IIT6`、LQFP176，固件包记录为 `STM32Cube FW_H7 V1.12.1`。
3. 工程文件记录 `Keil.STM32H7xx_DFP.4.0.0`，并引用 `arm_cortexM7lfdp_math.lib`；应核对本机文件和浮点 ABI 配置。
4. 阅读通信示例时以 `main.cpp` 和 `fdcan.c` 为入口；阅读算法时以工程实际引用的根目录 `src/` 为入口。
5. 若重新生成 CubeMX 代码，先比较现有用户代码和工程配置；当前 `.ioc` 与手工修改后的 FDCAN 源码存在差异。

## 通信示例

| 项目 | 当前源码 |
| --- | --- |
| FDCAN1 RX / TX | PA11 / PA12 |
| 帧格式 | FD、标准 ID、BRS 关闭 |
| 仲裁时序 | Prescaler=2、Seg1=14、Seg2=5、SJW=1 |
| 数据时序 | Prescaler=2、Seg1=14、Seg2=5、SJW=1 |
| USART1 | 115200 波特率，PB14 / PB15 |
| 主循环示例帧 | ID `0x01`（4 字节）、`0x200`（8 字节）、`0x001`（2 字节） |
| 额外心跳函数 | `FDCAN1_SendHeartbeat()`，ID `0x100`；当前主循环未调用 |

波特率需用实际 FDCAN 内核时钟计算：`f_fdcan / (Prescaler × (1 + Seg1 + Seg2))`。不要直接把源码注释中的 500 kbit/s 当作实测结果。主循环还执行机器人计算，并等待 2000 ms，因此它不是固定周期的高速控制环。

## 机器人计算

`main.cpp` 构造 `robotics::Serial_Link<6>`，调用 `fkine`、`jacob`、`ikine` 与 `rne`。这些调用用于演示计算流程，模型参数、逆解收敛性和输出精度仍需通过独立算例及硬件对象验证。

## 后续工作

- 对齐 CubeMX 配置与实际源码，统一重复算法目录。
- 检查 FD 报文 DLC 编解码，尤其是超过 8 字节的长度；现有长度移位写法不能视作完整的 0–64 字节长度映射。
- 补充构建日志、总线抓包、计算结果对照和目标硬件接线说明。
- 根据实际电机协议接入控制与反馈，再验证控制周期。

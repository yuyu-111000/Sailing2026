# STM32F407VET6 红外探头板

F407只负责实时采集16个 TSOP34838，并通过 UART 向 L431发布每个通道的强度统计。它不运行导航状态机，也不直接控制电调。

## 目录

```text
include/sailing/f407_hal.hpp            硬件适配接口
include/sailing/infrared_publisher.hpp  采样快照到串口帧的发布逻辑
src/infrared_publisher.cpp              可移植实现
tests/test_main.cpp                     主机测试
```

`tests/fakes.hpp` 仅用于主机测试，禁止加入正式固件目标。

## CubeIDE移植接口

需要在未来的 STM32CubeIDE 工程内实现：

- `InfraredSampler::snapshot_and_reset()`：原子地复制16路统计值并清零下一采样窗；
- `ByteOutput::write()`：通过板间 UART 发送完整37字节帧。

TSOP34838输出是低有效的解调数字包络，并非UART。建议以外部中断或定时器输入捕获统计每个固定窗口内的有效低脉冲/低电平时间。采样中断更新计数，发布任务以50～100 Hz调用 `InfraredPublisher::publish()`。复制与清零计数时必须使用短临界区，避免丢计数。

## 通道约定

机械探头编号为 CGQ1～CGQ16，相邻22.5°。协议固定为：

```text
strength[0]  = CGQ1（约定船头0°）
strength[1]  = CGQ2
...
strength[15] = CGQ16
```

CGQ2相对CGQ1的实际旋转方向必须装船后确认。如果方向与L431算法的正角方向相反，应在F407通道映射表中一次性纠正，不要在多个控制公式中分别取反。

## 待提供

- F407原理图或 `.ioc`；
- 16路TSOP对应GPIO/定时器通道；
- 板间UART实例、引脚、波特率和DMA使用方式；
- 红外门信号的实际载波/编码波形；
- 探头板现有固件（若存在）。

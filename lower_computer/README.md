# 单主控红外自主航行固件核心

本目录是 C1 智能航行的唯一软件运行端，不需要上位机。主控同时读取 16 路红外探头板串口和 JY61 串口，计算目标门方向，并通过两路 PWM 控制双电调差速。

## 已确认的探头机械数据

`3D_船控探头测试板装配体_2026-07-07.step` 来自 EasyEDA Pro。装配体包含 CGQ1～CGQ16 共 16 个 TSOP34838，安装半径约 32.8 mm，相邻探头角间隔 22.5°，构成完整 360°环阵。固件约定 CGQ1 为船头 0°，CGQ2～CGQ16 沿编号方向依次为 22.5°～337.5°。

TSOP34838 的裸输出是解调数字脉冲，不是 UART。本项目假定探头板另有采集 MCU，将 16 路统计值通过 UART 发给主控。若实板没有采集 MCU，应将输入适配为 GPIO/定时器采样，导航核心无需改变。

## 红外探头板串口协议 v1（待与实板固件确认）

固定 37 字节，小端：

```text
A5 5A | sequence:u8 | strength[16]:u16 | CRC16:u16
```

- `strength[0]` 对应 CGQ1，依次到 `strength[15]` 对应 CGQ16。
- 强度建议定义为固定采样窗内的有效 38 kHz 包络脉冲数。
- CRC 为 CCITT-FALSE，覆盖前 35 字节，CRC 字段低字节先发。
- 推荐 115200 8N1、50～100 Hz；实际波特率由硬件适配层配置。

## JY61

支持维特智能标准 11 字节帧：`0x55 0x52` 角速度和 `0x55 0x53` 角度，校验为前 10 字节累加和。导航使用 Z 轴角速度做转向阻尼；航向角保留用于遥测和后续策略。

## 导航逻辑

1. 用 16 路强度的圆向量加权平均计算相对方位和置信度。
2. 无可信信号时原地搜索；有信号时按角度误差进行差速跟踪。
3. 转向量采用红外比例项和 JY61 偏航角速度阻尼，不使用易饱和的积分项。
4. 强信号且居中后进入接近判定；随后信号持续丢失才进入穿门直行。
5. 穿门直行窗口结束后门计数加一，重新搜索下一门；第 10 门后停车。
6. 300 秒超时、急停、撤销使能或输出失败均使双推进器归零。

穿门判定仅依据红外切换，是当前硬件条件下的工程假设，必须用赛事门架实测验证。建议后续由探头板输出“载波有效时长/脉冲计数”，不要只输出 16 位开关量。

## 需要实船标定的参数

参数集中在 `NavigationConfig`：有效强度、置信度、接近强度、红外丢失时间、穿门直行时间、搜索速度、基础油门、`Kp/Kd`、死区和左右电机补偿。所有默认值仅用于台架测试，不能直接视为比赛参数。

## 主控移植边界

实现 `hal.hpp` 中五个接口即可接入 STM32/其他 MCU：单调时钟、红外 UART、JY61 UART、双路归一化推进输出和物理使能/急停输入。`DifferentialDrive::apply(-1..1, -1..1)` 由硬件层换算成电调 PWM 脉宽并执行限幅、方向和上电解锁时序。

## 主机测试

```powershell
& 'D:\MinGW\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Wpedantic -Werror `
  -Ilower_computer/include lower_computer/src/sensors.cpp `
  lower_computer/src/navigation.cpp lower_computer/src/controller.cpp `
  lower_computer/tests/test_main.cpp -o build/lower_computer_tests.exe
& build/lower_computer_tests.exe
```

# STM32L431RCT6 船体主控

L431是整船唯一决策控制器：接收F407红外帧、解析JY61、运行导航状态机，并把归一化左右推力交给双电调PWM硬件层。

## 目录

```text
include/sailing/l431_hal.h   时钟、双UART、双推进器及使能/急停接口
include/sailing/jy61.h       JY61解析器
include/sailing/navigation.h 方位估计、参数和导航状态机
include/sailing/controller.h 单周期组合入口
src/                           可移植实现
tests/test_main.c              主机测试
```

`tests/fakes.h` 仅用于主机测试，禁止加入正式固件目标。

## 运行周期

`sailing_boat_controller_tick()` 是唯一周期入口，建议由1 ms系统节拍驱动，以5～10 ms周期调用。每次调用依次：

1. 排空F407串口与JY61串口接收缓冲；
2. 检查输出故障、急停和物理使能；
3. 计算红外方位与数据新鲜度；
4. 运行搜索、跟踪、穿门或终止状态；
5. 输出左右归一化推力 `[-1, 1]`。

## CubeIDE移植接口

需要实现 `l431_hal.h` 中的函数指针接口：

- `sailing_l431_clock_t`：不回拨的毫秒时钟回调；
- 两个 `sailing_l431_byte_input_t`：F407 UART和JY61 UART，推荐DMA环形缓冲；
- `sailing_l431_drive_t`：归一化推力到两路电调PWM、限幅和解锁时序；
- `sailing_l431_arm_input_t`：物理启动使能与急停。

调用 `sailing_boat_controller_init()` 时会立即要求电调归零。任何急停、撤销使能、300秒超时或输出失败都必须让两路PWM进入定义好的安全中位值。

## 调参入口

所有算法参数集中在 `sailing_navigation_config_t`，包括信号阈值、置信度、搜索转速、基础油门、`Kp/Kd`、电机死区、左右补偿、丢信号时间和穿门直行时间。默认值仅用于主机测试，必须经过台架和水池分层标定。

## 待提供

- L431原理图或 `.ioc`；
- F407 UART、JY61 UART和两路PWM引脚；
- JY61具体版本、接口电平、波特率和安装轴向；
- 电调型号、PWM频率/范围、解锁流程及正反转能力；
- 启动、急停、LED和蜂鸣器引脚。

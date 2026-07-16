# 单主控软件架构

```text
红外UART -> InfraredSerialDecoder --+
                                    +-> Navigator -> DifferentialDrive HAL -> 双路PWM
JY61 UART -> Jy61Decoder -----------+
物理使能/急停 ----------------------+
单调时钟 ---------------------------+
```

`BoatController::tick()` 是唯一周期入口，依次完成串口排空、急停/使能检查、导航计算和执行器输出。解析器、导航器和 HAL 均不依赖 MCU 厂商库，实际工程只需提供串口、时钟、GPIO 与 PWM 适配。

热路径不使用动态分配、异常、文件系统或网络。传感器数据必须满足 CRC/校验和及新鲜度要求；任何终止态输出均为双推进器零值。

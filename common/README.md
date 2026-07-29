# F407/L431公共红外链路

`common` 是两块MCU之间唯一允许共享的代码，目前只包含红外串口协议。不得把STM32 HAL、导航策略或执行器逻辑放入这里。

## 帧格式 v1

固定37字节，小端：

```text
A5 5A | sequence:u8 | strength[16]:u16 | crc16:u16
```

- `strength[0..15]` 对应 CGQ1～CGQ16；
- CRC采用CRC-16/CCITT-FALSE，覆盖前35字节；
- CRC低字节先发；
- 序号按成功发送的帧递增并自然回绕，L431拒绝重复序号帧刷新数据时间；
- 建议板间串口为115200 8N1、50～100 Hz，最终以原理图和链路实测为准。

F407调用 `encode_infrared_frame()`；L431持续调用 `InfraredStreamDecoder::feed()`。任何CRC失败的帧都不会替换上一份有效样本，L431还会按本地接收时刻执行新鲜度超时。

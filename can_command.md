# Can  Bus Config Odirve

## 常用命令

- 电机编码器校准，正转一圈反转一圈

```shell
cansend can0 207#0700000000000000
```

- 检测电机参数，在该命令大约 2s 后执行后会听到电机"哔"的一声。

```shell
cansend can0 207#0400000000000000
```

> 如果是电机启动后进行的编码器校准，则校准完成后电机不会进入闭环模式，需要再次输入命令手动开启闭环。

- 开启电机的闭环模式

```shell
cansend can0 207#0800000000000000
```

- 重启 odrive

```shell
cansend can0 216#0000000000000000
```





## Erros

- MotorError.CURRENT_SENSE_SATURATION

  莫名其妙点击停止转动，突然停止

- AxisError.DC_BUS_UNDER_VOLTAGE

  莫名其妙点击停止转动

- MotorError.DRV_FAULT

- MotorError.CURRENT_LIMIT_VIOLATION

  闭环模式下，手动强行转动电机



## Reference

- https://blog.csdn.net/gjy_skyblue/article/details/115412902 （速度模式配置）
- 

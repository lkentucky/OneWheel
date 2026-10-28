# 驱动端固件记录测试

无需采电流或连接示波器。本功能记录驱动软件和定时器寄存器的时序，不改变 FAST_FOC 控制参数、引脚或保护阈值。

## 烧录准备

1. CYT2BL3：已更新原驱动工程并完成 IAR Debug 编译。烧录 `project/iar/Debug_m4/Exe/cyt2bl3.hex`，沿用原板的烧录方法。
2. TC264：在 ADS 中重新编译 `C:/Users/csz18/Desktop/unicycle/Qcar` 并烧录，新增 UART/WiFi 记录转发功能。之前只有阶跃功能的主控固件不能导出记录。
3. 确认电脑和板子连接同一网络，`code/config.h` 的目标电脑 IP 与实际一致。连接 WiFi，选择 Mode8 并 Start，保持三路设定值为零。

驱动 HEX 只用于 CYT2BL3，不用于 TC264。驱动增加约 224 KiB 记录缓冲，最终 RAM 使用以 IAR map 为准。原文件已备份到 `validation/driver-trace-backup-*`。

## 测试

固定车体、确保飞轮可自由旋转后执行：

```powershell
python tools/mode8_step_capture.py --execute --driver-trace --wait 120
```

程序先确认 Mode8、零输出、启动斜坡完成、转速反馈新鲜和飞轮接近静止，再等待驱动返回带 CRC 的 ARM 确认。随后只触发一次既有的左飞轮 0/300/600/300/0 阶跃。右飞轮和地轮保持零。

驱动缓存第一阶跃前约 128 ms，以及第一条变化指令后约 7.5 秒的原始记录；板端阶跃本身 7.5 秒后归零。记录冻结且电机命令归零后，才通过串口和 WiFi 按块导出。采集/导出过程中保持 Mode8，不要重启驱动或再次 ARM。脚本会显示下载进度。

## 数据含义

- 驱动使用空闲的 TC_TIME2_CH2，8 MHz 自由运行计数器，所有事件共用同一时基。
- `received_ticks`：UART 中断读取到文本指令换行字节的时刻，**不是串口线上最后一位实际到达时刻**。
- `dispatched_ticks`：有效占空比命令写入驱动状态的时刻。
- `written_ticks`：对应命令的三相 PWM 缓冲寄存器写入完成后的时刻。
- `active_ticks`：后续编码器中断首次观察到三个活动比较寄存器均匹配此次目标值的时刻，观察粒度约为一个 50 μs 控制周期。
- 原有 20 kHz 编码器增量持续累加，约每 1 ms 保存角度、累积计数、命令、活动/缓冲 PWM 比较宽度、保护状态。这里没有原驱动转速低通滤波。
- 元数据记录最大编码器中断间隔，帮助判断记录本身是否明显扰动了控制。

`UINT32_MAX` 表示某个事件时刻尚未观察到；原始值和异常标志保留在文件中，不能当作有效延迟。

## 输出和分析

`validation/mode8-step-时间戳*` 包含普通遥测 CSV、驱动约 1 kHz CSV、事件 CSV、元数据、带 CRC 的原始返回文本，以及自动生成的指令/PWM 寄存器/编码器图和第一阶跃运动图。

可重新绘图：

```powershell
python tools/analyze_driver_trace.py validation/实际文件名-driver.json
```

这轮能测驱动内部处理和定时器更新时序。编码器运动开始时间还受到惯量、摩擦和分辨率影响；不能把它称为力矩延迟。若寄存器及时更新而运动仍异常，下一步用示波器确认实际逻辑 PWM/栅极驱动，仍无需采电流。

## 验证

`tests/check_driver_trace.py` 验证真实记录 C 的计时回绕、事件关联、预触发顺序、容量和 CRC；`check_trace_bridge.py` 验证真实 TC264 串口实现；`check_trace_client.py` 使用真实 C 序列化的模拟记录验证导出和绘图。模拟文件只保存在临时目录，不代表实车测试。完整回归入口为 `python tests/run_all.py`。

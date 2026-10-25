# WiFi-SPI 直立遥测

网络、SPI引脚和速率统一在 `code/config.h`。本地目标PC为192.168.0.112:8081，模块命令端口5001；WiFi名和密码保持原值。

先停车并退出IMU/开环预览，在Startup/WiFi connect选中后设On。连接失败可再次设On重试；连接后可Start或使用IMU预览。Stop停电机但保持遥测。WiFi等待期间仍处理菜单导航，硬件动作待SPI退出后执行。

数据：N个小端IEEE754 float32 + `00 00 80 7F`，N=1～40，每UDP报一帧。文本回执保留MCAR前缀。Connect/Start按模式选择34或40通道；SUB回执给出真实顺序，接收端不应猜字段偏移。

| 命令 | 作用 |
|---|---|
| LIST? | 当前可用直立变量 |
| GET? | 通道/周期/STREAM |
| SUB roll_deg,pitch_deg,yaw_deg | 替换通道列表 |
| RATE 20 | 2～1000ms发送间隔 |
| STREAM 0/1 | 暂停/恢复遥测，不是停车 |
| [slider,rate_kp,30] | 写行进轮内环KP（固定绑定） |
| [slider,fly_rate_kp,35] | 写飞轮内环KP |
| [slider,ground_angle_kp,40] | 写行进轮外环KP |
| [slider,balance_angle_x,3.9] | 写行进轮目标角度，范围±20° |

其他可写名称见 `wifispi.c/g_slider_parameters`。非法、非有限或越界输入拒绝；方向只能通过有效值±1运行。下发修改RAM，停车后Storage/Save才保存。未增加远程启停、模式切换或远程Flash功能。旧cascade_* PID接口已删除。

上位机实际UI是否暴露某个可写参数取决于其版本；固件支持字段不等于旧上位机已增加相应输入框。协议回环测试不代表无线实车已连通。

# 树形菜单

菜单节点结构、静态节点池、父子关系和循环兄弟链表移植自 `C:\Users\csz18\Desktop\mcar-mailun\project\code\menu.c/.h`。按键分支顺序及 Key_* 控制实现采用该项目 `Mymenu.c`，业务回调适配 Qcar，每页显示 7 项。

2026-10-21 按用户指定重新映射四个按键；导航、编辑和步进逻辑保持原方案，当前键位以下表为准。

## 操作

2026-10-23 修复联网等待期间的菜单响应：自动联网／重连等待时继续轮询四键，可导航、选择和编辑参数。Start、Stop、IMU、预览及保存等动作会先取消当前网络等待，再由主循环执行；不在 SPI 通信回调中初始化设备或写 Flash。K4 同时取消连接等待并执行原来的取消选中／返回操作。网络调用的按键事件在执行动作前消费，防止回调重复处理同一次按下。

- KEY_1（P20.6）：未选中时上移；选中时增大数据或将布尔值设为 On。
- KEY_2（P20.7）：未选中时下移；选中时减小数据或将布尔值设为 Of。
- KEY_3（P11.2）：进入文件夹或选中参数；已选中时减小步进并循环。
- KEY_4（P11.3）：取消选中；未选中时返回上级，根目录保持不变。
- 步进循环：`1 → 0.1 → 0.01 → 100 → 10 → 1`，初始为 1。整数类型按原方案进行类型转换，步进小于 1 时加减量为 0。
- 同时产生事件时，优先级为 KEY_1、KEY_2、KEY_3、KEY_4；每轮只消费实际处理的短按，其他按键排到下一轮，避免丢键。
- 按键通过 1 ms PIT 扫描。按下和释放分别稳定 10 ms 后确认；确认按下即触发一次菜单动作，无需等到松手。连续按住不会重复导航。
- 每键独立保存待处理事件，直到主循环消费；屏幕刷新或通信阻塞期间也不会被下一次扫描自动清除。累计容量为每键每类 255 次，达到上限后忽略新增事件。
- 长按事件独立记录一次，菜单消费但不额外执行导航。事件计数的消费使用短临界区，不会因扫描中断并发而误删新事件。

运行期间仍按同一套规则导航。停车请进入 Startup → Stop，按 KEY_3 选中后按 KEY_1 置 On；也可在 Start 节点按减键置 Of。KEY_4 按原方案只负责取消编辑／返回，不再额外触发停车。参数通过 Storage → Save 显式保存；选中 Save 后按加键触发。保存和读取不在运行或预览过程中执行。

## 目录

| 文件夹 | 内容 |
| --- | --- |
| Startup | Mode、Start、Stop、Camera、IMU |
| PID | Roll/Pitch/Yaw 各自的速度、角度、角速度环；每环 Kp/Ki/Kd |
| Vehicle | 快慢速度、速度档位、平衡零点、死区、转向系数 |
| Vision | 昼夜模式、阈值、半径、灰度与方差、丢帧、距离、IOU、轮廓参数 |
| Trajectory | 距离/角度/横向偏差权重、转向阈值、中线参数 |
| Storage | Save、Load |
| WiFi | Connect：停车并关闭预览后连接／断开 UDP 遥测，失败时可再次设 On 重试 |

Startup → Mode：`0` 为定点平衡，`1` 为自动信标运行，`2` 为遥控运行。Vision → Day/night：`1` 为白天、`2` 为夜间；Vehicle → Speed mode：`1` 为快档、`2` 为慢档。后两项保留原数据的 float 类型，请使用步进 1 设置为有效枚举值。

Camera、IMU、Wheel test 均使用 bool_Box 节点启停，菜单保持可导航。中轮测试的输出通过同目录 Wheel PWM 整数节点编辑，幅值受 PWM_DUTY_MAX 限制；退出测试应将 Wheel test 置 Of 或触发 Stop。保留现有外设延迟初始化和控制实现，未烧录验证。

## 统一创建接口

所有目录与文件在 `Menu_Create()` 中显式创建，文件一律调用：

```c
Create_Menu_File_dynamic(folder, "Kp", &cascade_ang_roll.kp, float_Box);
Create_Menu_File_dynamic(folder, "Start", &start_switch, bool_Box);
```

最后一个参数为变量类型，支持 int32/uint32/int16/uint16/int8/uint8/float/bool。没有 add_pid、add_float、add_action 包装，也没有单独的 MENU_Action 节点。PID 子目录逐个调用同一创建函数。

## 文件与验证

- `code/menu_tree.c/.h`：移植后的通用菜单节点层，128 项静态池；修正重复连接循环链表时可能死循环的问题。
- `code/menu.c/.h`：Qcar 菜单定义、输入处理、显示及动作接入。
- 原菜单备份：`../.sources/Qcar-before-tree-menu.zip`。
- 主机节点测试：`python tests/check_menu_tree.py`，检查空目录、单项循环、嵌套目录、重复连接和节点池上限。
- 按键测试：`python tests/check_menu_keys.py`，直接编译实际 Key_* 和 Menu_Switch，检查源项目键位、优先级、事件清除、返回、编辑、步进循环及各类变量。

尚未连接实车验证按键、显示刷新效果和运行模式。

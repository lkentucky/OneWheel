# VS Code 配置

用 VS Code 打开 Qcar 根目录，CMake Tools 会读取 `vscode-intellisense` 预设。
若提示选择配置预设，选择 `Qcar TC264 - IntelliSense`，再执行 `CMake: Configure`。
旧的红色波浪线若未刷新，执行 `C/C++: Reset IntelliSense Database`，然后执行
`Developer: Reload Window`。

CMake 从 `.cproject` 读取头文件路径和源文件排除项，生成
`build/vscode/compile_commands.json`，供 C/C++ 扩展解析。TASKING 的扩展关键字、
内建函数和 TriCore 汇编通过 `tasking_intellisense.h` 适配，普通 C 语法诊断仍然启用。
此适配只用于代码解析，不能验证芯片指令、内存布局或定点运算行为。

本机使用 `C:/Program Files (x86)/cpeditor/mingw64/bin` 下的 CMake、Ninja 和 GCC。
更换电脑时，修改 `CMakePresets.json`、`.vscode/settings.json` 和
`.vscode/c_cpp_properties.json` 中的工具路径。

该 CMake 项目只供编辑器使用，默认 Build 只显示提示。
固件编译和下载继续使用 AURIX Development Studio / TASKING。

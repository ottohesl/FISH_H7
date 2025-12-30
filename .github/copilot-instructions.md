# Copilot / AI Agent 指南 — FISH_H7 项目

简短目标：帮助 AI 代理快速理解本仓库的架构、构建流程和关键模式，以便安全、可预测地修改固件代码。

- **项目类型**: STM32H7 MCU 固件，基于 STM32CubeMX 生成代码 + FreeRTOS (CMSIS-RTOS v2)。主要源码在 `Core/`，外设驱动由 `Drivers/` 和 `STM32H7xx_HAL_Driver/` 提供。
- **构建系统**: CMake + Ninja，使用交叉工具链 `cmake/gcc-arm-none-eabi.cmake` 和 CMakePresets。首选命令示例：
  - 配置（Debug）：`cmake --preset Debug` 或 `cmake -S . -B build/Debug -D CMAKE_BUILD_TYPE=Debug`（使用预置更简单）。
  - 构建：`cmake --build build/Debug --target FISH_H7`。
  相关文件：[CMakePresets.json](CMakePresets.json), [CMakeLists.txt](CMakeLists.txt), [cmake/gcc-arm-none-eabi.cmake](cmake/gcc-arm-none-eabi.cmake)

- **代码组织（高层）**:
  - 设备初始化与 HAL 周边由 CubeMX 生成并放在 `Core/Src`、`Core/Inc` 与 `cmake/stm32cubemx` 子目录。
  - 应用逻辑主要分布在 `Task_Link/`（任务入口），`Core/Src/*`（各模块实现），以及 `Middlewares/Third_Party/FreeRTOS`（RTOS 相关）。
  - `main.c` 初始化外设并启动 FreeRTOS，后续逻辑由线程驱动。参见 [Core/Src/main.c](Core/Src/main.c)

- **重要外设/协议约定（项目特有）**:
  - 串口约定（在 `Core/Src/usart.c`）:
    - `USART1`：SBUS（100000, 8E2）用于遥控输入。
    - `USART2`：JY901S 传感器（115200），代理文件 `Core/Src/jy901s.c` 与头 `Core/Inc/jy901s.h`。
    - `USART6`：GPS/NMEA（9600）。
  - 这些硬件句柄在 `Core/Inc/main.h` 有宏定义（例如 `huart_JY901S` 映射到 `huart2`）。参见 [Core/Inc/main.h](Core/Inc/main.h) 和 [Core/Src/usart.c](Core/Src/usart.c)

- **FreeRTOS / 任务模式**:
  - 线程与队列在 [Core/Src/freertos.c](Core/Src/freertos.c) 中创建，许多任务实现使用 `__weak`，可被用户实现覆盖（典型模式：框架提供 weak stub，应用实现替换）。
  - 关键消息队列示例：`JY901SHandle`（保存 `jy901*` 指针）、`SUBSHandle`（保存 `uint16_t`），队列长度与元素类型在 freertos.c 中定义。

- **代码修改注意事项**:
  - 不要直接修改 CubeMX 自动生成的大量模板，除非确实需要。优选在 `/* USER CODE BEGIN */` / `/* USER CODE END */` 区间插入用户代码。
  - 若需修改引脚或外设配置，应在 `FISH_H7.ioc` 中通过 CubeMX 修改并重新生成（文件位于仓库根）。参见 [FISH_H7.ioc](FISH_H7.ioc)
  - CMake 里通过 `COMMON_FLAGS` 添加了 `-u _printf_float` 以启用浮点 printf，修改链接/编译标志时请保留该意图（见 [CMakeLists.txt](CMakeLists.txt)）。

- **常见变更位置与示例**:
  - 添加新的任务：在 `Task_Link/Start_Task.c` 注册并在 `freertos.c` 的任务创建处设置正确属性。
  - 新的传感器驱动：在 `Core/Inc` 添加头文件，在 `Core/Src` 新建实现文件，并在 `CMakeLists.txt` 的可执行列表或 `cmake/stm32cubemx` 中挂载。
  - 串口协议解析：查看 `Core/Src/NMEA_ATGM336H.c`（GPS）与 `Core/Src/sbus.c`（SBUS），保持中断回调风格（在 `main.c` 的 `HAL_UART_RxCpltCallback` 有示例）。

- **测试与调试**:
  - 构建产物在 `build/` 下（Debug/Release）；可在本地用 OpenOCD/GDB 或 IDE（CLion）加载固件到板上。
  - 日志/调试：代码中常用 `printf`（通过 `huart3`/`huart_debug`），检查 `main.c` 中初始化与 `usart.c` 的波特率配置以确保一致。

- **安全与边界条件**:
  - 严格避免在 RTOS 关键路径中执行长阻塞或浮点密集操作（除非已确保线程优先级与栈足够）。
  - 修改中断或 HAL 回调时，遵循原有缓冲长度与溢出保护（例如 `main.c` 的 NMEA 接收缓冲处理）。

- **如何提 PR / 交付变更**:
  - 小改动（逻辑或注释）直接提交分支 PR；涉及外设或 ioc 改动，先在本地通过 CubeMX 再提交生成改变并在 PR 描述中注明“CubeMX 重新生成”。

请审阅此文件并指出不清楚或遗漏的点（例如希望补充的具体文件/行号示例），我会根据反馈迭代更新。

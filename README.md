# IMU UART to CAN Bridge 🌉

![Platform](https://img.shields.io/badge/Platform-Windows-blue)
![Language](https://img.shields.io/badge/Language-C%2B%2B-orange)
![Protocol](https://img.shields.io/badge/Protocol-UART%20%7C%20CAN-brightgreen)

## 📖 项目简介 (Introduction)

本项目是一个基于 Windows C++ 开发的多线程串口转发工具。主要用于通过 UART 读取 **HiPNUC 陀螺仪 (IMU)** 的姿态数据，经过严格的 **CRC16-CCITT** 校验和解析后，将 `Yaw`、`Pitch`、`Roll` 数据打包，通过另一个串口（USB-CAN 模块）转发到 CAN 总线上。

这份代码最初作为通信测试的 Demo 开发，验证了传感器数据的多线程安全读取与稳定转发。

## ✨ 核心特性 (Features)

* **多线程并发 (Multi-threading)**: 
    * **Thread A**: 负责监听 IMU 串口，独立完成协议帧的拼装、CRC 校验与数据解析。
    * **Thread B**: 负责以 100Hz 的固定频率将最新的姿态数据组装成 CAN 报文发出。
* **完整的 HiPNUC 协议解析**:
    * 自动识别 `0x5A 0xA5` 帧头。
    * 内置官方 `CRC16-CCITT` 校验算法，自动丢弃错误包，保证数据绝对可靠。
    * 精准提取 `HI91` (Tag: `0x91`) 数据域中的三轴欧拉角。
* **线程安全 (Thread-Safe)**: 使用 `std::atomic<float>` 共享姿态数据，并利用 `CRITICAL_SECTION` 确保控制台打印不乱码。

## ⚙️ 硬件连接与配置 (Hardware Setup)

运行本程序前，你需要准备以下硬件并确认串口号：

1.  **HiPNUC IMU 模块**: 默认波特率 `115200`。
2.  **USB转CAN 模块**: 默认波特率 `9600`。

### 📌 修改 COM 口配置

在使用前，必须在代码的 `main()` 函数中修改为你电脑上实际的串口号：

```cpp
// !!! 请确认并修改为你本机的 COM 口 !!!
char gyroPort[] = "\\\\.\\COM9";  // IMU 所在的串口
char canPort[]  = "\\\\.\\COM16"; // USB-CAN 所在的串口
```

*(注：当串口号大于 COM9 时，Windows 必须使用 `\\\\.\\COMx` 格式)*

## 📦 CAN 发送数据协议 (CAN TX Protocol)

CAN 线程会将浮点型的欧拉角放大 100 倍转换为 `int16_t` 发送，报文长度为 8 字节 (8 Bytes)：

| **Byte 0** | **Byte 1** | **Byte 2**  | **Byte 3**   | **Byte 4** | **Byte 5**  | **Byte 6**   | **Byte 7**  |
| ---------- | ---------- | ----------- | ------------ | ---------- | ----------- | ------------ | ----------- |
| Yaw (Low)  | Yaw (High) | Pitch (Low) | Pitch (High) | Roll (Low) | Roll (High) | Speed (0x00) | Mode (0x01) |

*数据解算示例：实际 Yaw 角度 = `(int16_t)(Byte1 << 8 | Byte0) / 100.0f`*

## 🚀 编译与运行 (Build & Run)

本项目依赖 Windows API (`<windows.h>`)，仅限 Windows 平台编译。

### 方法一：使用 g++ (MinGW) 命令行编译

打开终端，进入代码所在目录，执行以下命令：

Bash

```
g++ test_crc.cpp -o UART2CAN_Bridge.exe -std=c++11
./UART2CAN_Bridge.exe
```

### 方法二：使用 Visual Studio

1.  创建一个新的 Windows 控制台空项目 (C++)。
2.  将 `test_crc.cpp` 添加到源文件中。
3.  编译并运行即可。

## 🛠️ 调试信息 (Debug)

代码中保留了调试用的打印接口：

-   成功接收并校验 IMU 数据时，每 20 帧会打印一次欧拉角信息。
-   如遇数据不通，请首先检查串口号是否被占用，或 Tx/Rx 引脚是否接反。

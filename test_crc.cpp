#include <iostream>
#include <windows.h>
#include <string>
#include <cstdio>
#include <atomic>
#include <vector>

// ==================== 全局变量 ====================

CRITICAL_SECTION g_cs; 

struct SharedData {
    std::atomic<float> yaw;
    std::atomic<float> pitch;
    std::atomic<float> roll;
    
    SharedData() {
        yaw = 0.0f;
        pitch = 0.0f;
        roll = 0.0f;
    }
};

SharedData g_imu_data;

// ==================== 辅助函数 ====================

// 1. 新增：CRC16-CCITT 校验算法 (参考 HiPNUC 官方手册) 
void crc16_update(uint16_t* currectCrc, const uint8_t* src, uint32_t lengthInBytes) {
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j = 0; j < lengthInBytes; ++j) {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i) {
            uint32_t temp = crc << 1;
            if (crc & 0x8000) {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currectCrc = crc;
}

void SafePrint(const char* msg) {
    EnterCriticalSection(&g_cs);
    std::cout << msg << std::endl;
    LeaveCriticalSection(&g_cs);
}

// ==================== 串口类 ====================
class SerialPort {
private:
    HANDLE hSerial;
    bool connected;
    std::string portName;

public:
    SerialPort(std::string name) {
        portName = name;
        hSerial = INVALID_HANDLE_VALUE;
        connected = false;
    }

    ~SerialPort() { Close(); }

    bool Open(int baudRate) {
        hSerial = CreateFileA(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
        if (hSerial == INVALID_HANDLE_VALUE) return false;

        DCB dcb = { 0 };
        dcb.DCBlength = sizeof(dcb);
        if (!GetCommState(hSerial, &dcb)) return false;

        dcb.BaudRate = baudRate;
        dcb.ByteSize = 8;
        dcb.StopBits = ONESTOPBIT;
        dcb.Parity = NOPARITY;
        if (!SetCommState(hSerial, &dcb)) return false;

        COMMTIMEOUTS timeouts = { 0 };
        timeouts.ReadIntervalTimeout = 1; 
        timeouts.ReadTotalTimeoutConstant = 1;
        timeouts.ReadTotalTimeoutMultiplier = 1;
        SetCommTimeouts(hSerial, &timeouts);

        connected = true;
        return true;
    }

    void Close() {
        if (connected) { CloseHandle(hSerial); connected = false; }
    }

    bool IsConnected() { return connected; }

    int Read(uint8_t* buffer, int maxLen) {
        if (!connected) return 0;
        DWORD bytesRead = 0;
        ReadFile(hSerial, buffer, maxLen, &bytesRead, NULL);
        return (int)bytesRead;
    }

    bool Write(const uint8_t* data, int len) {
        if (!connected) return false;
        DWORD bytesWritten;
        return WriteFile(hSerial, data, len, &bytesWritten, NULL);
    }
};

// ==================== 线程任务逻辑 ====================

// 线程 A: 读取陀螺仪 (HiPNUC 5A A5 解析 + CRC校验)
DWORD WINAPI GyroThreadFunc(LPVOID lpParam) {
    char* portName = (char*)lpParam;
    SerialPort serial(portName);
    char logBuf[128];

    // HiPNUC 默认波特率 115200
    int baudRate = 115200;

    if (!serial.Open(baudRate)) { 
        sprintf(logBuf, "[Gyro Error] 无法打开串口: %s", portName);
        SafePrint(logBuf);
        return 1;
    }

    sprintf(logBuf, "[Gyro] 连接成功: %s (115200) - 模式: HiPNUC(含CRC校验)", portName);
    SafePrint(logBuf);

    std::vector<uint8_t> rxBuf; 
    uint8_t tmpBuf[128];
    int debug_counter = 0;

    while (serial.IsConnected()) {
        int len = serial.Read(tmpBuf, sizeof(tmpBuf));
        if (len > 0) {
            rxBuf.insert(rxBuf.end(), tmpBuf, tmpBuf + len);
        }

        // 处理缓冲区 (最小包长: Header(2)+Len(2)+CRC(2) = 6字节)
        while (rxBuf.size() >= 6) { 
            // 1. 寻找帧头 5A A5 [cite: 403]
            if (rxBuf[0] != 0x5A || rxBuf[1] != 0xA5) {
                rxBuf.erase(rxBuf.begin()); 
                continue;
            }

            // 2. 解析长度 (小端模式)
            uint16_t dataLen = rxBuf[2] | (rxBuf[3] << 8);
            uint32_t totalLen = 6 + dataLen;

            // 3. 检查数据接收完整性
            if (rxBuf.size() < totalLen) {
                break; // 等待更多数据
            }

            // 4. === CRC 校验环节 ===
            // 提取包内自带的 CRC (第4、5字节)
            uint16_t packet_crc = rxBuf[4] | (rxBuf[5] << 8);
            
            // 计算本地 CRC
            // 规则：校验(Header+Len) + 校验(Payload) 
            uint16_t calc_crc = 0;
            crc16_update(&calc_crc, rxBuf.data(), 4);        // 校验前4字节 (5A A5 Len Len)
            crc16_update(&calc_crc, rxBuf.data() + 6, dataLen); // 校验数据域 (跳过CRC本身的2字节)

            if (calc_crc != packet_crc) {
                // 校验失败：丢弃包头，重新寻找
                // sprintf(logBuf, "[CRC Fail] Calc: %04X, Recv: %04X", calc_crc, packet_crc);
                // SafePrint(logBuf);
                rxBuf.erase(rxBuf.begin()); 
                continue;
            }

            // 5. CRC 通过，解析 Payload
            uint8_t* payload = &rxBuf[6];
            
            // HI91 协议 Tag = 0x91 [cite: 409]
            if (payload[0] == 0x91) {
                // 根据手册 Offset 解析数据 [cite: 409]
                // Roll: offset 48, Pitch: offset 52, Yaw: offset 56
                if (dataLen >= 60) {
                    float f_roll, f_pitch, f_yaw;
                    memcpy(&f_roll,  &payload[48], 4);
                    memcpy(&f_pitch, &payload[52], 4);
                    memcpy(&f_yaw,   &payload[56], 4);

                    g_imu_data.roll  = f_roll;
                    g_imu_data.pitch = f_pitch;
                    g_imu_data.yaw   = f_yaw;

                    debug_counter++;
                    if (debug_counter % 20 == 0) { 
                        sprintf(logBuf, "[Rx OK] Yaw: %.2f | Pitch: %.2f | Roll: %.2f", f_yaw, f_pitch, f_roll);
                        SafePrint(logBuf);
                    }
                }
            }

            // 6. 移除处理完的帧
            rxBuf.erase(rxBuf.begin(), rxBuf.begin() + totalLen);
        }
        
        Sleep(1);
    }
    return 0;
}

// 线程 B: 发送 CAN (保持不变)
DWORD WINAPI CanThreadFunc(LPVOID lpParam) {
    char* portName = (char*)lpParam;
    SerialPort serial(portName);
    char logBuf[128];

    // USB-CAN 波特率通常为 9600
    if (!serial.Open(9600)) { 
        sprintf(logBuf, "[CAN Error] 无法打开串口: %s", portName);
        SafePrint(logBuf);
        return 1;
    }

    sprintf(logBuf, "[CAN] 连接成功: %s (发送中...)", portName);
    SafePrint(logBuf);

    int debug_counter = 0;

    while (serial.IsConnected()) {
        float yaw = g_imu_data.yaw;
        float pitch = g_imu_data.pitch;
        float roll = g_imu_data.roll;

        uint8_t send_buf[8];
        int16_t s_yaw = (int16_t)(yaw * 100.0f);
        memcpy(&send_buf[0], &s_yaw, 2);
        int16_t s_pitch = (int16_t)(pitch * 100.0f);
        memcpy(&send_buf[2], &s_pitch, 2);
        int16_t s_roll = (int16_t)(roll * 100.0f);
        memcpy(&send_buf[4], &s_roll, 2);

        send_buf[6] = 0; // Speed
        send_buf[7] = 0x01; // Mode=1

        if (serial.Write(send_buf, 8)) {
            debug_counter++;
            if (debug_counter % 20 == 0) {
                // 打印发送的数据用于调试
                // sprintf(logBuf, " -> [CAN Sent] Yaw: %d (%.2f)", s_yaw, yaw);
                // SafePrint(logBuf);
            }
        }

        Sleep(10); // 100Hz
    }
    return 0;
}

// ==================== 主函数 ====================

int main() {
    InitializeCriticalSection(&g_cs);

    // !!! 请确认 COM 口 !!!
    char gyroPort[] = "\\\\.\\COM9";  
    char canPort[] = "\\\\.\\COM16";   

    SafePrint("=== 系统启动 (HiPNUC CRC Check Enabled) ===");

    HANDLE hGyro = CreateThread(NULL, 0, GyroThreadFunc, gyroPort, 0, NULL);
    HANDLE hCan = CreateThread(NULL, 0, CanThreadFunc, canPort, 0, NULL);

    WaitForSingleObject(hGyro, INFINITE);
    WaitForSingleObject(hCan, INFINITE);

    DeleteCriticalSection(&g_cs);
    CloseHandle(hGyro);
    CloseHandle(hCan);

    return 0;
}
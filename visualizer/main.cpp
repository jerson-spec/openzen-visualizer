/**
 * @file main.cpp
 * @brief OpenZen 传感器可视化工具 —— 程序入口
 *
 * ============================================================
 * 程序功能
 * ============================================================
 *
 * 本程序通过蓝牙连接 LPMS 系列 IMU 传感器，实时显示以下数据：
 *   1. 加速度计、陀螺仪、磁力计原始数据（时序图）
 *   2. 传感器固件内置姿态（欧拉角 + 3D 立方体）
 *   3. Madgwick 滤波器融合姿态（独立于固件的软件解算结果）
 *   4. 两种姿态的逐通道对比（Roll/Pitch/Yaw）
 *   5. 线性加速度（减去重力后的纯运动加速度）
 *   6. 温度 & 气压环境数据
 *
 * 技术栈：
 *   - OpenZen SDK:  传感器通信抽象层
 *   - GLFW:         窗口管理 & OpenGL 上下文
 *   - Dear ImGui:   即时模式 GUI 框架
 *   - ImPlot:       ImGui 的绘图扩展
 *   - Madgwick AHRS: 9轴姿态融合算法
 *
 * ============================================================
 * 程序架构（双线程模型）
 * ============================================================
 *
 *   main() - 主线程 (~60fps 渲染循环)
 *     ├── GLFW + ImGui 初始化
 *     ├── 创建 DataBuffer (容量 10000)
 *     ├── 创建 SensorThread(DataBuffer)
 *     │     └── 启动子线程 → run() @ ~100Hz
 *     │           ├── 连接蓝牙传感器 (OpenZen)
 *     │           ├── 读取 IMU 数据
 *     │           ├── 运行 Madgwick 滤波器
 *     │           └── 推入 DataBuffer
 *     ├── 创建 GuiRenderer(DataBuffer)
 *     └── 主循环:
 *           ├── glfwPollEvents()
 *           ├── renderer.render()  ← 从 DataBuffer 取数据
 *           ├── ImGui::Render()
 *           └── glfwSwapBuffers()
 */

#define OPENZEN_CXX17
#include "OpenZen.h"
#include "DataBuffer.h"
#include "SensorThread.h"
#include "GuiRenderer.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include <GLFW/glfw3.h>

#include <iostream>
#include <string>
#include <vector>

// === GLFW 回调函数 ===

/// GLFW 错误回调：将错误信息输出到 stderr
static void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

/// 窗口大小变化回调：更新 OpenGL 视口
static void glfw_framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

/**
 * @brief 扫描蓝牙传感器
 *
 * OpenZen SDK 支持的蓝牙连接类型：
 *   - "Bluetooth": Bluetooth Classic (SPP 协议)
 *   - "Ble":       Bluetooth Low Energy
 *
 * 扫描流程：
 *   1. 创建 OpenZen 客户端
 *   2. 发送异步传感器扫描请求 (listSensorsAsync)
 *   3. 轮询事件（最多 300 个时间片，约 3 秒超时）
 *   4. 获取 SensorFound 事件，检查 IO 类型是否为 Bluetooth/BLE
 *   5. 找到后立即返回，停止扫描
 *
 * @param outIoType     输出：IO 类型 ("Bluetooth" 或 "Ble")
 * @param outSensorName 输出：传感器标识符（用于后续 obtainSensorByName）
 * @return true=找到蓝牙传感器, false=扫描超时或未找到
 */
bool findBluetoothSensor(std::string& outIoType, std::string& outSensorName) {
    ZenSetLogLevel(ZenLogLevel_Info);

    // 创建客户端
    auto [clientError, client] = zen::make_client();
    if (clientError) {
        std::cerr << "Cannot create OpenZen client" << std::endl;
        return false;
    }

    // 启动异步传感器扫描（非阻塞）
    if (auto error = client.listSensorsAsync(); error != ZenError_None) {
        std::cerr << "Cannot start sensor listing" << std::endl;
        client.close();
        return false;
    }

    bool listingComplete = false;
    int timeout = 300;      // 超时循环计数（每次 ~10ms，总计 ~3s）
    int foundCount = 0;     // 找到的传感器总数（所有类型）

    // 收集完整扫描结果再选：双模传感器会同时以 Classic 和 BLE 两种身份被发现，
    // 经典蓝牙 (SPP) 带宽远高于 BLE（BLE 实测常常只有 30-50 Hz）。
    // 之前"谁先出现连谁"导致每次启动随机连到其中一种，数据率忽高忽低。
    struct FoundSensor {
        std::string ioType, name, identifier;
    };
    std::vector<FoundSensor> btSensors;

    std::cout << "Scanning for Bluetooth sensors (Classic and BLE)... (timeout: " << timeout << ")" << std::endl;

    while (!listingComplete && timeout > 0) {
        auto event = client.waitForNextEvent();
        if (!event) {
            timeout--;
            continue;
        }

        switch (event->eventType) {
            case ZenEventType_SensorFound: {
                // 发现一个传感器设备
                const auto& desc = event->data.sensorFound;
                foundCount++;
                std::string ioTypeStr(desc.ioType);
                std::cout << "Found sensor #" << foundCount << ": IO=" << ioTypeStr << ", Name=" << desc.name << ", Identifier=" << desc.identifier << std::endl;

                // 收集蓝牙类传感器，扫描结束后统一择优
                if (ioTypeStr == "Bluetooth" || ioTypeStr == "Ble") {
                    btSensors.push_back({ ioTypeStr, std::string(desc.name),
                                          std::string(desc.identifier) });
                }
                break;
            }
            case ZenEventType_SensorListingProgress: {
                // 扫描进度更新
                auto& progress = event->data.sensorListingProgress;
                listingComplete = progress.complete != 0;
                std::cout << "Listing progress: " << (int)(progress.progress * 100) << "%, complete=" << (int)progress.complete << std::endl;
                break;
            }
            default:
                break;
        }
        timeout--;
    }

    client.close();
    std::cout << "Scan completed. Total sensors found: " << foundCount << std::endl;

    // 优先经典蓝牙（高带宽），其次 BLE；扫描超时也已收集到传感器，照样可用
    for (const char* pref : { "Bluetooth", "Ble" }) {
        for (const auto& s : btSensors) {
            if (s.ioType == pref) {
                std::cout << "Selected " << s.ioType << " sensor: " << s.name
                          << " (identifier " << s.identifier << ")" << std::endl;
                outIoType = s.ioType;
                outSensorName = s.identifier;
                return true;
            }
        }
    }
    return false;
}

int main(int argc, char** argv) {
    std::string ioType = "Bluetooth";
    std::string sensorName;

    // ===== 第 1 步：扫描蓝牙传感器 =====
    std::cout << "Searching for Bluetooth sensor..." << std::endl;

    if (!findBluetoothSensor(ioType, sensorName)) {
        std::cerr << "No Bluetooth sensor found. Please check your sensor connection and try again." << std::endl;
        return 1;
    }

    std::cout << "Connecting to Bluetooth sensor: " << sensorName << std::endl;

    // ===== 第 2 步：初始化 GLFW =====
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return 1;
    }

    // OpenGL 3.2 Core Profile（Intel/AMD/NVIDIA 广泛的兼容版本）
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    // 创建窗口 1400×900
    GLFWwindow* window = glfwCreateWindow(1400, 900, "OpenZen Sensor Visualizer", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // 启用 VSync（帧率与显示器刷新率同步）
    glfwSetFramebufferSizeCallback(window, glfw_framebuffer_size_callback);

    // ===== 第 3 步：初始化 ImGui + ImPlot =====
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // 键盘导航

    ImGui::StyleColorsDark();

    // Load real font at 22px (Microsoft YaHei; crisp glyph rendering, no blur
    // from bitmap-font scaling). Falls back to Segoe UI, then default 13px.
    {
        // 用原始字符串字面量：普通字面量里 \W \F \m \s 都是无法识别的转义序列，
        // MSVC 会把反斜杠直接吃掉，路径变成 "C:WindowsFontsmsyh.ttc"，
        // fopen_s 必然失败，最后静默退回到 13px 内置点阵字体。
        const char* candidates[] = {
            R"(C:\Windows\Fonts\msyh.ttc)",
            R"(C:\Windows\Fonts\simhei.ttf)",
            R"(C:\Windows\Fonts\segoeui.ttf)",
        };
        bool loaded = false;
        for (const char* path : candidates) {
            FILE* fp = nullptr;
            fopen_s(&fp, path, "rb");
            if (!fp) continue;
            fclose(fp);
            if (io.Fonts->AddFontFromFileTTF(path, 22.0f)) { loaded = true; break; }
        }
        if (!loaded)
            io.Fonts->AddFontDefault();  // fallback to built-in 13px
    }  // 深色主题

    // 绑定 GLFW + OpenGL3 后端
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // ===== 第 4 步：创建核心对象 =====
    // DataBuffer:    线程安全的环形缓冲（容量 10000，≈100 秒 @100Hz）
    // SensorThread:  传感器采集线程（生产者）
    // GuiRenderer:   可视化渲染（消费者）
    DataBuffer buffer(10000);
    SensorThread sensorThread(buffer);
    GuiRenderer renderer(buffer, sensorThread);

    // 启动传感器线程（baudrate=0 使用默认值）
    sensorThread.start(ioType, sensorName, 0);

    // ===== 第 5 步：主渲染循环 =====
    bool connected = false;
    bool showWindow = true;

    while (!glfwWindowShouldClose(window) && showWindow) {
        glfwPollEvents();  // 处理窗口事件（关闭、拖拽等）

        // 开始新的一帧 ImGui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 检测连接状态变化
        bool currentlyConnected = sensorThread.isConnected();
        if (currentlyConnected != connected) {
            connected = currentlyConnected;
            if (connected) {
                std::cout << "Bluetooth sensor connected successfully!" << std::endl;
            } else {
                std::cout << "Bluetooth sensor disconnected: " << sensorThread.lastError() << std::endl;
            }
        }

        // 渲染所有可视化窗口
        renderer.render(&showWindow);

        // 提交渲染指令
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);  // 深色背景
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);  // 交换前后缓冲区
    }

    // ===== 第 6 步：清理资源 =====
    sensorThread.stop();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

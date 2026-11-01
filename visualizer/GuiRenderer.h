#pragma once

#include "DataBuffer.h"
#include "MagEllipsoidCalib.h"
#include "MagFieldMap.h"
#include "imgui.h"
#include "implot.h"
#include <vector>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <atomic>
#include <mutex>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <functional>
#include <algorithm>

struct GuiState {
    //  " vs "
    //    +  Control Panel 
    bool showAccelWindow = false;
    bool showGyroWindow = false;
    bool showMagWindow = false;
    bool showEulerWindow = false;
    bool showLinAccWindow = false;
    bool showEnvWindow = false;
    bool showFusedEulerWindow = false;
    bool showFusedLinAccWindow = false;
    bool showEulerCompareWindow = true;       //  vs Madgwick
    bool showSensor3dWindow = false;
    bool showFused3dWindow = false;
    bool showCompEulerWindow = false;
    bool showCompLinAccWindow = false;
    bool showComp3dWindow = false;
    bool showEulerAllCompareWindow = false;
    bool showEkfEulerWindow = false;          // EKF 
    bool showEkfCompareWindow = true;         //  vs 

    // 磁椭球校准可视化
    bool showMagCalibWindow = true;
    bool magSpin = true;                      ///< 3D 视图自动旋转
    float magAzim = 35.0f;                    ///< 3D 视图方位角 (deg)
    float magElev = 25.0f;                    ///< 3D 视图俯仰角 (deg)
    float magYawGap = 40.0f;                  ///< 观测到的 host 与固件 yaw 差值 (deg)，用于反推所需残余硬铁

    // Field Survey（桌面网格扫描读数工具）
    bool showSurveyWindow = true;             ///< 网格读数窗口，默认随程序打开

    // Field Map（室内干扰场图）
    bool showFieldMapWindow = true;            ///< 场图窗口，默认随程序打开

    // Control Panel 数据区折叠（一键收起/展开）
    bool secRaw = true, secFused = true, secComp = true, secEkf = true, secQuat = true;

    int historyLength = 250;                  //  500  250ImPlot  2.5s@100Hz 
    float plotHeight = 150.0f;

    bool isRecording = false;
    std::string recordFilePath;
    uint64_t recordCount = 0;
};

class SensorThread;

class GuiRenderer {
public:
    GuiRenderer(DataBuffer& buffer, SensorThread& sensorThread)
        : m_buffer(buffer)
        , m_sensorThread(sensorThread)
        , m_samplingRate(100)
    {}

    ~GuiRenderer() {}

    void render(bool* pOpen) {
        renderAccelWindow();
        renderGyroWindow();
        renderMagWindow();
        renderEulerWindow();
        renderLinAccWindow();
        renderEnvWindow();
        renderSensor3dWindow();
        renderEkfCompareWindow();
        renderMagCalibWindow();
        renderSurveyWindow();
        renderFieldMapWindow();
        renderMainWindow(pOpen);
    }

private:
    void renderMainWindow(bool* pOpen) {
        ImGui::SetNextWindowSize(ImVec2(470, 700), ImGuiCond_FirstUseEver);
        ImGui::Begin("OpenZen Control Panel", pOpen);
        // font size comes from the 18px real font loaded in main.cpp (no per-window scaling)
        ImGui::SetWindowFontScale(1.0f);

        ImGui::Text("Window Visibility");
        ImGui::Separator();

        ImGui::Checkbox("Acceleration", &m_state.showAccelWindow);
        ImGui::Checkbox("Gyroscope", &m_state.showGyroWindow);
        ImGui::Checkbox("Magnetometer", &m_state.showMagWindow);
        ImGui::Checkbox("Euler Angles", &m_state.showEulerWindow);
        ImGui::Checkbox("Linear Acceleration", &m_state.showLinAccWindow);
        ImGui::Checkbox("Environment", &m_state.showEnvWindow);
        ImGui::Checkbox("Sensor 3D", &m_state.showSensor3dWindow);
        ImGui::Checkbox("Euler Compare", &m_state.showEkfCompareWindow);
        ImGui::Checkbox("Mag Calibration", &m_state.showMagCalibWindow);
        ImGui::Checkbox("Field Survey", &m_state.showSurveyWindow);
        ImGui::Checkbox("Field Map", &m_state.showFieldMapWindow);

        ImGui::Separator();
        ImGui::Text("Plot Settings");
        ImGui::Separator();

        ImGui::SliderInt("History Length", &m_state.historyLength, 100, 2000);
        ImGui::SliderFloat("Plot Height", &m_state.plotHeight, 80.0f, 800.0f);

        // 五个数据块可独立折叠，也可一键全部收起/展开：
        // 既减少视觉噪音，也省掉这些文本行的逐帧提交
        const bool anyDataOpen = m_state.secRaw || m_state.secFused || m_state.secComp
                              || m_state.secEkf || m_state.secQuat;
        if (ImGui::Button(anyDataOpen ? "Collapse all data sections"
                                      : "Expand all data sections")) {
            m_state.secRaw = m_state.secFused = m_state.secComp
                = m_state.secEkf = m_state.secQuat = !anyDataOpen;
        }

        std::vector<SensorData> data;
        m_buffer.fillLatest(data, 1);
        if (!data.empty()) {
            const SensorData& d = data[0];
            if (ImGui::CollapsingHeader("Current Sensor Data (Raw)", &m_state.secRaw)) {
                ImGui::Text("Accel: X=%.2f Y=%.2f Z=%.2f g", d.acc[0], d.acc[1], d.acc[2]);
                ImGui::Text("Gyro: X=%.1f Y=%.1f Z=%.1f deg/s", d.gyro[0], d.gyro[1], d.gyro[2]);
                ImGui::Text("MagB: X=%.1f Y=%.1f Z=%.1f uT", d.magB[0], d.magB[1], d.magB[2]);
                ImGui::Text("Euler: R=%.1f P=%.1f Y=%.1f deg", d.euler[0], d.euler[1], d.euler[2]);
                ImGui::Text("LinAcc: X=%.2f Y=%.2f Z=%.2f g", d.linAcc[0], d.linAcc[1], d.linAcc[2]);
                ImGui::Text("Temp: %.1f C | Pressure: %.1f mPa", d.temperature, d.pressure);
            }
            if (ImGui::CollapsingHeader("Fused Data (Madgwick)", &m_state.secFused)) {
                ImGui::Text("Fused Euler: R=%.1f P=%.1f Y=%.1f deg", d.fusedEuler[0], d.fusedEuler[1], d.fusedEuler[2]);
                ImGui::Text("Fused LinAcc: X=%.2f Y=%.2f Z=%.2f g", d.fusedLinAcc[0], d.fusedLinAcc[1], d.fusedLinAcc[2]);
                ImGui::Text("Fused Quat: W=%.4f X=%.4f Y=%.4f Z=%.4f", d.fusedQuat[0], d.fusedQuat[1], d.fusedQuat[2], d.fusedQuat[3]);
            }
            if (ImGui::CollapsingHeader("Complementary Filter", &m_state.secComp)) {
                ImGui::Text("Comp Euler: R=%.1f P=%.1f Y=%.1f deg", d.compEuler[0], d.compEuler[1], d.compEuler[2]);
                ImGui::Text("Comp LinAcc: X=%.2f Y=%.2f Z=%.2f g", d.compLinAcc[0], d.compLinAcc[1], d.compLinAcc[2]);
                ImGui::Text("Comp Quat: W=%.4f X=%.4f Y=%.4f Z=%.4f", d.compQuat[0], d.compQuat[1], d.compQuat[2], d.compQuat[3]);
            }
            if (ImGui::CollapsingHeader("EKF Filter", &m_state.secEkf)) {
                ImGui::Text("EKF Euler: R=%.1f P=%.1f Y=%.1f deg", d.ekfEuler[0], d.ekfEuler[1], d.ekfEuler[2]);
                ImGui::Text("EKF LinAcc: X=%.2f Y=%.2f Z=%.2f g", d.ekfLinAcc[0], d.ekfLinAcc[1], d.ekfLinAcc[2]);
                ImGui::Text("EKF Quat: W=%.4f X=%.4f Y=%.4f Z=%.4f", d.ekfQuat[0], d.ekfQuat[1], d.ekfQuat[2], d.ekfQuat[3]);
            }
            if (ImGui::CollapsingHeader("Sensor Quat / Buffer", &m_state.secQuat)) {
                ImGui::Text("Sensor Quat: W=%.4f X=%.4f Y=%.4f Z=%.4f", d.quat[0], d.quat[1], d.quat[2], d.quat[3]);
                ImGui::Text("Buffer: %zu samples", m_buffer.size());
            }
        } else {
            ImGui::Text("Waiting for sensor data...");
        }

        //   
        // Event Rate: 
        // Gyro Integral:  dt
        //       
        //     /  dt/
        const double effRate = m_sensorThread.effectiveDataRate();
        if (effRate > 0.0) {
            ImGui::Text("Event Rate: %.1f Hz (target %d Hz)", effRate, m_samplingRate);
        }
        // 区分两类"卡"：fps 低是渲染卡，Event Rate 掉是数据流断（蓝牙丢包/掉线）
        ImGui::Text("GUI Framerate: %.0f fps", ImGui::GetIO().Framerate);

        // 传输方式 + 最近 5 秒实测数据率（PC 墙钟整窗计数）。
        // Event Rate 是到达间隔的 EMA，蓝牙突发投递时读数来回摆；
        // 传感器时间戳是 frameCount 的换算值、单位不可靠，所以这里用墙钟
        {
            ImGui::Text("Link: %s | Data rate (5s): %.1f Hz",
                        m_sensorThread.linkIoType().c_str(),
                        m_sensorThread.linkRate5s());
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Bluetooth = Classic SPP (high bandwidth), Ble = BLE (often 30-50 Hz ceiling).\n"
                    "Data rate = samples per wall-clock second over the last 5 s window.\n"
                    "Far below 100 Hz means the wireless link, not the GUI, is the bottleneck.");
            }
        }
        const float* gi = m_sensorThread.gyroIntegral();
        ImGui::Text("Gyro Integral: X=%.1f Y=%.1f Z=%.1f deg", gi[0], gi[1], gi[2]);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Pure gyro integration, no fusion correction.\nRotate ~90 deg slowly about one axis:\nthat channel should change ~90 deg.\nIf OK but filters are wrong -> mag/accel correction issue.\nIf also wrong -> dt or gyro data issue.");
        }

        //   +  
        // Mag Heading: 
        //    90 90|B| 
        //    Mag Calibration
        if (!data.empty()) {
            const SensorData& d = data[0];
            const float accN = sqrtf(d.acc[0] * d.acc[0] + d.acc[1] * d.acc[1] + d.acc[2] * d.acc[2]);
            if (accN > 0.0f) {
                // LPMS  Z-1g
                const float axn = -d.acc[0] / accN, ayn = -d.acc[1] / accN, azn = -d.acc[2] / accN;
                const float roll = atan2f(ayn, azn);
                const float pitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));
                const float cr = cosf(roll), sr = sinf(roll);
                const float cp = cosf(pitch), sp = sinf(pitch);
                const float mxh = d.mag[0] * cp + d.mag[1] * sr * sp + d.mag[2] * cr * sp;
                const float myh = d.mag[1] * cr - d.mag[2] * sr;
                const float magNorm = sqrtf(d.mag[0] * d.mag[0] + d.mag[1] * d.mag[1] + d.mag[2] * d.mag[2]);
                const float magHeading = atan2f(-myh, mxh) * 180.0f / 3.14159265358979323846f;
                // d.mag 是单位球向量（|·|≈1），乘上拟合场强才是 µT；
                // 直接显示模长会得到 ~1.0 的无量纲值却标着 uT
                MagEllipsoidCalib::Result fitNow;
                m_sensorThread.magFitSnapshot(fitNow);
                ImGui::Text("Mag Heading: %.1f deg | |B|: %.1f uT", magHeading, magNorm * fitNow.fieldMag);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Tilt-compensated magnetic heading.\nRotate 90 deg slowly when level:\nheading should change ~90 deg and |B| stay ~constant.\nIf not -> run Mag Calibration below.");
                }

                ImGui::Separator();
                if (!m_sensorThread.isMagCalibrating()) {
                    if (ImGui::Button("Start Mag Calibration")) {
                        m_sensorThread.startMagCalibration();
                    }
                    if (!m_sensorThread.magCalibrationValid() && m_sensorThread.magCalibFail() > 0) {
                        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "Calib FAILED");
                        switch (m_sensorThread.magCalibFail()) {
                        case 1: ImGui::Text("Too few samples (<300)."); break;
                        case 6: {
                            float cx, cy, cz, cov, rms;
                            m_sensorThread.magCalibration(cx, cy, cz, cov, rms);
                            ImGui::Text("Quality gate: coverage %.0f%% (need 15%%+),",
                                cov * 100.0f);
                            ImGui::Text("rms %.3f (need 0.25 or less).", rms);
                            break;
                        }
                        default: ImGui::Text("Numerical failure (step %d).", m_sensorThread.magCalibFail()); break;
                        }
                        ImGui::Text("Tips: tumble slower but cover every face,");
                        ImGui::Text("stay away from PC/metal. See");
                        ImGui::Text("mag_calib_samples.txt for analysis.");
                    }
                    if (m_sensorThread.magCalibrationValid()) {
                        float cx, cy, cz, cov, rms;
                        m_sensorThread.magCalibration(cx, cy, cz, cov, rms);
                        ImGui::Text("Calib: center(%.1f, %.1f, %.1f) uT", cx, cy, cz);
                        ImGui::Text("   coverage %.0f%%, rms %.3f", cov * 100.0f, rms);
                        if (cov < 0.4f) {
                            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                                "Low coverage - redo calibration");
                        }
                    }
                } else {
                    const int nS = m_sensorThread.magCalibSamples();
                    const float liveCov = (float)m_sensorThread.magCalibLiveCoverage();
                    ImGui::TextColored(ImVec4(1.0f, 0.86f, 0.3f, 1.0f), "Calibrating... %d samples",
                        nS);
                    ImGui::Text("3D coverage: %.0f%% (need 15%%+)", liveCov * 100.0f);
                    if (liveCov < 0.15f && nS > 100) {
                        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                            "Tumble to ALL orientations!");
                    }
                    ImGui::Text("Slowly tumble the sensor through");
                    ImGui::Text("ALL orientations (every face up,");
                    ImGui::Text("every edge down, figure-8...),");
                    ImGui::Text("away from iron & magnets.");
                    if (ImGui::Button("Stop & Apply")) {
                        m_sensorThread.stopMagCalibration();
                    }
                }
            }
        }

        ImGui::Separator();
        ImGui::Text("Data Recording");
        ImGui::Separator();

        if (!m_sensorThread.isRecording()) {
            if (ImGui::Button("Start Recording")) {
                if (startRecording()) {
                    ImGui::OpenPopup("Recording Started");
                }
            }
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(255, 100, 100, 255));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(255, 150, 150, 255));
            if (ImGui::Button("Stop Recording")) {
                stopRecording();
                ImGui::OpenPopup("Recording Stopped");
            }
            ImGui::PopStyleColor(2);
        }

        if (!m_state.recordFilePath.empty()) {
            ImGui::Text("File: %s", m_state.recordFilePath.c_str());
        }
        ImGui::Text("Records: %llu", m_state.recordCount);

        if (ImGui::BeginPopup("Recording Started")) {
            ImGui::Text("Recording started!");
            ImGui::Text("File: %s", m_state.recordFilePath.c_str());
            ImGui::Separator();
            if (ImGui::Button("OK")) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        if (ImGui::BeginPopup("Recording Stopped")) {
            ImGui::Text("Recording stopped!");
            ImGui::Text("File: %s", m_state.recordFilePath.c_str());
            ImGui::Separator();
            ImGui::Text("Target Rate: %d Hz", m_samplingRate);
            ImGui::Separator();
            ImGui::Text("PC Duration: %.4f sec", m_pcDuration);
            ImGui::Text("Sensor Span: %.4f sec", m_sensorDuration);
            ImGui::Separator();
            ImGui::Text("Expected (PC): %llu", m_expectedPcCount);
            ImGui::Text("Expected (Sensor): %llu", m_expectedSensorCount);
            ImGui::Text("Actual: %llu", m_actualCount);
            ImGui::Separator();
            if (m_expectedSensorCount > 0) {
                ImGui::Text("Deviation: %.2f %%", m_diffRatio);
            }
            ImGui::Separator();
            if (ImGui::Button("OK")) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        ImGui::End();
    }

    void renderAccelWindow() {
        if (!m_state.showAccelWindow) return;
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Acceleration", &m_state.showAccelWindow);
        renderThreeAxisPlot("Acceleration",
            [](const SensorData& d) { return d.acc[0]; },
            [](const SensorData& d) { return d.acc[1]; },
            [](const SensorData& d) { return d.acc[2]; },
            "X (g)", "Y (g)", "Z (g)",
            IM_COL32(255, 0, 0, 255), IM_COL32(0, 255, 0, 255), IM_COL32(0, 0, 255, 255));
        ImGui::End();
    }

    void renderGyroWindow() {
        if (!m_state.showGyroWindow) return;
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Gyroscope", &m_state.showGyroWindow);
        renderThreeAxisPlot("Gyroscope",
            [](const SensorData& d) { return d.gyro[0]; },
            [](const SensorData& d) { return d.gyro[1]; },
            [](const SensorData& d) { return d.gyro[2]; },
            "X (deg/s)", "Y (deg/s)", "Z (deg/s)",
            IM_COL32(255, 128, 0, 255), IM_COL32(128, 255, 0, 255), IM_COL32(0, 128, 255, 255));
        ImGui::End();
    }

    void renderMagWindow() {
        if (!m_state.showMagWindow) return;
        ImGui::SetNextWindowSize(ImVec2(600, 460), ImGuiCond_FirstUseEver);
        ImGui::Begin("Magnetometer", &m_state.showMagWindow);
        // d.mag 是椭球校准后映射到单位球面的向量（量级 ±1），不是 µT；
        // 这里画 d.magB —— 固件输出的真实磁感应强度通道。
        renderThreeAxisPlot("Magnetometer",
            [](const SensorData& d) { return d.magB[0]; },
            [](const SensorData& d) { return d.magB[1]; },
            [](const SensorData& d) { return d.magB[2]; },
            "X (uT)", "Y (uT)", "Z (uT)",
            IM_COL32(255, 64, 64, 255), IM_COL32(64, 255, 64, 255), IM_COL32(64, 64, 255, 255));
        renderMagNormPlot();
        ImGui::End();
    }

    /// |B| 曲线：校准前应当随姿态大幅波动，校准后应当压平。
    /// 两条线一起画，是判断磁校准有没有起作用的最快方法。
    void renderMagNormPlot() {
        std::vector<SensorData> data;
        m_buffer.fillLatest(data, m_state.historyLength);
        if (data.empty()) return;

        std::vector<double> xs(data.size()), raw(data.size()), cal(data.size());
        MagEllipsoidCalib::Result fit;
        m_sensorThread.magFitSnapshot(fit);
        for (size_t i = 0; i < data.size(); i++) {
            const SensorData& d = data[i];
            xs[i] = d.timestamp;
            raw[i] = std::sqrt((double)d.magB[0] * d.magB[0]
                             + (double)d.magB[1] * d.magB[1]
                             + (double)d.magB[2] * d.magB[2]);
            // 单位球面向量还原回 µT，才能和 raw 同轴比较
            const double n = std::sqrt((double)d.mag[0] * d.mag[0]
                                     + (double)d.mag[1] * d.mag[1]
                                     + (double)d.mag[2] * d.mag[2]);
            cal[i] = n * fit.fieldMag;
        }

        if (ImPlot::BeginPlot("|B| before / after calibration", ImVec2(-1, m_state.plotHeight))) {
            ImPlot::SetupAxes("Time (s)", "uT");
            ImPlot::SetupAxisLimits(ImAxis_X1, xs.front(), xs.back(), ImGuiCond_Always);
            std::vector<double> allY = raw;
            allY.insert(allY.end(), cal.begin(), cal.end());
            double yMin, yMax;
            computeAutoRange(allY, yMin, yMax);
            ImPlot::SetupAxisLimits(ImAxis_Y1, yMin, yMax, ImGuiCond_Always);
            ImPlot::SetupAxisFormat(ImAxis_Y1, "%.1f");
            ImPlot::PlotLine("raw |b| (uT)", xs.data(), raw.data(), (int)data.size());
            ImPlot::PlotLine("calibrated |m|*|B| (uT)", xs.data(), cal.data(), (int)data.size());
            ImPlot::EndPlot();
        }
    }

    /// Field Survey 窗口：桌面网格扫描的"稳定读数 + 一键记录"工具。
    ///
    /// 椭球校准后的 Mag Heading / |B| 在室内随位置变化（固定磁物体的干扰场），
    /// 逐格点记录这两个量就能把干扰的空间分布量出来。Control Panel 的
    /// Mag Heading 是单样本瞬时值，抖动太快没法抄；这里改为对最近 N 秒内
    /// "静止"样本（陀螺小、加速度近 1g）做统计：航向取圆周平均避免 ±180°
    /// 跳变，|B| 取算术平均，稳定后一键 Mark 落盘 field_survey.txt。
    void renderSurveyWindow() {
        if (!m_state.showSurveyWindow) return;
        ImGui::SetNextWindowSize(ImVec2(430, 560), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Field Survey", &m_state.showSurveyWindow)) {
            ImGui::End();
            return;
        }
        constexpr double kPi = 3.14159265358979323846;

        MagEllipsoidCalib::Result fit;
        m_sensorThread.magFitSnapshot(fit);

        // 取最近 N 个样本，N = 实测数据率(墙钟) × 窗口秒数。
        // 不用传感器时间戳切窗口：它是 frameCount×0.0025 的换算值，单位不可靠
        std::vector<SensorData> data;
        m_buffer.fillLatest(data, 700);
        const double tLast = data.empty() ? 0.0 : data.back().timestamp;
        const double rate5 = m_sensorThread.linkRate5s();
        int windowN = rate5 > 1.0 ? (int)(rate5 * (double)m_surveyWindowSec + 0.5) : 200;
        windowN = std::max(30, std::min(600, windowN));
        if ((int)data.size() > windowN)
            data.erase(data.begin(), data.end() - windowN);

        // ---- 静止样本统计（航向公式与 Control Panel 的 Mag Heading 完全一致） ----
        int nStill = 0, nInWin = 0;
        double sumSin = 0.0, sumCos = 0.0;
        double bSum = 0.0;
        double magSum[3] = { 0, 0, 0 }, bchSum[3] = { 0, 0, 0 }, accSum[3] = { 0, 0, 0 };
        std::vector<double> heads, bmags;
        for (const auto& d : data) {
            ++nInWin;
            const float gN = std::sqrt(d.gyro[0] * d.gyro[0] + d.gyro[1] * d.gyro[1]
                                     + d.gyro[2] * d.gyro[2]);
            const float aN = std::sqrt(d.acc[0] * d.acc[0] + d.acc[1] * d.acc[1]
                                     + d.acc[2] * d.acc[2]);
            if (gN > 5.0f || aN < 0.85f || aN > 1.15f) continue;
            const float axn = -d.acc[0] / aN, ayn = -d.acc[1] / aN, azn = -d.acc[2] / aN;
            const float roll = std::atan2(ayn, azn);
            const float pitch = std::atan2(-axn, std::sqrt(ayn * ayn + azn * azn));
            const float cr = std::cos(roll), sr = std::sin(roll);
            const float cp = std::cos(pitch), sp = std::sin(pitch);
            const float mxh = d.magUnc[0] * cp + d.magUnc[1] * sr * sp + d.magUnc[2] * cr * sp;
            const float myh = d.magUnc[1] * cr - d.magUnc[2] * sr;
            const double h = std::atan2(-(double)myh, (double)mxh);
            sumSin += std::sin(h);
            sumCos += std::cos(h);
            heads.push_back(h);
            const double bm = std::sqrt((double)d.magUnc[0] * d.magUnc[0]
                                      + (double)d.magUnc[1] * d.magUnc[1]
                                      + (double)d.magUnc[2] * d.magUnc[2]) * fit.fieldMag;
            bSum += bm;
            bmags.push_back(bm);
            for (int i = 0; i < 3; ++i) {
                magSum[i] += d.magUnc[i];
                bchSum[i] += d.magB[i];
                accSum[i] += d.acc[i];
            }
            ++nStill;
        }

        // 可 Mark 的条件与时间戳单位无关：窗口内至少 1/3 样本静止、至少 8 个
        // 静止样本、且数据流活着（墙钟 1.5 s 内还有事件到来）
        const bool streamAlive = m_sensorThread.eventAgeSeconds() < 1.5;
        const bool have = streamAlive && nStill >= 8 && 3 * nStill >= nInWin;
        double headingDeg = 0.0, headingDevDeg = 0.0, bMean = 0.0, bDev = 0.0;
        double shiftHdgDeg = 1e9, shiftBmag = 1e9, semHdgDeg = 0.0, semBmag = 0.0;
        if (nStill > 0) {
            const double headingRad = std::atan2(sumSin, sumCos);
            // 每样本散布（90 分位）只作记录参考：它衡量的是噪声大小，不代表平均
            // 值的精度——平均值的标准误差是 sigma/sqrt(n)，上百个样本能把
            // ±2.5 deg 的散布平均到 ±0.1 deg 量级。
            // 圆周差用 atan2(sin,cos) 折回 (-pi,pi]，不受 ±180° 跳变影响
            auto wrapPi = [](double a) { return std::atan2(std::sin(a), std::cos(a)); };
            std::vector<double> hd;
            hd.reserve(heads.size());
            double ssH = 0.0, ssB = 0.0;
            for (double h : heads) {
                const double dh = wrapPi(h - headingRad);
                hd.push_back(std::fabs(dh));
                ssH += dh * dh;
            }
            std::sort(hd.begin(), hd.end());
            headingDevDeg = hd[(size_t)(0.9 * (double)(hd.size() - 1))] * 180.0 / kPi;
            headingDeg = headingRad * 180.0 / kPi;
            semHdgDeg = std::sqrt(ssH / heads.size()) / std::sqrt((double)heads.size())
                        * 180.0 / kPi;

            bMean = bSum / nStill;
            std::vector<double> bd;
            bd.reserve(bmags.size());
            for (double b : bmags) {
                const double db = b - bMean;
                bd.push_back(std::fabs(db));
                ssB += db * db;
            }
            std::sort(bd.begin(), bd.end());
            bDev = bd[(size_t)(0.9 * (double)(bd.size() - 1))];
            semBmag = std::sqrt(ssB / bmags.size()) / std::sqrt((double)bmags.size());

            // 判稳看"前半窗 vs 后半窗的平均是否一致"（收敛性）：
            // 纯噪声时两个半窗平均几乎相同（差约 2*sigma/sqrt(n)），
            // 真实移动/未放稳时差异是度级的——这才是"读数还在不在变"的正确问法
            const size_t half = heads.size() / 2;
            if (half >= 4 && heads.size() - half >= 4) {
                double s1 = 0.0, c1 = 0.0, s2 = 0.0, c2 = 0.0;
                for (size_t i = 0; i < half; ++i) {
                    s1 += std::sin(heads[i]); c1 += std::cos(heads[i]);
                }
                for (size_t i = half; i < heads.size(); ++i) {
                    s2 += std::sin(heads[i]); c2 += std::cos(heads[i]);
                }
                shiftHdgDeg = std::fabs(wrapPi(std::atan2(s1, c1) - std::atan2(s2, c2)))
                              * 180.0 / kPi;
                double b1 = 0.0, b2 = 0.0;
                for (size_t i = 0; i < half; ++i) b1 += bmags[i];
                for (size_t i = half; i < bmags.size(); ++i) b2 += bmags[i];
                shiftBmag = std::fabs(b1 / (double)half - b2 / (double)(bmags.size() - half));
            }
        }

        ImGui::TextUnformatted("Put the sensor on a grid cell and hold still ~2 s, then Mark.");
        ImGui::TextUnformatted("Yellow 'Hold still' only means not enough quiet data yet.");
        ImGui::TextUnformatted("First mark is the reference (delta columns below).");
        if (!m_sensorThread.magCalibrationValid())
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                               "No mag calibration loaded - heading/|B| are NOT calibrated!");
        if (m_sensorThread.fieldMapEnabled())
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                "Field-map compensation is ON - marks stay raw (magUnc),\n"
                "but keep it OFF while surveying so map data stays self-consistent.");
        ImGui::Separator();

        if (data.empty()) {
            ImGui::TextUnformatted("Waiting for sensor data...");
        } else {
            // 稳定 = 半窗平均偏移小（读数已收敛）。这只提示、不拦截 Mark：
            // 每样本散布和偏移都会写进文件，质量可事后审计
            const bool stable = shiftHdgDeg < 0.75 && shiftBmag < 0.4;
            if (!have) {
                if (!streamAlive)
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                        "No data arriving - sensor link stalled?");
                else
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                        "Hold still... (%d of %d samples still)", nStill, nInWin);
            } else {
                ImGui::TextColored(stable ? ImVec4(0.4f, 1.0f, 0.4f, 1.0f)
                                          : ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                                   stable ? "STABLE" : "UNSTABLE");
                ImGui::SameLine();
                ImGui::Text("(%d/%d still, %.0f Hz, shift %.2f deg / %.2f uT)",
                            nStill, nInWin, rate5, shiftHdgDeg, shiftBmag);
            }
            if (nStill > 0) {
                ImGui::Text("Mag Heading    : %8.2f deg  (mean +/-%.2f, scatter +/-%.1f)",
                            headingDeg, semHdgDeg, headingDevDeg);
                ImGui::Text("|B| calibrated : %8.2f uT   (mean +/-%.2f, scatter +/-%.2f)",
                            bMean, semBmag, bDev);
            }
        }

        ImGui::Separator();
        ImGui::SliderInt("Avg window (s)", &m_surveyWindowSec, 1, 5);

        bool doMark = ImGui::IsKeyPressed(ImGuiKey_F9, false);
        ImGui::SetNextItemWidth(120);
        if (ImGui::InputText("Label", m_surveyLabel, sizeof(m_surveyLabel),
                             ImGuiInputTextFlags_EnterReturnsTrue))
            doMark = true;
        ImGui::SameLine();
        ImGui::BeginDisabled(!have);
        if (ImGui::Button("Mark (F9)")) doMark = true;
        ImGui::EndDisabled();

        if (doMark && have) {
            SensorThread::SurveyMark mk;
            mk.sensorTime = tLast;
            snprintf(mk.label, sizeof(mk.label), "%s", m_surveyLabel);
            mk.headingDeg = headingDeg;
            mk.headingDevDeg = headingDevDeg;
            mk.fieldMag = bMean;
            mk.fieldMagDev = bDev;
            for (int i = 0; i < 3; ++i) {
                mk.magUnit[i] = magSum[i] / nStill;
                mk.magB[i] = bchSum[i] / nStill;
                mk.acc[i] = accSum[i] / nStill;
            }
            mk.nStill = nStill;
            if (m_sensorThread.surveyMark(mk))
                m_surveyHistory.push_back(mk);
        }

        ImGui::Text("%s rows: %d", SensorThread::kSurveyFileName,
                    m_sensorThread.surveyRowCount());

        // ---- 历史表：最新在上，Δ 相对第一条（参考点） ----
        if (!m_surveyHistory.empty()) {
            if (ImGui::BeginTable("surveyHistory", 5,
                                  ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("Label");
                ImGui::TableSetupColumn("Heading(deg)");
                ImGui::TableSetupColumn("dHeading(deg)");
                ImGui::TableSetupColumn("|B|(uT)");
                ImGui::TableSetupColumn("d|B|(uT)");
                ImGui::TableHeadersRow();
                const auto& ref = m_surveyHistory.front();
                for (auto it = m_surveyHistory.rbegin(); it != m_surveyHistory.rend(); ++it) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(it->label);
                    ImGui::TableNextColumn();
                    ImGui::Text("%.2f", it->headingDeg);
                    ImGui::TableNextColumn();
                    double dh = it->headingDeg - ref.headingDeg;
                    dh = std::atan2(std::sin(dh * kPi / 180.0), std::cos(dh * kPi / 180.0))
                         * 180.0 / kPi;
                    // 偏差超过 3 度标红：环境干扰强的格点一眼可见
                    if (std::fabs(dh) >= 3.0)
                        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.4f, 1.0f), "%+.2f", dh);
                    else
                        ImGui::Text("%+.2f", dh);
                    ImGui::TableNextColumn();
                    ImGui::Text("%.2f", it->fieldMag);
                    ImGui::TableNextColumn();
                    ImGui::Text("%+.2f", it->fieldMag - ref.fieldMag);
                }
                ImGui::EndTable();
            }
        }

        ImGui::End();
    }

    /// Field Map 窗口：场图构建 / 参考格与活动格选择 / 补偿开关 / 验证读数。
    ///
    /// v0 语义：场图是"按格点的离散表"，补偿时传感器必须在"活动格"上——
    /// 格点选错 = 扣除错误的干扰向量。自动定位（磁指纹匹配）是下一阶段。
    void renderFieldMapWindow() {
        if (!m_state.showFieldMapWindow) return;
        ImGui::SetNextWindowSize(ImVec2(680, 500), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Field Map", &m_state.showFieldMapWindow)) {
            ImGui::End();
            return;
        }
        constexpr double kPi = 3.14159265358979323846;

        MagFieldMap::Result fmap;
        m_sensorThread.fieldMapSnapshot(fmap);

        ImGui::TextUnformatted("v0 discrete per-cell map. The ACTIVE cell's disturbance is");
        ImGui::TextUnformatted("subtracted before the filters - the sensor MUST be at that cell!");
        ImGui::Separator();

        if (ImGui::Button("Build from field_survey.txt")) {
            std::string msg;
            std::string refLabel;
            if (fmap.ok && m_mapRefSel >= 0 && m_mapRefSel < (int)fmap.entries.size())
                refLabel = fmap.entries[m_mapRefSel].label;
            if (m_sensorThread.buildFieldMap(refLabel, msg))
                m_mapMsg = msg;
            else
                m_mapMsg = "BUILD FAILED: " + msg;
            m_sensorThread.fieldMapSnapshot(fmap);
            m_mapRefSel = fmap.refIndex;
            m_mapActiveSel = std::min(m_mapActiveSel,
                                      std::max(0, (int)fmap.entries.size() - 1));
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", m_mapMsg.c_str());

        if (!fmap.ok) {
            ImGui::TextWrapped("No map yet. Mark cells with the Field Survey window "
                               "(labels like REF, A1 ...), then press Build.");
            ImGui::End();
            return;
        }

        // ---- 参考格 / 活动格 / 补偿开关 ----
        auto labelGetter = [](void* data, int idx, const char** out) {
            const auto* es = static_cast<const std::vector<MagFieldMap::Entry>*>(data);
            if (idx < 0 || idx >= (int)es->size()) return false;
            *out = (*es)[idx].label;
            return true;
        };
        ImGui::SetNextItemWidth(150);
        if (ImGui::Combo("Reference cell", &m_mapRefSel, labelGetter,
                         (void*)&fmap.entries, (int)fmap.entries.size())) {
            m_sensorThread.setFieldMapRef(m_mapRefSel);
            m_sensorThread.fieldMapSnapshot(fmap);
        }
        ImGui::SetNextItemWidth(150);
        if (ImGui::Combo("Active cell (sensor is HERE)", &m_mapActiveSel, labelGetter,
                         (void*)&fmap.entries, (int)fmap.entries.size())) {
            m_sensorThread.setFieldMapActive(m_mapActiveSel);
        }
        ImGui::SameLine();
        bool on = m_sensorThread.fieldMapEnabled();
        if (ImGui::Checkbox("Apply compensation", &on))
            m_sensorThread.setFieldMapEnabled(on);

        // ---- 一致性警告 ----
        MagEllipsoidCalib::Result fit;
        m_sensorThread.magFitSnapshot(fit);
        if (fmap.fieldMagAtBuild > 1e-6
            && std::fabs(fit.fieldMag - fmap.fieldMagAtBuild) > 0.02 * fmap.fieldMagAtBuild)
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                "Ellipsoid calibration changed since map was built (%.1f -> %.1f uT)"
                " - rebuild the map!", fmap.fieldMagAtBuild, fit.fieldMag);
        const int act = std::max(0, std::min(m_sensorThread.fieldMapActive(),
                                             (int)fmap.entries.size() - 1));
        if (m_sensorThread.fieldMapEnabled())
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f),
                "COMPENSATION ON - active cell: %s. Control Panel heading and all"
                " three filters now see the reference-point field.",
                fmap.entries[act].label);

        // ---- 验证读数：原始 vs 补偿后的实时航向（最近 ~1.5 s 圆周平均） ----
        // 单样本噪声 ±2~3 deg 没法读；航向是角度量，必须圆周平均避免 ±180° 跳变。
        // 移动/旋转时读数会滞后 ~1.5 s，停下来等它稳定再读。
        {
            std::vector<SensorData> data;
            const double rate5 = m_sensorThread.linkRate5s();
            int winN = rate5 > 1.0 ? (int)(rate5 * 1.5 + 0.5) : 150;
            winN = std::max(20, std::min(400, winN));
            m_buffer.fillLatest(data, (size_t)winN);
            double sR = 0, cR = 0, sC = 0, cC = 0, sD = 0, cD = 0;
            int nUsed = 0;
            for (const auto& d : data) {
                const float aN = std::sqrt(d.acc[0] * d.acc[0] + d.acc[1] * d.acc[1]
                                         + d.acc[2] * d.acc[2]);
                if (aN <= 0.0f) continue;
                auto headingOf = [&](const float m[3]) -> double {
                    const float axn = -d.acc[0] / aN, ayn = -d.acc[1] / aN, azn = -d.acc[2] / aN;
                    const float roll = std::atan2(ayn, azn);
                    const float pitch = std::atan2(-axn, std::sqrt(ayn * ayn + azn * azn));
                    const float cr = std::cos(roll), sr = std::sin(roll);
                    const float cp = std::cos(pitch), sp = std::sin(pitch);
                    const float mxh = m[0] * cp + m[1] * sr * sp + m[2] * cr * sp;
                    const float myh = m[1] * cr - m[2] * sr;
                    return std::atan2(-(double)myh, (double)mxh) * (180.0 / kPi);
                };
                const double hr = headingOf(d.magUnc) * kPi / 180.0;
                const double hc = headingOf(d.mag) * kPi / 180.0;
                sR += std::sin(hr); cR += std::cos(hr);
                sC += std::sin(hc); cC += std::cos(hc);
                const double dd = hc - hr;
                sD += std::sin(dd); cD += std::cos(dd);
                ++nUsed;
            }
            if (nUsed > 0) {
                const double hRaw = std::atan2(sR, cR) * 180.0 / kPi;
                const double hComp = std::atan2(sC, cC) * 180.0 / kPi;
                const double dlt = std::atan2(sD, cD) * 180.0 / kPi;
                ImGui::Separator();
                ImGui::Text("Heading raw          : %8.2f deg", hRaw);
                ImGui::Text("Heading compensated  : %8.2f deg", hComp);
                ImGui::Text("Delta (comp - raw)   : %+8.2f deg", dlt);
                ImGui::TextDisabled("comp = raw + delta;  |delta| should equal the active"
                                    " cell's |dHeading| (sign flipped: delta = ref - cell)");
                ImGui::TextDisabled("smoothed over ~1.5 s (%d samples); settles ~1.5 s"
                                    " after you stop moving", nUsed);
            }
        }

        // ---- 场图表格 ----
        ImGui::Separator();
        if (ImGui::BeginTable("mapEntries", 7,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Label");
            ImGui::TableSetupColumn("|B| (uT)");
            ImGui::TableSetupColumn("Heading (deg)");
            ImGui::TableSetupColumn("dHeading (deg)");
            ImGui::TableSetupColumn("|dB| (uT)");
            ImGui::TableSetupColumn("Scatter (deg)");
            ImGui::TableSetupColumn("N");
            ImGui::TableHeadersRow();
            const auto& eR = fmap.entries[fmap.refIndex];
            for (int i = 0; i < (int)fmap.entries.size(); ++i) {
                const auto& e = fmap.entries[i];
                ImGui::TableNextRow();
                if (i == act)
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1,
                                           IM_COL32(70, 90, 130, 100));
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(e.label);
                ImGui::TableNextColumn();
                ImGui::Text("%.2f", e.fieldMag);
                ImGui::TableNextColumn();
                ImGui::Text("%.2f", e.headingDeg);
                ImGui::TableNextColumn();
                double dh = e.headingDeg - eR.headingDeg;
                dh = std::atan2(std::sin(dh * kPi / 180.0), std::cos(dh * kPi / 180.0))
                     * 180.0 / kPi;
                if (std::fabs(dh) >= 3.0)
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.4f, 1.0f), "%+.2f", dh);
                else
                    ImGui::Text("%+.2f", dh);
                ImGui::TableNextColumn();
                double dL[3];
                fmap.deltaLevel(i, dL);
                ImGui::Text("%.2f",
                            std::sqrt(dL[0] * dL[0] + dL[1] * dL[1] + dL[2] * dL[2]));
                ImGui::TableNextColumn();
                ImGui::Text("%.2f", e.scatterDeg);
                ImGui::TableNextColumn();
                ImGui::Text("%d", e.nSamples);
            }
            ImGui::EndTable();
        }
        ImGui::TextDisabled("dHeading / |dB| are relative to the reference cell.");
        ImGui::TextDisabled("Scatter = max heading spread across sessions of the same label"
                            " (placement repeatability).");

        ImGui::End();
    }

    void renderEulerWindow() {
        if (!m_state.showEulerWindow) return;
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Euler Angles (Sensor)", &m_state.showEulerWindow);
        renderThreeAxisPlot("Euler Angles",
            [](const SensorData& d) { return d.euler[0]; },
            [](const SensorData& d) { return d.euler[1]; },
            [](const SensorData& d) { return d.euler[2]; },
            "Roll (deg)", "Pitch (deg)", "Yaw (deg, unwrapped)",
            IM_COL32(255, 0, 128, 255), IM_COL32(0, 255, 128, 255), IM_COL32(128, 0, 255, 255),
            true);
        ImGui::End();
    }

    void renderLinAccWindow() {
        if (!m_state.showLinAccWindow) return;
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Linear Acceleration (Sensor)", &m_state.showLinAccWindow);
        renderThreeAxisPlot("Linear Acceleration",
            [](const SensorData& d) { return d.linAcc[0]; },
            [](const SensorData& d) { return d.linAcc[1]; },
            [](const SensorData& d) { return d.linAcc[2]; },
            "X (g)", "Y (g)", "Z (g)",
            IM_COL32(255, 192, 0, 255), IM_COL32(192, 255, 0, 255), IM_COL32(0, 192, 255, 255));
        ImGui::End();
    }

    void renderEnvWindow() {
        if (!m_state.showEnvWindow) return;
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Environment", &m_state.showEnvWindow);

        std::vector<SensorData> data;
        m_buffer.fillLatest(data, m_state.historyLength);
        if (data.empty()) {
            ImGui::Text("Waiting for sensor data...");
            ImGui::End();
            return;
        }

        std::vector<double> xs(data.size());
        for (size_t i = 0; i < data.size(); i++) {
            xs[i] = data[i].timestamp;
        }

        std::vector<double> temp(data.size()), pressure(data.size());
        for (size_t i = 0; i < data.size(); i++) {
            temp[i] = data[i].temperature;
            pressure[i] = data[i].pressure;
        }

        if (ImPlot::BeginPlot("Temperature & Pressure", ImVec2(-1, m_state.plotHeight))) {
            ImPlot::SetupAxes("Time (s)", "Value");
            ImPlot::SetupAxisLimits(ImAxis_X1, xs.front(), xs.back(), ImGuiCond_Always);
            std::vector<double> allY;
            allY.insert(allY.end(), temp.begin(), temp.end());
            allY.insert(allY.end(), pressure.begin(), pressure.end());
            double yMin, yMax;
            computeAutoRange(allY, yMin, yMax);
            ImPlot::SetupAxisLimits(ImAxis_Y1, yMin, yMax, ImGuiCond_Always);
            // Y axis: denser ticks (2x default) + 1-decimal labels for finer scale reading
            ImPlot::SetupAxisFormat(ImAxis_Y1, "%.1f");
            ImPlot::SetupAxisTicks(ImAxis_Y1, yMin, yMax, std::max(11, (int)(m_state.plotHeight / 18)));
            ImPlot::PlotLine("Temperature (C)", xs.data(), temp.data(), (int)data.size());
            ImPlot::PlotLine("Pressure (mPa)", xs.data(), pressure.data(), (int)data.size());
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

    void renderFusedEulerWindow() {
        if (!m_state.showFusedEulerWindow) return;
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Fused Euler Angles (Madgwick)", &m_state.showFusedEulerWindow);
        renderThreeAxisPlot("Fused Euler Angles",
            [](const SensorData& d) { return d.fusedEuler[0]; },
            [](const SensorData& d) { return d.fusedEuler[1]; },
            [](const SensorData& d) { return d.fusedEuler[2]; },
            "Roll (deg)", "Pitch (deg)", "Yaw (deg, unwrapped)",
            IM_COL32(255, 0, 128, 255), IM_COL32(0, 255, 128, 255), IM_COL32(128, 0, 255, 255),
            true);
        ImGui::End();
    }

    void renderFusedLinAccWindow() {
        if (!m_state.showFusedLinAccWindow) return;
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Fused Linear Acceleration (Madgwick)", &m_state.showFusedLinAccWindow);
        renderThreeAxisPlot("Fused Linear Acceleration",
            [](const SensorData& d) { return d.fusedLinAcc[0]; },
            [](const SensorData& d) { return d.fusedLinAcc[1]; },
            [](const SensorData& d) { return d.fusedLinAcc[2]; },
            "X (g)", "Y (g)", "Z (g)",
            IM_COL32(255, 192, 0, 255), IM_COL32(192, 255, 0, 255), IM_COL32(0, 192, 255, 255));
        ImGui::End();
    }

    void renderEulerCompareWindow() {
        if (!m_state.showEulerCompareWindow) return;
        ImGui::SetNextWindowSize(ImVec2(800, 500), ImGuiCond_FirstUseEver);
        ImGui::Begin("Euler Angles Comparison", &m_state.showEulerCompareWindow);

        std::vector<SensorData> data;
        m_buffer.fillLatest(data, m_state.historyLength);
        if (data.empty()) {
            ImGui::Text("Waiting for sensor data...");
            ImGui::End();
            return;
        }

        std::vector<double> xs(data.size());
        for (size_t i = 0; i < data.size(); i++) {
            xs[i] = data[i].timestamp;
        }

        renderCompareSubplot("Roll Comparison", xs, data,
            [](const SensorData& d) { return d.euler[0]; },
            [](const SensorData& d) { return d.fusedEuler[0]; },
            "Sensor Roll", "Fused Roll");

        renderCompareSubplot("Pitch Comparison", xs, data,
            [](const SensorData& d) { return d.euler[1]; },
            [](const SensorData& d) { return d.fusedEuler[1]; },
            "Sensor Pitch", "Fused Pitch");

        renderCompareSubplot("Yaw Comparison", xs, data,
            [](const SensorData& d) { return d.euler[2]; },
            [](const SensorData& d) { return d.fusedEuler[2]; },
            "Sensor Yaw", "Fused Yaw", true);

        ImGui::End();
    }

    void renderSensor3dWindow() {
        if (!m_state.showSensor3dWindow) return;
        ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);
        ImGui::Begin("3D Orientation (Sensor)", &m_state.showSensor3dWindow);
        render3dOrientation("Sensor", [](const SensorData& d) { return d.quat; },
            [](const SensorData& d) { return d.euler; });
        ImGui::End();
    }

    void renderFused3dWindow() {
        if (!m_state.showFused3dWindow) return;
        ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);
        ImGui::Begin("3D Orientation (Fused)", &m_state.showFused3dWindow);
        render3dOrientation("Fused", [](const SensorData& d) { return d.fusedQuat; },
            [](const SensorData& d) { return d.fusedEuler; });
        ImGui::End();
    }

    void renderCompEulerWindow() {
        if (!m_state.showCompEulerWindow) return;
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Euler Angles (Complementary)", &m_state.showCompEulerWindow);
        renderThreeAxisPlot("Complementary Euler Angles",
            [](const SensorData& d) { return d.compEuler[0]; },
            [](const SensorData& d) { return d.compEuler[1]; },
            [](const SensorData& d) { return d.compEuler[2]; },
            "Roll (deg)", "Pitch (deg)", "Yaw (deg, unwrapped)",
            IM_COL32(255, 0, 128, 255), IM_COL32(0, 255, 128, 255), IM_COL32(128, 0, 255, 255),
            true);
        ImGui::End();
    }

    void renderCompLinAccWindow() {
        if (!m_state.showCompLinAccWindow) return;
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Linear Acceleration (Complementary)", &m_state.showCompLinAccWindow);
        renderThreeAxisPlot("Complementary Linear Acceleration",
            [](const SensorData& d) { return d.compLinAcc[0]; },
            [](const SensorData& d) { return d.compLinAcc[1]; },
            [](const SensorData& d) { return d.compLinAcc[2]; },
            "X (g)", "Y (g)", "Z (g)",
            IM_COL32(255, 192, 0, 255), IM_COL32(192, 255, 0, 255), IM_COL32(0, 192, 255, 255));
        ImGui::End();
    }

    void renderComp3dWindow() {
        if (!m_state.showComp3dWindow) return;
        ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);
        ImGui::Begin("3D Orientation (Complementary)", &m_state.showComp3dWindow);
        render3dOrientation("Complementary", [](const SensorData& d) { return d.compQuat; },
            [](const SensorData& d) { return d.compEuler; });
        ImGui::End();
    }

    void renderEulerAllCompareWindow() {
        if (!m_state.showEulerAllCompareWindow) return;
        ImGui::SetNextWindowSize(ImVec2(900, 600), ImGuiCond_FirstUseEver);
        ImGui::Begin("Euler Angles All Comparison (Sensor vs Madgwick vs Complementary)", &m_state.showEulerAllCompareWindow);

        std::vector<SensorData> data;
        m_buffer.fillLatest(data, m_state.historyLength);
        if (data.empty()) {
            ImGui::Text("Waiting for sensor data...");
            ImGui::End();
            return;
        }

        std::vector<double> xs(data.size());
        for (size_t i = 0; i < data.size(); i++) {
            xs[i] = data[i].timestamp;
        }

        renderThreeCompareSubplot("Roll Comparison", xs, data,
            [](const SensorData& d) { return d.euler[0]; },
            [](const SensorData& d) { return d.fusedEuler[0]; },
            [](const SensorData& d) { return d.compEuler[0]; },
            "Sensor Roll", "Madgwick Roll", "Complementary Roll");

        renderThreeCompareSubplot("Pitch Comparison", xs, data,
            [](const SensorData& d) { return d.euler[1]; },
            [](const SensorData& d) { return d.fusedEuler[1]; },
            [](const SensorData& d) { return d.compEuler[1]; },
            "Sensor Pitch", "Madgwick Pitch", "Complementary Pitch");

        renderThreeCompareSubplot("Yaw Comparison", xs, data,
            [](const SensorData& d) { return d.euler[2]; },
            [](const SensorData& d) { return d.fusedEuler[2]; },
            [](const SensorData& d) { return d.compEuler[2]; },
            "Sensor Yaw", "Madgwick Yaw", "Complementary Yaw", true);

        ImGui::End();
    }

    void render3dOrientation(const char* label,
        std::function<const float*(const SensorData&)> quatGetter,
        std::function<const float*(const SensorData&)> eulerGetter)
    {
        ImGui::Text("%s 3D Orientation", label);
        ImGui::Separator();

        ImVec2 canvasSize = ImGui::GetContentRegionAvail();
        float size = std::min(canvasSize.x, canvasSize.y - 100) * 0.5f;
        ImVec2 drawOffset = ImGui::GetCursorScreenPos();
        ImVec2 center(canvasSize.x / 2, (canvasSize.y - 100) / 2);

        std::vector<SensorData> data;
        m_buffer.fillLatest(data, 1);

        float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float euler[3] = {0.0f, 0.0f, 0.0f};

        if (!data.empty()) {
            const float* qData = quatGetter(data[0]);
            const float* eData = eulerGetter(data[0]);
            for (int i = 0; i < 4; i++) q[i] = qData[i];
            for (int i = 0; i < 3; i++) euler[i] = eData[i];
        }

        ImDrawList* drawList = ImGui::GetWindowDrawList();
        float halfSize = size / 2;

        float cubeVertices[8][3] = {
            {-halfSize, -halfSize, -halfSize},
            {halfSize, -halfSize, -halfSize},
            {halfSize, halfSize, -halfSize},
            {-halfSize, halfSize, -halfSize},
            {-halfSize, -halfSize, halfSize},
            {halfSize, -halfSize, halfSize},
            {halfSize, halfSize, halfSize},
            {-halfSize, halfSize, halfSize}
        };

        int cubeEdges[12][2] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0},
            {4, 5}, {5, 6}, {6, 7}, {7, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}
        };

        float rotated[8][3];
        for (int i = 0; i < 8; i++) {
            float x = cubeVertices[i][0];
            float y = cubeVertices[i][1];
            float z = cubeVertices[i][2];
            float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
            float x2 = x * (1 - 2*qy*qy - 2*qz*qz) + y * (2*qx*qy - 2*qz*qw) + z * (2*qx*qz + 2*qy*qw);
            float y2 = x * (2*qx*qy + 2*qz*qw) + y * (1 - 2*qx*qx - 2*qz*qz) + z * (2*qy*qz - 2*qx*qw);
            float z2 = x * (2*qx*qz - 2*qy*qw) + y * (2*qy*qz + 2*qx*qw) + z * (1 - 2*qx*qx - 2*qy*qy);
            rotated[i][0] = x2;
            rotated[i][1] = y2;
            rotated[i][2] = z2;
        }

        float scaleX = 1.0f;
        float scaleY = 0.6f;
        float isoAngle = 0.785398f;

        float projected[8][2];
        for (int i = 0; i < 8; i++) {
            float x = rotated[i][0];
            float y = rotated[i][1];
            float z = rotated[i][2];
            projected[i][0] = drawOffset.x + center.x + (x * cosf(isoAngle) - z * cosf(isoAngle)) * scaleX;
            projected[i][1] = drawOffset.y + center.y - (y + (x * sinf(isoAngle) + z * sinf(isoAngle))) * scaleY;
        }

        ImVec2 centerScreen(drawOffset.x + center.x, drawOffset.y + center.y);

        for (int i = 0; i < 12; i++) {
            drawList->AddLine(
                ImVec2(projected[cubeEdges[i][0]][0], projected[cubeEdges[i][0]][1]),
                ImVec2(projected[cubeEdges[i][1]][0], projected[cubeEdges[i][1]][1]),
                IM_COL32(255, 255, 255, 255), 2.0f
            );
        }

        drawList->AddCircleFilled(centerScreen, 4.0f, IM_COL32(255, 0, 0, 255));
        float axisLen = size * 0.6f;
        drawList->AddLine(centerScreen, ImVec2(centerScreen.x + axisLen * cosf(isoAngle), centerScreen.y - axisLen * sinf(isoAngle) * scaleY), IM_COL32(255, 0, 0, 200), 2.0f);
        drawList->AddLine(centerScreen, ImVec2(centerScreen.x, centerScreen.y - axisLen * scaleY), IM_COL32(0, 255, 0, 200), 2.0f);
        drawList->AddLine(centerScreen, ImVec2(centerScreen.x - axisLen * cosf(isoAngle), centerScreen.y - axisLen * sinf(isoAngle) * scaleY), IM_COL32(0, 0, 255, 200), 2.0f);
        float labelOffset = 10.0f;
        drawList->AddText(ImVec2(centerScreen.x + axisLen * cosf(isoAngle) + labelOffset, centerScreen.y - axisLen * sinf(isoAngle) * scaleY - 5), IM_COL32(255, 0, 0, 255), "X");
        drawList->AddText(ImVec2(centerScreen.x + labelOffset, centerScreen.y - axisLen * scaleY - 15), IM_COL32(0, 255, 0, 255), "Y");
        drawList->AddText(ImVec2(centerScreen.x - axisLen * cosf(isoAngle) - 15, centerScreen.y - axisLen * sinf(isoAngle) * scaleY - 5), IM_COL32(0, 0, 255, 255), "Z");

        ImGui::SetCursorPos(ImVec2(10, canvasSize.y - 90));
        ImGui::Text("Quaternion: W=%.4f X=%.4f Y=%.4f Z=%.4f", q[0], q[1], q[2], q[3]);
        ImGui::Text("Roll: %.1f deg | Pitch: %.1f deg | Yaw: %.1f deg", euler[0], euler[1], euler[2]);
    }

    /**
     * @brief unwrap
     *
     *  [-180,180]  [0,360] 
     *  180 3600
     *  360  180  360 
     * computeAutoRange() 
     */
    static void unwrapInPlace(std::vector<double>& ys) {
        for (size_t i = 1; i < ys.size(); i++) {
            double k = std::floor((ys[i - 1] - ys[i] + 180.0) / 360.0);
            ys[i] += 360.0 * k;  //  <180 
        }
    }

    /**
     * @brief  360  ref 
     *
     *  yaw  0~360 yaw  180
     *  360 350 vs  -10
     *  360 
     */
    static void alignToRef(std::vector<double>& ys, double ref) {
        if (ys.empty()) return;
        double k = std::floor((ref - ys.front() + 180.0) / 360.0);
        double shift = 360.0 * k;
        if (shift == 0.0) return;
        for (auto& v : ys) v += shift;
    }

    void computeAutoRange(const std::vector<double>& yValues, double& yMin, double& yMax) {
        if (yValues.empty()) {
            yMin = -1.0;
            yMax = 1.0;
            return;
        }
        yMin = *std::min_element(yValues.begin(), yValues.end());
        yMax = *std::max_element(yValues.begin(), yValues.end());
        double range = yMax - yMin;
        if (range < 1e-9) {
            range = 1.0;
        }
        double padding = range * 0.15;
        double niceStep = computeNiceStep(range / 10.0);
        yMin = std::floor((yMin - padding) / niceStep) * niceStep;
        yMax = std::ceil((yMax + padding) / niceStep) * niceStep;
    }

    double computeNiceStep(double roughStep) {
        if (roughStep <= 0) roughStep = 0.1;
        double magnitude = std::pow(10.0, std::floor(std::log10(roughStep)));
        double normalized = roughStep / magnitude;
        double niceStep;
        if (normalized < 1.5) niceStep = 1.0;
        else if (normalized < 3) niceStep = 2.0;
        else if (normalized < 7) niceStep = 5.0;
        else niceStep = 10.0;
        return niceStep * magnitude;
    }

    template<typename F1, typename F2, typename F3>
    void renderThreeAxisPlot(const char* title, F1 getter1, F2 getter2, F3 getter3,
        const char* label1, const char* label2, const char* label3,
        ImU32 color1, ImU32 color2, ImU32 color3, bool unwrapThirdAxis = false)
    {
        std::vector<SensorData> data;
        m_buffer.fillLatest(data, m_state.historyLength);
        if (data.empty()) {
            ImGui::Text("Waiting for sensor data...");
            return;
        }

        std::vector<double> xs(data.size()), ys1(data.size()), ys2(data.size()), ys3(data.size());
        for (size_t i = 0; i < data.size(); i++) {
            xs[i] = data[i].timestamp;
            ys1[i] = getter1(data[i]);
            ys2[i] = getter2(data[i]);
            ys3[i] = getter3(data[i]);
        }

        //  180/360 
        if (unwrapThirdAxis) {
            unwrapInPlace(ys3);
        }

        if (ImPlot::BeginPlot(title, ImVec2(-1, m_state.plotHeight))) {
            ImPlot::SetupAxes("Time (s)", "Value");
            ImPlot::SetupAxisLimits(ImAxis_X1, xs.front(), xs.back(), ImGuiCond_Always);
            std::vector<double> allY;
            allY.insert(allY.end(), ys1.begin(), ys1.end());
            allY.insert(allY.end(), ys2.begin(), ys2.end());
            allY.insert(allY.end(), ys3.begin(), ys3.end());
            double yMin, yMax;
            computeAutoRange(allY, yMin, yMax);
            ImPlot::SetupAxisLimits(ImAxis_Y1, yMin, yMax, ImGuiCond_Always);
            // Y axis: denser ticks (2x default) + 1-decimal labels for finer scale reading
            ImPlot::SetupAxisFormat(ImAxis_Y1, "%.1f");
            ImPlot::SetupAxisTicks(ImAxis_Y1, yMin, yMax, std::max(11, (int)(m_state.plotHeight / 18)));
            ImPlot::PlotLine(label1, xs.data(), ys1.data(), (int)data.size());
            ImPlot::PlotLine(label2, xs.data(), ys2.data(), (int)data.size());
            ImPlot::PlotLine(label3, xs.data(), ys3.data(), (int)data.size());
            ImPlot::EndPlot();
        }
    }

    template<typename F1, typename F2>
    void renderCompareSubplot(const char* title, const std::vector<double>& xs, const std::vector<SensorData>& data,
        F1 getter1, F2 getter2, const char* label1, const char* label2, bool unwrapAlign = false)
    {
        std::vector<double> ys1(data.size()), ys2(data.size());
        for (size_t i = 0; i < data.size(); i++) {
            ys1[i] = getter1(data[i]);
            ys2[i] = getter2(data[i]);
        }

        // 360 
        if (unwrapAlign) {
            unwrapInPlace(ys1);
            unwrapInPlace(ys2);
            alignToRef(ys2, ys1.front());
        }

        if (ImPlot::BeginPlot(title, ImVec2(-1, m_state.plotHeight))) {
            ImPlot::SetupAxes("Time (s)", "Value");
            ImPlot::SetupAxisLimits(ImAxis_X1, xs.front(), xs.back(), ImGuiCond_Always);
            std::vector<double> allY;
            allY.insert(allY.end(), ys1.begin(), ys1.end());
            allY.insert(allY.end(), ys2.begin(), ys2.end());
            double yMin, yMax;
            computeAutoRange(allY, yMin, yMax);
            ImPlot::SetupAxisLimits(ImAxis_Y1, yMin, yMax, ImGuiCond_Always);
            // Y axis: denser ticks (2x default) + 1-decimal labels for finer scale reading
            ImPlot::SetupAxisFormat(ImAxis_Y1, "%.1f");
            ImPlot::SetupAxisTicks(ImAxis_Y1, yMin, yMax, std::max(11, (int)(m_state.plotHeight / 18)));
            ImPlot::PlotLine(label1, xs.data(), ys1.data(), (int)data.size());
            ImPlot::PlotLine(label2, xs.data(), ys2.data(), (int)data.size());
            ImPlot::EndPlot();
        }
    }

    template<typename F1, typename F2, typename F3>
    void renderThreeCompareSubplot(const char* title, const std::vector<double>& xs, const std::vector<SensorData>& data,
        F1 getter1, F2 getter2, F3 getter3, const char* label1, const char* label2, const char* label3,
        bool unwrapAlign = false)
    {
        std::vector<double> ys1(data.size()), ys2(data.size()), ys3(data.size());
        for (size_t i = 0; i < data.size(); i++) {
            ys1[i] = getter1(data[i]);
            ys2[i] = getter2(data[i]);
            ys3[i] = getter3(data[i]);
        }

        // 360 
        if (unwrapAlign) {
            unwrapInPlace(ys1);
            unwrapInPlace(ys2);
            unwrapInPlace(ys3);
            alignToRef(ys2, ys1.front());
            alignToRef(ys3, ys1.front());
        }

        if (ImPlot::BeginPlot(title, ImVec2(-1, m_state.plotHeight))) {
            ImPlot::SetupAxes("Time (s)", "Value");
            ImPlot::SetupAxisLimits(ImAxis_X1, xs.front(), xs.back(), ImGuiCond_Always);
            std::vector<double> allY;
            allY.insert(allY.end(), ys1.begin(), ys1.end());
            allY.insert(allY.end(), ys2.begin(), ys2.end());
            allY.insert(allY.end(), ys3.begin(), ys3.end());
            double yMin, yMax;
            computeAutoRange(allY, yMin, yMax);
            ImPlot::SetupAxisLimits(ImAxis_Y1, yMin, yMax, ImGuiCond_Always);
            // Y axis: denser ticks (2x default) + 1-decimal labels for finer scale reading
            ImPlot::SetupAxisFormat(ImAxis_Y1, "%.1f");
            ImPlot::SetupAxisTicks(ImAxis_Y1, yMin, yMax, std::max(11, (int)(m_state.plotHeight / 18)));
            ImPlot::PlotLine(label1, xs.data(), ys1.data(), (int)data.size());
            ImPlot::PlotLine(label2, xs.data(), ys2.data(), (int)data.size());
            ImPlot::PlotLine(label3, xs.data(), ys3.data(), (int)data.size());
            ImPlot::EndPlot();
        }
    }

    //  + Madgwick/Complementary/EKF
    template <typename F1, typename F2, typename F3, typename F4>
    void renderFourCompareSubplot(const char* title, const std::vector<double>& xs, const std::vector<SensorData>& data,
        F1 getter1, F2 getter2, F3 getter3, F4 getter4,
        const char* label1, const char* label2, const char* label3, const char* label4,
        bool unwrapAlign = false)
    {
        std::vector<double> ys1(data.size()), ys2(data.size()), ys3(data.size()), ys4(data.size());
        for (size_t i = 0; i < data.size(); i++) {
            ys1[i] = getter1(data[i]);
            ys2[i] = getter2(data[i]);
            ys3[i] = getter3(data[i]);
            ys4[i] = getter4(data[i]);
        }

        if (unwrapAlign) {
            unwrapInPlace(ys1);
            unwrapInPlace(ys2);
            unwrapInPlace(ys3);
            unwrapInPlace(ys4);
            alignToRef(ys2, ys1.front());
            alignToRef(ys3, ys1.front());
            alignToRef(ys4, ys1.front());
        }

        if (ImPlot::BeginPlot(title, ImVec2(-1, m_state.plotHeight))) {
            ImPlot::SetupAxes("Time (s)", "Value");
            ImPlot::SetupAxisLimits(ImAxis_X1, xs.front(), xs.back(), ImGuiCond_Always);
            std::vector<double> allY;
            allY.insert(allY.end(), ys1.begin(), ys1.end());
            allY.insert(allY.end(), ys2.begin(), ys2.end());
            allY.insert(allY.end(), ys3.begin(), ys3.end());
            allY.insert(allY.end(), ys4.begin(), ys4.end());
            double yMin, yMax;
            computeAutoRange(allY, yMin, yMax);
            ImPlot::SetupAxisLimits(ImAxis_Y1, yMin, yMax, ImGuiCond_Always);
            // Y axis: denser ticks (2x default) + 1-decimal labels for finer scale reading
            ImPlot::SetupAxisFormat(ImAxis_Y1, "%.1f");
            ImPlot::SetupAxisTicks(ImAxis_Y1, yMin, yMax, std::max(11, (int)(m_state.plotHeight / 18)));
            ImPlot::PlotLine(label1, xs.data(), ys1.data(), (int)data.size());
            ImPlot::PlotLine(label2, xs.data(), ys2.data(), (int)data.size());
            ImPlot::PlotLine(label3, xs.data(), ys3.data(), (int)data.size());
            ImPlot::PlotLine(label4, xs.data(), ys4.data(), (int)data.size());
            ImPlot::EndPlot();
        }
    }

    //  EKF 
    void renderEkfEulerWindow() {
        if (!m_state.showEkfEulerWindow) return;
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("EKF Euler Angles", &m_state.showEkfEulerWindow);
        renderThreeAxisPlot("EKF Euler Angles",
            [](const SensorData& d) { return d.ekfEuler[0]; },
            [](const SensorData& d) { return d.ekfEuler[1]; },
            [](const SensorData& d) { return d.ekfEuler[2]; },
            "Roll (deg)", "Pitch (deg)", "Yaw (deg, unwrapped)",
            IM_COL32(255, 0, 128, 255), IM_COL32(0, 255, 128, 255), IM_COL32(128, 0, 255, 255),
            true);
        ImGui::End();
    }

    //   +  
    void renderEkfCompareWindow() {
        if (!m_state.showEkfCompareWindow) return;
        ImGui::SetNextWindowSize(ImVec2(900, 600), ImGuiCond_FirstUseEver);
        ImGui::Begin("Euler Compare (Sensor vs Madgwick vs Complementary vs EKF)", &m_state.showEkfCompareWindow);

        std::vector<SensorData> data;
        m_buffer.fillLatest(data, m_state.historyLength);
        if (data.empty()) {
            ImGui::Text("Waiting for sensor data...");
            ImGui::End();
            return;
        }

        std::vector<double> xs(data.size());
        for (size_t i = 0; i < data.size(); i++) xs[i] = data[i].timestamp;

        renderFourCompareSubplot("Roll Comparison", xs, data,
            [](const SensorData& d) { return d.euler[0]; },
            [](const SensorData& d) { return d.fusedEuler[0]; },
            [](const SensorData& d) { return d.compEuler[0]; },
            [](const SensorData& d) { return d.ekfEuler[0]; },
            "Sensor", "Madgwick", "Complementary", "EKF");

        renderFourCompareSubplot("Pitch Comparison", xs, data,
            [](const SensorData& d) { return d.euler[1]; },
            [](const SensorData& d) { return d.fusedEuler[1]; },
            [](const SensorData& d) { return d.compEuler[1]; },
            [](const SensorData& d) { return d.ekfEuler[1]; },
            "Sensor", "Madgwick", "Complementary", "EKF");

        renderFourCompareSubplot("Yaw Comparison", xs, data,
            [](const SensorData& d) { return d.euler[2]; },
            [](const SensorData& d) { return d.fusedEuler[2]; },
            [](const SensorData& d) { return d.compEuler[2]; },
            [](const SensorData& d) { return d.ekfEuler[2]; },
            "Sensor", "Madgwick", "Complementary", "EKF", true);

        ImGui::End();
    }

    // ==================================================================
    //  磁椭球校准模型可视化
    //
    //  三件事必须能直接看出来，而不是靠读数字：
    //    1) 拟合出的椭球长什么样、离原点多远  -> 硬铁偏移
    //    2) 椭球离"正球"差多少              -> 软铁 + 拟合残差（球面磁误差）
    //    3) 这些误差换成 yaw 是多少度        -> 磁对航向的实际影响
    // ==================================================================

    static constexpr double kPi = 3.14159265358979323846;

    /// ImPlot 的样式接口收 ImVec4，而本文件其余部分统一用 IM_COL32 打包色。
    static ImVec4 colOf(ImU32 c) { return ImGui::ColorConvertU32ToFloat4(c); }

    void renderMagCalibWindow() {
        if (!m_state.showMagCalibWindow) return;
        ImGui::SetNextWindowSize(ImVec2(1060, 960), ImGuiCond_FirstUseEver);
        ImGui::Begin("Mag Calibration (Ellipsoid Fit)", &m_state.showMagCalibWindow);

        MagEllipsoidCalib::Result fit;
        m_sensorThread.magFitSnapshot(fit);
        const bool hasFit = m_sensorThread.magCalibrationValid();

        std::vector<std::array<float, 3>> samples;
        m_sensorThread.magSamplesSnapshot(samples, 4000);

        std::vector<SensorData> live;
        m_buffer.fillLatest(live, m_state.historyLength);

        if (!hasFit && samples.empty()) {
            ImGui::TextWrapped(
                "No calibration yet.\n\n"
                "Press \"Start Mag Calibration\" in the control panel, tumble the sensor slowly "
                "through EVERY orientation (all six faces up, all twelve edges down, figure-8s), "
                "well away from the PC, steel furniture and speakers, then press \"Stop & Apply\".\n\n"
                "While collecting, this window shows the point cloud building up so you can see "
                "which orientations are still missing. After the fit it shows the ellipsoid, the "
                "spherical error and the magnetic-impact metrics.");
            if (ImGui::Button("Reload from disk")) {
                m_reloadMsg = m_sensorThread.reloadMagCalibrationFromDisk()
                    ? "Loaded mag_calib.txt." : "No usable mag_calib.txt found.";
            }
            ImGui::SameLine();
            ImGui::TextDisabled("%s", m_reloadMsg.c_str());
            ImGui::End();
            return;
        }

        if (ImGui::Button("Reload from disk")) {
            m_reloadMsg = m_sensorThread.reloadMagCalibrationFromDisk()
                ? "Loaded mag_calib.txt." : "No usable mag_calib.txt found.";
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Re-read mag_calib.txt and mag_calib_samples.txt from the exe directory.\n"
                              "Use this after regenerating the calibration with an offline tool.\n"
                              "The three fusion filters are reset, because the yaw reference moves.");
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", m_reloadMsg.c_str());

        const double range = magViewRange(fit, hasFit, samples);

        drawMagCloud3d(fit, hasFit, samples, range);
        drawMagProjections(fit, hasFit, samples, range);
        drawSphericalError(fit, hasFit, samples, live);
        drawMagImpact(fit, hasFit, samples, live);

        ImGui::End();
    }

    /// 视图半径：必须同时装下参考球和整个椭球（含硬铁偏移），否则椭球会被裁掉。
    static double magViewRange(const MagEllipsoidCalib::Result& fit, bool hasFit,
                               const std::vector<std::array<float, 3>>& samples) {
        double r = hasFit ? fit.fieldMag : 1.0;
        if (hasFit) {
            const double maxAxis = std::max(fit.semiAxis[0], std::max(fit.semiAxis[1], fit.semiAxis[2]));
            r = std::max(r, fit.hardIron + maxAxis);
        }
        for (const auto& s : samples) {
            const double n = std::sqrt((double)s[0] * s[0] + (double)s[1] * s[1] + (double)s[2] * s[2]);
            r = std::max(r, n);
        }
        return r * 1.12 + 1e-9;
    }

    static void meanSd(const std::vector<double>& v, double& mean, double& sd) {
        if (v.empty()) { mean = 0; sd = 0; return; }
        mean = 0;
        for (double x : v) mean += x;
        mean /= (double)v.size();
        double s = 0;
        for (double x : v) s += (x - mean) * (x - mean);
        sd = std::sqrt(s / (double)v.size());
    }

    // ------------------------------------------------------------------
    // 1. 3D 视图（原始 µT 空间）
    // ------------------------------------------------------------------

    /**
     * 在原点上画一个半径 = 当地磁场强度的绿色参考球：那是一个"理想磁力计"
     * 在各个姿态下应该描出的轨迹。拟合椭球离这个球多远、偏到哪里，
     * 硬铁偏移就一目了然 —— 不需要读任何数字。
     */
    void drawMagCloud3d(const MagEllipsoidCalib::Result& fit, bool hasFit,
                        const std::vector<std::array<float, 3>>& samples, double range) {
        ImGui::SeparatorText("1. 3D view - raw uT space");

        ImGui::Checkbox("Auto-spin", &m_state.magSpin);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("##magAzim", &m_state.magAzim, 0.0f, 360.0f, "azim %.0f deg");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("##magElev", &m_state.magElev, 0.0f, 90.0f, "elev %.0f deg");

        const float azDeg = m_state.magAzim + (m_state.magSpin ? (float)(ImGui::GetTime() * 20.0) : 0.0f);
        const double azR = azDeg * kPi / 180.0, elR = m_state.magElev * kPi / 180.0;
        const double ca = std::cos(azR), sa = std::sin(azR);
        const double ce = std::cos(elR), se = std::sin(elR);

        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const float W = std::max(200.0f, avail.x), H = 430.0f;
        const ImVec2 o = ImGui::GetCursorScreenPos();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImGui::Dummy(ImVec2(W, H));

        const ImVec2 c(o.x + W * 0.5f, o.y + H * 0.5f);
        const float scale = (float)(std::min(W, H) * 0.47 / range);

        // 机体系 +Z 朝下，竖直方向取 -z，画面才符合直觉
        auto proj = [&](double x, double y, double z, ImVec2& out) {
            const double xr = x * ca - y * sa;
            const double yr = x * sa + y * ca;
            out.x = c.x + (float)(xr * scale);
            out.y = c.y - (float)((yr * ce - z * se) * scale);
        };

        const int N = 80;

        // --- 理想参考球（原点为心，半径 = |B|）---
        if (hasFit && fit.fieldMag > 1e-6) {
            const ImU32 green = IM_COL32(90, 220, 120, 190);
            const double R = fit.fieldMag;
            for (int ring = 0; ring < 3; ++ring) {
                ImVec2 prev{};
                for (int i = 0; i <= N; ++i) {
                    const double t = 2.0 * kPi * i / N;
                    double x = 0, y = 0, z = 0;
                    if (ring == 0)      { x = R * std::cos(t); y = R * std::sin(t); }
                    else if (ring == 1) { x = R * std::cos(t); z = R * std::sin(t); }
                    else                { y = R * std::cos(t); z = R * std::sin(t); }
                    ImVec2 p; proj(x, y, z, p);
                    if (i > 0) dl->AddLine(prev, p, green, 1.3f);
                    prev = p;
                }
            }
        }

        // --- 拟合椭球线框（3 纬圈 + 3 经圈，经 C = M^-1 映射到椭球面上）---
        double C[9];
        const bool haveInv = hasFit && MagEllipsoidCalib::inverseM(fit, C);
        if (haveInv) {
            const ImU32 cyan = IM_COL32(80, 200, 255, 230);
            for (int k = 0; k < 6; ++k) {
                ImVec2 prev{};
                for (int i = 0; i <= N; ++i) {
                    const double t = 2.0 * kPi * i / N;
                    double ux, uy, uz;
                    if (k < 3) {
                        const double lat = (k - 1) * 45.0 * kPi / 180.0;
                        const double cl = std::cos(lat);
                        ux = cl * std::cos(t); uy = cl * std::sin(t); uz = std::sin(lat);
                    } else {
                        const double lon = (k - 3) * 60.0 * kPi / 180.0;
                        ux = std::cos(t) * std::cos(lon); uy = std::cos(t) * std::sin(lon); uz = std::sin(t);
                    }
                    const double x = fit.center[0] + C[0] * ux + C[1] * uy + C[2] * uz;
                    const double y = fit.center[1] + C[3] * ux + C[4] * uy + C[5] * uz;
                    const double z = fit.center[2] + C[6] * ux + C[7] * uy + C[8] * uz;
                    ImVec2 p; proj(x, y, z, p);
                    if (i > 0) dl->AddLine(prev, p, cyan, 1.7f);
                    prev = p;
                }
            }
        }

        // --- 采样点云 ---
        {
            const ImU32 grey = IM_COL32(215, 215, 215, 140);
            const size_t n = samples.size();
            const size_t stride = n > 1400 ? n / 1400 : 1;
            for (size_t i = 0; i < n; i += stride) {
                ImVec2 p; proj(samples[i][0], samples[i][1], samples[i][2], p);
                dl->AddCircleFilled(p, 1.8f, grey);
            }
        }

        // --- 坐标轴、原点、硬铁向量 ---
        {
            ImVec2 p0; proj(0, 0, 0, p0);
            const double axis[3][3] = { {range, 0, 0}, {0, range, 0}, {0, 0, range} };
            const ImU32 acol[3] = { IM_COL32(255, 95, 95, 225),
                                    IM_COL32(130, 255, 130, 225),
                                    IM_COL32(110, 155, 255, 225) };
            const char* anm[3] = { "X", "Y", "Z" };
            for (int i = 0; i < 3; ++i) {
                ImVec2 p1; proj(axis[i][0], axis[i][1], axis[i][2], p1);
                dl->AddLine(p0, p1, acol[i], 1.5f);
                dl->AddText(ImVec2(p1.x + 5.0f, p1.y - 9.0f), acol[i], anm[i]);
            }
            dl->AddCircleFilled(p0, 3.5f, IM_COL32(255, 255, 255, 255));
            dl->AddText(ImVec2(p0.x + 7.0f, p0.y + 5.0f), IM_COL32(255, 255, 255, 220), "origin");

            if (hasFit) {
                ImVec2 pc; proj(fit.center[0], fit.center[1], fit.center[2], pc);
                const ImU32 yellow = IM_COL32(255, 214, 80, 255);
                dl->AddLine(p0, pc, yellow, 2.2f);
                dl->AddCircleFilled(pc, 4.5f, yellow);
                char buf[96];
                std::snprintf(buf, sizeof(buf), "ellipsoid center | hard iron %.1f uT", fit.hardIron);
                dl->AddText(ImVec2(pc.x + 9.0f, pc.y - 11.0f), yellow, buf);
            }
        }

        ImGui::TextDisabled("green sphere at origin = ideal magnetometer | cyan cage = fitted ellipsoid | yellow line = hard iron");
    }

    // ------------------------------------------------------------------
    // 2. 三视图
    // ------------------------------------------------------------------

    void drawMagProjections(const MagEllipsoidCalib::Result& fit, bool hasFit,
                            const std::vector<std::array<float, 3>>& samples, double range) {
        ImGui::SeparatorText("2. Orthographic projections");

        if (samples.empty()) {
            ImGui::TextDisabled("No samples recorded yet.");
            return;
        }

        if (!ImGui::BeginTable("##magproj", 3, ImGuiTableFlags_SizingStretchSame)) return;
        ImGui::TableNextRow();

        const int planes[3][2] = { {0, 1}, {0, 2}, {1, 2} };
        const char* names[3] = { "X-Y plane", "X-Z plane", "Y-Z plane" };
        const char* axlab[3] = { "X (uT)", "Y (uT)", "Z (uT)" };

        for (int p = 0; p < 3; ++p) {
            ImGui::TableSetColumnIndex(p);
            const int ia = planes[p][0], ib = planes[p][1];

            const size_t stride = samples.size() > 2500 ? samples.size() / 2500 : 1;
            std::vector<double> sx, sy;
            sx.reserve(samples.size() / stride + 1);
            sy.reserve(samples.size() / stride + 1);
            for (size_t i = 0; i < samples.size(); i += stride) {
                sx.push_back(samples[i][ia]);
                sy.push_back(samples[i][ib]);
            }

            const float side = std::max(120.0f, ImGui::GetContentRegionAvail().x);
            // 内绘图区要尽量是正方形，否则正圆会被压成椭圆，"是不是球"就看不出来了。
            // 去掉图例并补偿坐标轴标签占的边距（左约 52px、上约 30px）。
            if (!ImPlot::BeginPlot(names[p], ImVec2(side, side - 22.0f), ImPlotFlags_NoLegend)) continue;

            ImPlot::SetupAxes(axlab[ia], axlab[ib],
                              ImPlotAxisFlags_NoGridLines, ImPlotAxisFlags_NoGridLines);
            // 三张图用同一对称范围，横向纵向刻度一致，圆才不会被压成椭圆
            ImPlot::SetupAxisLimits(ImAxis_X1, -range, range, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -range, range, ImGuiCond_Always);
            ImPlot::SetupAxisFormat(ImAxis_X1, "%.0f");
            ImPlot::SetupAxisFormat(ImAxis_Y1, "%.0f");

            // 先画点云（小而半透明），再画两条轮廓线压在上面。
            // 顺序反过来的话，几千个采样点会糊成一个实心白盘，把轮廓完全盖住。
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 1.8f,
                                       colOf(IM_COL32(225, 225, 225, 90)), 0.0f,
                                       colOf(IM_COL32(225, 225, 225, 90)));
            ImPlot::PlotScatter("samples", sx.data(), sy.data(), (int)sx.size());

            if (hasFit && fit.fieldMag > 1e-6) {
                std::vector<double> ix, iy;
                const int M = 128;
                ix.reserve(M + 1); iy.reserve(M + 1);
                for (int i = 0; i <= M; ++i) {
                    const double t = 2.0 * kPi * i / M;
                    ix.push_back(fit.fieldMag * std::cos(t));
                    iy.push_back(fit.fieldMag * std::sin(t));
                }
                ImPlot::SetNextLineStyle(colOf(IM_COL32(90, 220, 120, 230)), 1.8f);
                ImPlot::PlotLine("ideal circle r=|B|", ix.data(), iy.data(), M + 1);
            }

            if (hasFit) {
                std::vector<double> ex, ey;
                if (MagEllipsoidCalib::projectedEllipse(fit, ia, ib, ex, ey)) {
                    ImPlot::SetNextLineStyle(colOf(IM_COL32(80, 200, 255, 250)), 2.4f);
                    ImPlot::PlotLine("fitted ellipsoid", ex.data(), ey.data(), (int)ex.size());
                }
            }

            ImPlot::EndPlot();
        }
        ImGui::EndTable();

        ImGui::TextDisabled("Green is centred on the origin. The gap between green and cyan IS the hard-iron error;");
        ImGui::TextDisabled("cyan being non-circular IS the soft-iron error. Both are what corrupt yaw.");
    }

    // ------------------------------------------------------------------
    // 3. 球面磁误差
    // ------------------------------------------------------------------

    /**
     * 球面磁误差 = |m| - 1。校准把椭球映射到单位球面，所以拟合得好的样本
     * 应该全部落在 0 上；残差曲线直接显示"还有多少磁误差没被校掉"。
     * 实时曲线偏离 0 则说明当前磁环境和校准时不一样了 —— 校准已失效。
     */
    void drawSphericalError(const MagEllipsoidCalib::Result& fit, bool hasFit,
                            const std::vector<std::array<float, 3>>& samples,
                            const std::vector<SensorData>& live) {
        ImGui::SeparatorText("3. Spherical error  (|m| - 1) x 100%");

        if (!hasFit) {
            ImGui::TextDisabled("No calibration yet - nothing to measure against.");
            return;
        }

        const double tol = 5.0;   // %

        if (!samples.empty()) {
            std::vector<double> xs(samples.size()), ys(samples.size());
            for (size_t i = 0; i < samples.size(); ++i) {
                xs[i] = (double)i;
                ys[i] = (MagEllipsoidCalib::mappedNorm(fit, samples[i][0], samples[i][1], samples[i][2]) - 1.0) * 100.0;
            }
            plotWithBand("Spherical error across the calibration sweep", "sample #", "%", xs, ys, tol,
                         IM_COL32(80, 200, 255, 235));
        }

        if (!live.empty()) {
            std::vector<double> xs(live.size()), ys(live.size());
            for (size_t i = 0; i < live.size(); ++i) {
                const SensorData& d = live[i];
                xs[i] = d.timestamp;
                const double n = std::sqrt((double)d.mag[0] * d.mag[0]
                                         + (double)d.mag[1] * d.mag[1]
                                         + (double)d.mag[2] * d.mag[2]);
                ys[i] = (n - 1.0) * 100.0;
            }
            plotWithBand("Live spherical error - drifting off 0 means the calibration is stale",
                         "Time (s)", "%", xs, ys, tol, IM_COL32(255, 170, 60, 235));
        }
    }

    void plotWithBand(const char* title, const char* xlab, const char* ylab,
                      const std::vector<double>& xs, const std::vector<double>& ys,
                      double tol, ImU32 color) {
        if (xs.empty()) return;
        if (!ImPlot::BeginPlot(title, ImVec2(-1, m_state.plotHeight))) return;

        ImPlot::SetupAxes(xlab, ylab);
        ImPlot::SetupAxisLimits(ImAxis_X1, xs.front(), xs.back(), ImGuiCond_Always);

        double yMin = *std::min_element(ys.begin(), ys.end());
        double yMax = *std::max_element(ys.begin(), ys.end());
        yMin = std::min(yMin, -tol);
        yMax = std::max(yMax, tol);
        const double pad = (yMax - yMin) * 0.12 + 1e-6;
        ImPlot::SetupAxisLimits(ImAxis_Y1, yMin - pad, yMax + pad, ImGuiCond_Always);
        ImPlot::SetupAxisFormat(ImAxis_Y1, "%.1f");

        const int n = (int)xs.size();
        // 三条水平参考线每帧都画，用成员缓冲复用容量，避免 60fps 下反复分配
        m_bandHi.assign((size_t)n, tol);
        m_bandLo.assign((size_t)n, -tol);
        m_bandZero.assign((size_t)n, 0.0);
        ImPlot::SetNextLineStyle(colOf(IM_COL32(90, 220, 120, 110)), 1.0f);
        ImPlot::PlotLine("##tolHi", xs.data(), m_bandHi.data(), n);
        ImPlot::SetNextLineStyle(colOf(IM_COL32(90, 220, 120, 110)), 1.0f);
        ImPlot::PlotLine("##tolLo", xs.data(), m_bandLo.data(), n);
        ImPlot::SetNextLineStyle(colOf(IM_COL32(180, 180, 180, 90)), 1.0f);
        ImPlot::PlotLine("##zero", xs.data(), m_bandZero.data(), n);
        ImPlot::SetNextLineStyle(colOf(color), 1.8f);
        ImPlot::PlotLine("error", xs.data(), ys.data(), n);

        ImPlot::EndPlot();
    }

    // ------------------------------------------------------------------
    // 4. 磁影响度量
    // ------------------------------------------------------------------

    void drawMagImpact(const MagEllipsoidCalib::Result& fit, bool hasFit,
                       const std::vector<std::array<float, 3>>& samples,
                       const std::vector<SensorData>& live) {
        ImGui::SeparatorText("4. Magnetic impact metrics");

        // ---- 实时量：单位球面残差、|b| 与 |bRaw| 的恒定性、磁倾角 ----
        double liveMean = 0, liveSd = 0, bMean = 0, bSd = 0, bRawMean = 0, bRawSd = 0;
        double bvSum = 0; int nv = 0;
        // |b| 是否恒定只有在传感器转过足够角度时才有意义：静止时任何固件都"恒定"。
        double rotDeg = 0;
        double bMin = 1e30, bMax = -1e30;
        int bSame = 0, bCmp = 0;
        std::vector<double> vn, vb, vbr;
        for (size_t i = 0; i < live.size(); ++i) {
            const SensorData& d = live[i];
            vn.push_back(std::sqrt((double)d.mag[0] * d.mag[0] + (double)d.mag[1] * d.mag[1]
                                 + (double)d.mag[2] * d.mag[2]));
            const double nb = std::sqrt((double)d.magB[0] * d.magB[0] + (double)d.magB[1] * d.magB[1]
                                 + (double)d.magB[2] * d.magB[2]);
            const double nbr = std::sqrt((double)d.magBRaw[0] * d.magBRaw[0] + (double)d.magBRaw[1] * d.magBRaw[1]
                                  + (double)d.magBRaw[2] * d.magBRaw[2]);
            vb.push_back(nb);
            vbr.push_back(nbr);
            bMin = std::min(bMin, nb);
            bMax = std::max(bMax, nb);

            // b 与 bRaw 逐样本比对：全等就说明固件没有对磁通道做任何修正
            if (nbr > 1e-6) {
                bCmp++;
                if (std::fabs(nb - nbr) <= 0.005 * nbr) bSame++;
            }

            if (i > 0) {
                const double dt = d.timestamp - live[i - 1].timestamp;
                if (dt > 0 && dt < 0.1) {
                    rotDeg += std::sqrt((double)d.gyro[0] * d.gyro[0] + (double)d.gyro[1] * d.gyro[1]
                                        + (double)d.gyro[2] * d.gyro[2]) * dt;
                }
            }

            // 只在接近静止时取倾角：运动加速度会污染"向下"方向
            const double an = std::sqrt((double)d.acc[0] * d.acc[0] + (double)d.acc[1] * d.acc[1]
                                      + (double)d.acc[2] * d.acc[2]);
            if (an > 0.85 && an < 1.15 && vn.back() > 1e-6) {
                const double g[3] = { -d.acc[0] / an, -d.acc[1] / an, -d.acc[2] / an };
                bvSum += (d.mag[0] * g[0] + d.mag[1] * g[1] + d.mag[2] * g[2]) / vn.back();
                nv++;
            }
        }
        meanSd(vn, liveMean, liveSd);
        meanSd(vb, bMean, bSd);
        meanSd(vbr, bRawMean, bRawSd);
        if (bMax < bMin) { bMin = 0; bMax = 0; }

        const double bv = nv > 0 ? bvSum / nv : 0.0;
        const double bh = std::sqrt(std::max(0.0, 1.0 - bv * bv));
        const double inclDeg = std::atan2(bv, bh) * 180.0 / kPi;
        const double Bh_uT = hasFit ? fit.fieldMag * bh : 0.0;
        // 水平分量越小，同样的残余硬铁造成的 yaw 误差越大
        const double degPerUt = Bh_uT > 1e-6 ? (180.0 / kPi) / Bh_uT : 0.0;

        // ---- 采集覆盖度 ----
        double center[3] = { 0, 0, 0 };
        if (hasFit) {
            for (int i = 0; i < 3; ++i) center[i] = fit.center[i];
        } else if (!samples.empty()) {
            for (const auto& s : samples) for (int i = 0; i < 3; ++i) center[i] += s[i];
            for (int i = 0; i < 3; ++i) center[i] /= (double)samples.size();
        }
        int oct[8] = {};
        const int filled = samples.empty() ? 0 : MagEllipsoidCalib::octantCounts(samples, center, oct);
        double elMin = 1e9, elMax = -1e9;
        for (const auto& s : samples) {
            const double dx = s[0] - center[0], dy = s[1] - center[1], dz = s[2] - center[2];
            const double nn = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (nn < 1e-9) continue;
            const double el = std::asin(std::max(-1.0, std::min(1.0, dz / nn))) * 180.0 / kPi;
            elMin = std::min(elMin, el); elMax = std::max(elMax, el);
        }

        // ---- 拟合残差 ----
        double rmsPct = 0, maxPct = 0;
        if (hasFit && !samples.empty()) {
            double sum = 0;
            for (const auto& s : samples) {
                const double e = std::fabs(MagEllipsoidCalib::mappedNorm(fit, s[0], s[1], s[2]) - 1.0) * 100.0;
                sum += e * e;
                maxPct = std::max(maxPct, e);
            }
            rmsPct = std::sqrt(sum / (double)samples.size());
        }

        if (ImGui::BeginTable("##magmetrics", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable)) {
            ImGui::TableSetupColumn("##k", ImGuiTableColumnFlags_WidthFixed, 300.0f);
            ImGui::TableSetupColumn("##v");

            auto row = [](const char* k, const char* fmt, ...) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(k);
                ImGui::TableSetColumnIndex(1);
                char buf[256];
                va_list args;
                va_start(args, fmt);
                std::vsnprintf(buf, sizeof(buf), fmt, args);
                va_end(args);
                ImGui::TextUnformatted(buf);
            };

            row("Samples collected", "%zu", samples.size());
            row("Octants visited", "%d / 8   (%s)", filled, filled >= 7 ? "adequate" : "PARTIAL - fit under-constrained");
            if (!samples.empty())
                row("Elevation span", "[%.0f, %.0f] deg   (%s)", elMin, elMax,
                    (elMax - elMin) > 140.0 ? "adequate" : "PARTIAL");
            row("Coverage metric", "%.0f %%", fit.coverage * 100.0);
            ImGui::EndTable();
        }

        ImGui::Spacing();
        if (ImGui::BeginTable("##magmetrics2", 2, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable)) {
            ImGui::TableSetupColumn("##k2", ImGuiTableColumnFlags_WidthFixed, 300.0f);
            ImGui::TableSetupColumn("##v2");

            auto row = [](const char* k, const char* fmt, ...) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(k);
                ImGui::TableSetColumnIndex(1);
                char buf[256];
                va_list args;
                va_start(args, fmt);
                std::vsnprintf(buf, sizeof(buf), fmt, args);
                va_end(args);
                ImGui::TextUnformatted(buf);
            };

            row("Hard iron |center|", "%.2f uT   (%.2f, %.2f, %.2f)", fit.hardIron,
                fit.center[0], fit.center[1], fit.center[2]);
            row("Local field |B|", "%.2f uT", fit.fieldMag);
            row("Hard iron / field", "%.2f x   (%s)", fit.fieldMag > 1e-6 ? fit.hardIron / fit.fieldMag : 0.0,
                (fit.fieldMag > 1e-6 && fit.hardIron / fit.fieldMag > 0.6)
                    ? "BODY-fixed: something magnetized moves with the sensor"
                    : "plausible sensor offset");
            row("Semi-axes", "%.2f / %.2f / %.2f uT", fit.semiAxis[0], fit.semiAxis[1], fit.semiAxis[2]);
            row("Anisotropy (max/min)", "%.3f   (%s)", fit.anisotropy,
                fit.anisotropy < 1.15 ? "near-spherical, soft iron small" : "elongated, soft iron significant");
            row("Sphericity rms / max", "%.2f %% / %.2f %%", rmsPct, maxPct);
            row("Inclination (live)", "%.1f deg   (%d static frames)", inclDeg, nv);
            row("Horizontal field Bh", "%.2f uT", Bh_uT);
            row("Yaw sensitivity", "%.2f deg per uT of residual", degPerUt);
            ImGui::EndTable();
        }

        if (nv >= 20 && inclDeg < -5.0) {
            ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.3f, 1.0f),
                "Inclination is NEGATIVE (%.1f deg). In the northern hemisphere the field dips DOWN,",
                inclDeg);
            ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.3f, 1.0f),
                "so +40..+60 deg is expected. A negative value means the mag Z axis disagrees with the");
            ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.3f, 1.0f),
                "accelerometer's \"down\" (sign flip or axis swap between the two chips).");
            ImGui::TextWrapped(
                "A level yaw is unaffected, because tilt compensation only rotates the horizontal "
                "components; but every tilted pose gets a corrupted yaw. Confirm it in 10 s: hold the "
                "sensor level and note mx, my, mz, then stand it on its end (rotate 90 deg about the "
                "axis that was pointing east). If mz flips sign while the accel says you are still "
                "upright, the frames are inconsistent. Otherwise you are simply in the southern "
                "hemisphere and this is normal.");
        }

        // ---- 用观测到的 yaw 差反推需要多大的残余硬铁 ----
        ImGui::Spacing();
        ImGui::SetNextItemWidth(220);
        ImGui::SliderFloat("Observed host-vs-firmware yaw gap", &m_state.magYawGap, 0.0f, 180.0f, "%.0f deg");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Type in the yaw difference you actually see between the three host\n"
                              "algorithms and the sensor firmware. This converts it into the residual\n"
                              "magnetic offset that would explain it, via gap = atan(delta / Bh).\n"
                              "The same relation holds whether the residual is body-fixed (a\n"
                              "mis-calibration) or lab-fixed (environmental magnetism).");
        }
        ImGui::SameLine();
        if (Bh_uT > 1e-6) {
            const double need = Bh_uT * std::tan(std::fabs(m_state.magYawGap) * kPi / 180.0);
            ImGui::Text("=> needs %.1f uT of residual offset (body- or lab-fixed)", need);
            if (std::fabs(m_state.magYawGap) > 90.0f) {
                ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.3f, 1.0f),
                    "A gap above ~90 deg cannot be explained by a residual offset alone -");
                ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.3f, 1.0f),
                    "check the reference frame / firmware yaw zero / yaw sign instead.");
            }
        } else {
            ImGui::TextDisabled("Hold the sensor level and still to measure Bh.");
        }

        // ---- 判据 ----
        ImGui::Spacing();
        ImGui::SeparatorText("How to tell magnetometer error from a reference-frame offset");
        ImGui::TextWrapped(
            "Two different leftovers exist and they look DIFFERENT. Rotate the sensor slowly "
            "through a full 360 deg while keeping it LEVEL and watch (host yaw - firmware yaw) "
            "in the Euler Compare window.\n"
            "\n"
            "  1. Error SWINGS, crossing zero twice per revolution (zero at two headings 180 deg "
            "apart, opposite peaks in between)  ->  BODY-fixed residual hard iron. Something "
            "magnetized travels with the sensor, or the fit is wrong. Re-calibrate.\n"
            "\n"
            "  2. Error is CONSTANT at every heading  ->  either (a) a LAB-fixed environmental "
            "field perpendicular to the magnetic meridian, or (b) a reference-frame difference "
            "(magnetic declination, firmware yaw zeroed at power-on, opposite yaw sign). "
            "A tumble calibration CANNOT remove (a): while you tumble, a lab-fixed vector "
            "averages to zero, so it never enters the fitted centre. Magnitude-wise both give "
            "gap = atan(delta / Bh), so ~20 uT of environmental offset is a ~35 deg constant bias.\n"
            "\n"
            "Separate (a) from (b) with the TRANSLATION TEST: keep the sensor level and DO NOT "
            "rotate it, then physically move it 1-2 m to another spot in the room.\n"
            "  - yaw CHANGES  ->  environmental magnetism, case (a). Move away from steel, the PC, "
            "speakers and motors, then re-calibrate in the place you actually measure in.\n"
            "  - yaw does NOT change  ->  case (b), a reference-frame difference. Compare against "
            "declination and check the firmware's yaw zero / sign convention.");

        // ---- 实时健康检查 ----
        ImGui::Spacing();
        ImGui::SeparatorText("Live health");
        if (live.empty()) {
            ImGui::TextDisabled("Waiting for sensor data...");
            return;
        }

        if (!hasFit) {
            ImGui::TextDisabled("No calibration loaded yet - live |m| would just be the raw field in uT.");
        } else {
            const bool stale = std::fabs(liveMean - 1.0) > 0.15 || liveSd > 0.15;
            ImGui::Text("Live |m| = %.3f +/- %.3f  (1.000 == current field matches the stored calibration)",
                        liveMean, liveSd);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("|m| is the norm of the calibrated mag vector, which the ellipsoid\n"
                                  "fit maps onto the unit sphere. Away from 1.000 means the field the\n"
                                  "sensor sees now is not the field the fit was made in.");
            }
            if (stale) {
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f),
                    "CALIBRATION LOOKS STALE - the magnetic environment changed since the fit.");
                ImGui::TextWrapped(
                    "Important: re-calibrating fixes only the BODY-fixed part (something magnetized "
                    "that moves with the sensor). It does NOT remove a LAB-fixed environmental "
                    "offset, because while you tumble, a lab-fixed vector averages to zero and never "
                    "enters the fitted centre - it survives re-calibration and bends the heading by a "
                    "constant atan(O_perp / Bh). For that part the only fix is to physically remove "
                    "the disturbance: get away from steel furniture, the PC, speakers, motors and "
                    "magnetic mounts, then calibrate AND measure in that clean spot.");
            } else {
                ImGui::TextColored(ImVec4(0.45f, 0.9f, 0.5f, 1.0f), "OK - calibration still matches the environment.");
            }
        }

        // 固件到底有没有做磁校准：最硬的证据是 b 与 bRaw 逐样本全等（不需要运动），
        // 其次是转起来之后 |b| 仍然不恒定。静止时 |b| 恒定说明不了任何事。
        if (bMean > 1e-6) {
            ImGui::Text("Streamed |b| = %.2f uT, spread %.2f..%.2f (%.1f%% peak-to-peak)",
                        bMean, bMin, bMax, bMean > 1e-9 ? 100.0 * (bMax - bMin) / bMean : 0.0);
            if (bRawMean > 1e-6)
                ImGui::Text("Streamed |bRaw| = %.2f +/- %.2f uT   (%.1f%% spread)", bRawMean, bRawSd,
                            100.0 * bRawSd / bRawMean);
            else
                ImGui::TextDisabled("|bRaw| is all zero - this firmware does not stream the raw mag channel.");
            ImGui::Text("Rotation covered by this %.1f s window: %.0f deg",
                        live.size() > 1 ? (live.back().timestamp - live.front().timestamp) : 0.0, rotDeg);

            const bool identical = bCmp > 0 && bSame == bCmp;
            if (identical) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.35f, 1.0f),
                    "b == bRaw on all %d samples: the firmware applies NO magnetometer correction.", bCmp);
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.35f, 1.0f),
                    "Its yaw is fused from the same distorted field you are fitting -");
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.35f, 1.0f),
                    "treat it as a reference to compare against, not as ground truth.");
            } else if (bCmp > 0) {
                ImGui::Text("b differs from bRaw on %d/%d samples, so some firmware correction is active.",
                            bCmp - bSame, bCmp);
            }

            if (!identical && rotDeg < 30.0) {
                ImGui::TextDisabled(
                    "Sensor has not rotated enough in this window to judge |b| constancy - "
                    "tilt and turn it through ~90 deg, then read this again.");
            } else if (!identical && (bMax - bMin) / bMean > 0.05) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.35f, 1.0f),
                    "|b| changes with orientation, so ZenImuData::b is effectively uncalibrated on this unit.");
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.35f, 1.0f),
                    "The firmware yaw is therefore fused from the same distorted field -");
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.35f, 1.0f),
                    "treat it as a reference to compare against, not as ground truth.");
            }
        }

        const long long ct = m_sensorThread.magCalibTime();
        if (ct > 0) {
            const long long now = (long long)std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            const double hours = (double)(now - ct) / 3600.0;
            char when[64] = { 0 };
            const std::time_t tt = (std::time_t)ct;
            std::tm tmv;
            localtime_s(&tmv, &tt);
            std::strftime(when, sizeof(when), "%Y-%m-%d %H:%M", &tmv);
            ImGui::Text("Calibrated at %s  (%.1f h ago)", when, hours);
            if (hours > 24.0) {
                ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.35f, 1.0f),
                    "Older than a day - a large hard-iron term is usually environmental and drifts.");
            }
        } else if (hasFit) {
            ImGui::TextDisabled("Calibration loaded from an older mag_calib.txt with no timestamp.");
        }
    }

    bool startRecording() {
        bool success = m_sensorThread.startRecording();
        if (success) {
            m_state.isRecording = true;
            m_state.recordFilePath = m_sensorThread.recordFilePath();
        }
        return success;
    }

    void stopRecording() {
        m_state.isRecording = false;
        std::string filePath;
        m_sensorThread.stopRecording(m_samplingRate, filePath, m_pcDuration, m_sensorDuration,
                                     m_expectedPcCount, m_expectedSensorCount, m_actualCount, m_diffRatio);
        m_state.recordFilePath = filePath;
        m_state.recordCount = m_actualCount;
    }

    uint64_t m_expectedPcCount = 0;
    uint64_t m_expectedSensorCount = 0;
    uint64_t m_actualCount = 0;
    double m_pcDuration = 0.0;
    double m_sensorDuration = 0.0;
    double m_diffRatio = 0.0;

    DataBuffer& m_buffer;
    SensorThread& m_sensorThread;
    GuiState m_state;
    int32_t m_samplingRate;

    // 球面误差图里 ±容差带和零线的复用缓冲（每帧重画，内容恒定）
    std::vector<double> m_bandHi, m_bandLo, m_bandZero;

    /// "Reload from disk" 按钮的结果提示
    std::string m_reloadMsg = "mag_calib.txt is read at connect time.";

    // Field Survey 窗口状态（网格扫描读数工具）
    int m_surveyWindowSec = 2;                              ///< 统计窗口长度 (s)
    char m_surveyLabel[40] = "A1";                          ///< 格点标签输入框
    std::vector<SensorThread::SurveyMark> m_surveyHistory;  ///< 本会话已 Mark 的记录（供 Δ 表显示）

    // Field Map 窗口状态
    int m_mapRefSel = 0;        ///< 参考格下标（Combo 选择）
    int m_mapActiveSel = 0;     ///< 活动格下标（Combo 选择）
    std::string m_mapMsg = "build from field_survey.txt after marking cells";
};
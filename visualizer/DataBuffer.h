#pragma once

#include "ZenTypes.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <vector>

/**
 * @file DataBuffer.h
 * @brief    GUI 
 *
 * ============================================================
 * 
 * ============================================================
 *
 * 
 *   - SensorThread ()  ~100Hz  IMU 
 *   - GuiRenderer  ()  ~60Hz 
 *
 * 
 *   - 
 *   -  N 
 *
 *  (Ring Buffer / Circular Buffer) 
 *   - O(1)  push  pop
 *   - 
 *   - FIFO
 *
 *  std::mutex + std::condition_variable 
 */

/**
 * @struct SensorData
 * @brief  IMU  Madgwick 
 *
 * 
 *   :  euler[], quat[], linAcc[]
 *   Madgwick :   fusedEuler[], fusedQuat[], fusedLinAcc[]
 *
 *  GUI 
 */
struct SensorData {
    double timestamp;          ///<  ()

    // ---  ---
    float acc[3];              ///<  (g) [X, Y, Z]
    float gyro[3];             ///<  (deg/s) [X, Y, Z]

    /// 椭球校准后的磁向量，已映射到单位球面（|mag| ≈ 1），这是喂给三个融合滤波器的输入。
    /// 注意：它不是 µT，各分量量级在 ±1 附近，不要当作磁感应强度显示。
    float mag[3];

    /// ZenImuData::b —— 固件"已修正"磁通道 (µT)。椭球拟合采集的就是这一路。
    /// 若 |magB| 明显不恒定，说明固件端并未真正做磁校准。
    float magB[3] = { 0, 0, 0 };

    /// ZenImuData::bRaw —— 固件修正前的原始磁通道 (µT)。
    /// 传感器不支持流式输出时全为 0，用它可以判断 b 与 bRaw 是否真的不同。
    float magBRaw[3] = { 0, 0, 0 };

    // ---  ---
    float euler[3];            ///<  () [Roll, Pitch, Yaw]
    float quat[4];             ///<  [qw, qx, qy, qz]
    float linAcc[3];           ///<  (g)

    // ---  ---
    float pressure;            ///<  (mPa / )
    float temperature;         ///<  (C)

    // --- Madgwick  ---
    float fusedEuler[3];       ///<  () [Roll, Pitch, Yaw]
    float fusedQuat[4];        ///<  [qw, qx, qy, qz]
    float fusedLinAcc[3];      ///<  (g)

    // ---  (Complementary Filter)  ---
    float compEuler[3];        ///<  () [Roll, Pitch, Yaw]
    float compQuat[4];         ///<  [qw, qx, qy, qz]
    float compLinAcc[3];       ///<  (g)

    // --- EKF ()  ---
    float ekfEuler[3] = { 0, 0, 0 };   ///< EKF  () [Roll, Pitch, Yaw]
    float ekfQuat[4] = { 1, 0, 0, 0 }; ///< EKF  [qw, qx, qy, qz]
    float ekfLinAcc[3] = { 0, 0, 0 };  ///< EKF  (g)
};

/**
 * @class DataBuffer
 * @brief 
 *
 * 
 *
 *   [ _ | _ | D3 | D4 | D5 | D6 | _ | _ ]   = 8
 *            tail         head
 *               
 *
 *   - head: 
 *   - tail: 
 *   - count:  (0  count  capacity)
 *
 *    push  head tail   
 *    pop  
 */
class DataBuffer {
public:
    /**
     * @param capacity  capacity 
     *                  10000  100Hz  100 
     */
    explicit DataBuffer(size_t capacity)
        : m_buffer(capacity)
        , m_capacity(capacity)
        , m_head(0)        //  0
        , m_tail(0)        //  0
        , m_count(0)       // 
        , m_running(true)
    {}

    ~DataBuffer() {
        stop();
    }

    ///  pop() 
    void stop() {
        m_running.store(false);
        m_notEmpty.notify_all();
    }

    /**
     * @brief 
     *
     * 
     *   -    head count++
     *   -   tail count 
     *
     * @return true=, false=
     */
    bool push(const SensorData& data) {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (!m_running.load()) return false;

        m_buffer[m_head] = data;
        m_head = (m_head + 1) % m_capacity;  // 

        if (m_count < m_capacity) {
            m_count++;  // 
        } else {
            m_tail = (m_tail + 1) % m_capacity;  // tail  head 
        }

        m_notEmpty.notify_one();  // 
        return true;
    }

    /**
     * @brief 
     *
     * 
     *   -    true
     *   -   
     *
     * @return true=, false=
     */
    bool pop(SensorData& out) {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_notEmpty.wait(lock, [this] {
            return !m_running.load() || m_count > 0;
        });

        if (!m_running.load() && m_count == 0) return false;

        out = m_buffer[m_tail];
        m_tail = (m_tail + 1) % m_capacity;
        m_count--;

        return true;
    }

    /// 
    size_t size() const {
        return m_count.load();
    }

    bool isRunning() const {
        return m_running.load();
    }

    /**
     * @brief  N  GUI 
     *
     *  pop() fillLatest() """"
     *   
     *
     * @param out        resize 
     * @param maxCount   count
     *
     * 
     */
    void fillLatest(std::vector<SensorData>& out, size_t maxCount) {
        std::unique_lock<std::mutex> lock(m_mutex);
        size_t count = std::min(maxCount, m_count.load());
        //  tail  count 
        size_t start = (m_tail + m_count - count) % m_capacity;

        out.resize(count);
        for (size_t i = 0; i < count; i++) {
            out[i] = m_buffer[(start + i) % m_capacity];
        }
    }

private:
    std::vector<SensorData> m_buffer;  ///< 
    const size_t m_capacity;           ///< 

    size_t m_head;                     ///< 
    size_t m_tail;                     ///< 
    std::atomic<size_t> m_count;       ///<  ()

    // 
    std::atomic<bool> m_running;                  ///< 
    std::mutex m_mutex;                            ///<  head/tail/count 
    std::condition_variable m_notEmpty;            ///< ""
};

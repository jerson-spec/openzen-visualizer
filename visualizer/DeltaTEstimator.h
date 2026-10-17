#pragma once

#include <algorithm>
#include <chrono>

/**
 * @class DeltaTEstimator
 * @brief  IMU  (dt)
 *
 * 
 *
 *   OpenZen SDK  imuData.timestamp 
 *   frameCount  legacy  0.0025 / IG1 0.002
 *   
 *
 *    frameCount 
 *   "" 200~500Hz 
 *   frameCount PC 
 *
 *     dt =  (EMA)
 *
 *   
 *   - EMA 
 *   - BLE 0EMA 
 *      =
 *        = 
 *   - 
 *      dt 
 *
 *   dt 
 *   
 *
 *  steady_clock
 */
class DeltaTEstimator {
public:
    /**
     * @param nominalRate   (Hz) EMA  fallback
     */
    explicit DeltaTEstimator(float nominalRate = 100.0f)
        : m_nominalDt(nominalRate > 0.0f ? 1.0f / nominalRate : 0.01f)
    {}

    void reset() {
        m_hasPrev = false;
        m_emaSeeded = false;
        m_emaDt = m_nominalDt;
    }

    /**
     * @brief  IMU 
     * @param nowSeconds 
     * @return 
     */
    float update(double nowSeconds) {
        if (!m_hasPrev) {
            m_hasPrev = true;
            m_prevWall = nowSeconds;
            return static_cast<float>(m_emaDt);
        }

        double wallDt = nowSeconds - m_prevWall;
        m_prevWall = nowSeconds;

        if (wallDt < 0.0) {
            wallDt = 0.0;  // 
        } else if (wallDt > 0.5) {
            // / EMA
            // 
            wallDt = m_emaDt;
        }

        if (!m_emaSeeded) {
            //  EMA 
            //  dt 25Hz  0.8s  10ms 
            m_emaSeeded = true;
            m_emaDt = (wallDt > 0.0) ? wallDt : m_nominalDt;
        } else {
            // EMA  1/alpha alpha=0.05  ~20   0.2s@100Hz
            m_emaDt += EMA_ALPHA * (wallDt - m_emaDt);
        }

        return clampDt(static_cast<float>(m_emaDt));
    }

    /**
     * @brief  (Hz)
     *
     * /
     */
    double eventRate() const {
        if (m_emaDt <= 0.0) return 0.0;
        return 1.0 / m_emaDt;
    }

private:
    static float clampDt(float dt) {
        //  dt 5kHz  5Hz 
        if (dt < 0.0002f) return 0.0002f;
        if (dt > 0.2f) return 0.2f;
        return dt;
    }

    static constexpr double EMA_ALPHA = 0.05;  ///< EMA 

    float m_nominalDt;
    bool m_hasPrev = false;
    bool m_emaSeeded = false;
    double m_prevWall = 0.0;
    double m_emaDt = 0.01;
};

/**
 * @brief  steady_clock  DeltaTEstimator 
 */
struct WallClockSource {
    double operator()() const {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
};

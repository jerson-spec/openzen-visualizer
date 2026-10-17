#pragma once

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @class MadgwickFilter
 * @brief  Madgwick 9 AHRS 
 * 
 * 
 * 
 * Madgwick, S.O.H. (2011). "An efficient orientation filter 
 *       for inertial and inertial/magnetic sensor arrays"
 * 
 * 
 * 1.  _2q0mx, _2q0my, _2q0mz, _2q1mx
 * 2. _2bx, _2bz  2
 * 3. 
 * 4.  6 
 * 5. 
 * 6.  invSqrt  64  memcpy 
 * 7.  9 6
 * 8. +
 *    yaw=0 120  14s180  60~120s
 * 9.  INIT_BOOST_SECONDS  3  beta
 * 10.  setDeltaT() 1/sampleRate 
 * 11. invSqrt  1.0f/sqrtf()Quake  ~0.2% PC 
 * 
 *  API
 *   - MadgwickFilter(float sampleRate, float beta = 0.1f)
 *   - void reset()
 *   - void setBeta(float beta)
 *   - void update(float ax, ay, az, gx, gy, gz, mx, my, mz)  [9]
 *   - void getQuaternion(float& q0, float& q1, float& q2, float& q3) const
 *   - void getEulerAngles(float& roll, float& pitch, float& yaw) const
 *   - void getLinearAcceleration(float ax, ay, az, float&, float&, float&) const   
 */
class MadgwickFilter {
public:
    /**
     * @param sampleRate 
     * @param beta       
     *                   beta 
     *                   beta 
     */
    MadgwickFilter(float sampleRate, float beta = 0.1f)
        : m_sampleRate(sampleRate)
        , m_beta(beta)
        , m_q0(1.0f)
        , m_q1(0.0f)
        , m_q2(0.0f)
        , m_q3(0.0f)
    {}

    void reset() {
        m_q0 = 1.0f;
        m_q1 = 0.0f;
        m_q2 = 0.0f;
        m_q3 = 0.0f;
        m_initialized = false;      // 
        m_elapsed = 0.0f;           // 
        m_dtOverride = -1.0f;       //  1/sampleRate
    }

    void setBeta(float beta) {
        m_beta = beta;
    }

    /**
     * @brief  1/sampleRate 
     *
     *  0.01s@100Hz
     * 0.02s@50Hz [0.0002, 0.1]
     *  1/sampleRate
     */
    void setDeltaT(float dt) {
        if (dt >= 0.0002f && dt <= 0.1f) {
            m_dtOverride = dt;
        }
    }

    /**
     * @brief 9 +  + 
     * @param ax, ay, az   (g)
     * @param gx, gy, gz   (deg/s)
     * @param mx, my, mz   (T)
     */
    void update(float ax, float ay, float az, 
                float gx, float gy, float gz, 
                float mx, float my, float mz) {
        update9DOF(ax, ay, az, gx, gy, gz, mx, my, mz);
    }

    /**
     * @brief 9
     */
    void update9DOF(float ax, float ay, float az, 
                    float gx, float gy, float gz, 
                    float mx, float my, float mz) {
        float q0 = m_q0, q1 = m_q1, q2 = m_q2, q3 = m_q3;
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
        float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
        float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

        // /
        const float DEG_TO_RAD = static_cast<float>(M_PI) / 180.0f;
        gx *= DEG_TO_RAD;
        gy *= DEG_TO_RAD;
        gz *= DEG_TO_RAD;

        // 
        float aNorm = ax * ax + ay * ay + az * az;
        if (aNorm < 1e-10f) {
            // 
            integrateGyroOnly(gx, gy, gz);
            return;
        }
        recipNorm = invSqrt(aNorm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        float mNorm = mx * mx + my * my + mz * mz;
        if (mNorm < 1e-10f) {
            // 6
            // 
            update6DOFRad(ax, ay, az, gx, gy, gz);
            return;
        }
        recipNorm = invSqrt(mNorm);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        //  + 
        // yaw=0
        if (!m_initialized) {
            initializeFromSensors(ax, ay, az, mx, my, mz);
            m_initialized = true;
            m_elapsed += currentDt();
            return;  // 
        }

        // 
        const float dt = currentDt();
        m_elapsed += dt;

        //  3 
        const float beta = (m_elapsed < INIT_BOOST_SECONDS) ? (3.0f * m_beta) : m_beta;

        // 
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // 
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        
        _2bx = 2.0f * sqrtf(hx * hx + hy * hy);
        _2bz = 2.0f * (-_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3);
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // 
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) 
           - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
           + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
           + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) 
           - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) 
           + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
           + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
           + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) 
           - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) 
           + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
           + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
           + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
           + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        //  3 
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

        // 
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        m_q0 = q0;
        m_q1 = q1;
        m_q2 = q2;
        m_q3 = q3;
    }

    /**
     * @brief 6 + 
     *  / 
     */
    void update6DOF(float ax, float ay, float az, 
                    float gx, float gy, float gz) {
        const float DEG_TO_RAD = static_cast<float>(M_PI) / 180.0f;
        gx *= DEG_TO_RAD;
        gy *= DEG_TO_RAD;
        gz *= DEG_TO_RAD;
        
        update6DOFRad(ax, ay, az, gx, gy, gz);
    }

    /**
     * @brief 
     * @param q0 
     * @param q1  x
     * @param q2  y
     * @param q3  z
     */
    void getQuaternion(float& q0, float& q1, float& q2, float& q3) const {
        q0 = m_q0;
        q1 = m_q1;
        q2 = m_q2;
        q3 = m_q3;
    }

    /**
     * @brief  (ZYX Tait-Bryan )
     *
     * @param roll   (X)
     * @param pitch  (Y)
     * @param yaw    (Z)
     */
    void getEulerAngles(float& roll, float& pitch, float& yaw) const {
        const float RAD_TO_DEG = 180.0f / static_cast<float>(M_PI);
        
        roll = atan2f(2.0f * (m_q0 * m_q1 + m_q2 * m_q3), 
                      1.0f - 2.0f * (m_q1 * m_q1 + m_q2 * m_q2));
        pitch = asinf(2.0f * (m_q0 * m_q2 - m_q3 * m_q1));
        yaw = atan2f(2.0f * (m_q0 * m_q3 + m_q1 * m_q2), 
                     1.0f - 2.0f * (m_q2 * m_q2 + m_q3 * m_q3));

        roll *= RAD_TO_DEG;
        pitch *= RAD_TO_DEG;
        yaw *= RAD_TO_DEG;
    }

    /**
     * @brief 
     *
     * @param ax, ay, az  (g)
     * @param linAccX, linAccY, linAccZ  (g)
     *
     *  -  = 
     *   gravity_predicted = R * [0, 0, 1]
     */
    void getLinearAcceleration(float ax, float ay, float az,
                               float& linAccX, float& linAccY, float& linAccZ) const {
        float q0 = m_q0, q1 = m_q1, q2 = m_q2, q3 = m_q3;

        //  [0,0,1] 
        float gravityX = 2.0f * (q1 * q3 - q0 * q2);
        float gravityY = 2.0f * (q0 * q1 + q2 * q3);
        float gravityZ = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        //  =  - 
        linAccX = ax - gravityX;
        linAccY = ay - gravityY;
        linAccZ = az - gravityZ;
    }

private:
    /**
     * @brief 
     *
     * PC  Quake III ~0.2% 
     * 
     *
     * 1/sqrtf(0) = inf 0 0
     *  NaN 0  0
     * 
     */
    static float invSqrt(float x) {
        if (x < 1e-12f) return 0.0f;
        return 1.0f / std::sqrtf(x);
    }

    /**
     * @brief 
     * /
     */
    void integrateGyroOnly(float gx, float gy, float gz) {
        float q0 = m_q0, q1 = m_q1, q2 = m_q2, q3 = m_q3;

        // 
        float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // 
        const float dt = currentDt();
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        // 
        float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        m_q0 = q0;
        m_q1 = q1;
        m_q2 = q2;
        m_q3 = q3;
    }

    /**
     * @brief 6/
     */
    void update6DOFRad(float ax, float ay, float az, 
                       float gx, float gy, float gz) {
        float q0 = m_q0, q1 = m_q1, q2 = m_q2, q3 = m_q3;
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
        float q0q0, q1q1, q2q2, q3q3;

        // 
        float aNorm = ax * ax + ay * ay + az * az;
        if (aNorm < 1e-10f) {
            integrateGyroOnly(gx, gy, gz);
            return;
        }
        recipNorm = invSqrt(aNorm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //  Roll/PitchYaw  0
        if (!m_initialized) {
            initializeFromAccel(ax, ay, az);
            m_initialized = true;
            m_elapsed += currentDt();
            return;
        }

        const float dt = currentDt();
        m_elapsed += dt;
        const float beta = (m_elapsed < INIT_BOOST_SECONDS) ? (3.0f * m_beta) : m_beta;

        // 
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // 
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        //  3 
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

        // 
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        m_q0 = q0;
        m_q1 = q1;
        m_q2 = q2;
        m_q3 = q3;
    }

    static constexpr float INIT_BOOST_SECONDS = 5.0f;  //  3 

    /**
     * @brief 
     *
     *  setDeltaT() 
     *  1/sampleRate
     */
    float currentDt() const {
        return (m_dtOverride > 0.0f) ? m_dtOverride : (1.0f / m_sampleRate);
    }

    /**
     * @brief +9 
     *
     *  ax/ay/azmx/my/mz  ComplementaryFilter 
     * eulerquatZYX Tait-Bryan
     *  raw yaw 
     *  180  60~120s
     */
    void initializeFromSensors(float ax, float ay, float az,
                               float mx, float my, float mz) {
        float roll  = atan2f(ay, az);
        float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

        float cr = cosf(roll);
        float sr = sinf(roll);
        float cp = cosf(pitch);
        float sp = sinf(pitch);

        // 
        float mxh = mx * cp + my * sr * sp + mz * cr * sp;
        float myh = my * cr - mz * sr;
        float yaw = 0.0f;
        float magHNorm = sqrtf(mxh * mxh + myh * myh);
        if (magHNorm > 0.0f) {
            yaw = atan2f(-myh, mxh);
        }

        float cr2 = cosf(roll  * 0.5f);
        float sr2 = sinf(roll  * 0.5f);
        float cp2 = cosf(pitch * 0.5f);
        float sp2 = sinf(pitch * 0.5f);
        float cy2 = cosf(yaw   * 0.5f);
        float sy2 = sinf(yaw   * 0.5f);

        // ZYX Tait-Bryan : q = q_yaw  q_pitch  q_roll
        m_q0 = cy2 * cp2 * cr2 + sy2 * sp2 * sr2;
        m_q1 = cy2 * cp2 * sr2 - sy2 * sp2 * cr2;
        m_q2 = sy2 * cp2 * sr2 + cy2 * sp2 * cr2;
        m_q3 = sy2 * cp2 * cr2 - cy2 * sp2 * sr2;
    }

    /**
     * @brief  Roll/Pitch6 
     *
     *  ax/ay/az Yaw  0
     */
    void initializeFromAccel(float ax, float ay, float az) {
        float roll  = atan2f(ay, az);
        float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

        float cr2 = cosf(roll  * 0.5f);
        float sr2 = sinf(roll  * 0.5f);
        float cp2 = cosf(pitch * 0.5f);
        float sp2 = sinf(pitch * 0.5f);

        // yaw = 0cy2 = 1, sy2 = 0
        m_q0 = cp2 * cr2;
        m_q1 = cp2 * sr2;
        m_q2 = sp2 * cr2;
        m_q3 = -sp2 * sr2;
    }

    float m_sampleRate;
    float m_beta;
    float m_q0, m_q1, m_q2, m_q3;
    bool m_initialized = false;   ///< 
    float m_elapsed = 0.0f;       ///< 
    float m_dtOverride = -1.0f;   ///< -1 
};

#pragma once

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @class ComplementaryFilter
 * @brief 
 *
 * 
 *   - 
 *   - /
 *   - 
 *       attitude = alpha * (gyro_integrated) + (1 - alpha) * (accel_mag_corrected)
 *
 * 
 *   
 *     1.  q_gyro
 *     2.  +  q_correction
 *     3.  slerp () 
 *        q = slerp(q_correction, q_gyro, alpha)
 *
 *  MadgwickFilter 
 *   -  (ENU): X, Y, Z
 *   - : X, Y, Z
 *   - : q = [w, x, y, z]
 *   - : ZYX Tait-Bryan  [Roll, Pitch, Yaw]
 */
class ComplementaryFilter {
public:
    /**
     * @param sampleRate   (Hz)
     * @param alpha        (0.0 ~ 1.0)
     *                    alpha 
     *                    alpha /
     *                    0.98 ( 98%  + 2% /)
     */
    ComplementaryFilter(float sampleRate, float alpha = 0.98f)
        : m_sampleRate(sampleRate)
        , m_alpha(alpha)
        , m_q0(1.0f)  // 
        , m_q1(0.0f)  //  x
        , m_q2(0.0f)  //  y
        , m_q3(0.0f)  //  z
    {}

    /**
     * @brief 
     */
    void reset() {
        m_q0 = 1.0f;
        m_q1 = 0.0f;
        m_q2 = 0.0f;
        m_q3 = 0.0f;
        m_initialized = false;      // 
        m_dtOverride = -1.0f;       //  1/sampleRate
    }

    /**
     * @brief 
     * @param alpha  (0.0~1.0)
     */
    void setAlpha(float alpha) {
        m_alpha = alpha;
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
     * @brief /
     *
     *  6 roll/pitch yaw 
     * 
     *  90  yaw 
     * /
     */
    void setMagEnabled(bool enabled) {
        m_magEnabled = enabled;
    }

    /**
     * @brief 
     *
     * @param ax, ay, az   (g)
     * @param gx, gy, gz   (deg/s)
     * @param mx, my, mz   (T)
     *
     * 
     *   1. q_gyro = q  q_delta_gyro
     *   2.  Roll/Pitch 
     *   3.  Yaw 
     *   4.  q_correction
     *   5.  slerp  q_gyro  q_correction
     */
    void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
        // +
        // yaw=0
        if (!m_initialized) {
            initializeFromSensors(ax, ay, az, mx, my, mz);
            m_initialized = true;
            return;
        }

        float q0 = m_q0, q1 = m_q1, q2 = m_q2, q3 = m_q3;
        float dt = currentDt();

        // ============================================================
        // 
        // ============================================================
        // 
        float gxRad = gx * float(M_PI) / 180.0f;
        float gyRad = gy * float(M_PI) / 180.0f;
        float gzRad = gz * float(M_PI) / 180.0f;

        // q_dot = 0.5 * q  
        float qDot0 = 0.5f * (-q1 * gxRad - q2 * gyRad - q3 * gzRad);
        float qDot1 = 0.5f * ( q0 * gxRad + q2 * gzRad - q3 * gyRad);
        float qDot2 = 0.5f * ( q0 * gyRad - q1 * gzRad + q3 * gxRad);
        float qDot3 = 0.5f * ( q0 * gzRad + q1 * gyRad - q2 * gxRad);

        // 
        float gq0 = q0 + qDot0 * dt;
        float gq1 = q1 + qDot1 * dt;
        float gq2 = q2 + qDot2 * dt;
        float gq3 = q3 + qDot3 * dt;

        // 
        float gNorm = sqrt(gq0 * gq0 + gq1 * gq1 + gq2 * gq2 + gq3 * gq3);
        if (gNorm > 0.0f) {
            gq0 /= gNorm;
            gq1 /= gNorm;
            gq2 /= gNorm;
            gq3 /= gNorm;
        }

        // ============================================================
        //  + 
        // ============================================================

        // ---  Roll/Pitch---
        float accNorm = sqrt(ax * ax + ay * ay + az * az);
        float rollAcc = 0.0f;
        float pitchAcc = 0.0f;

        if (accNorm > 0.0f) {
            float axn = ax / accNorm;
            float ayn = ay / accNorm;
            float azn = az / accNorm;

            // 
            //  =  = [0, 0, -1]ENU
            //  az  +1g
            // roll  = atan2(ay, az)
            // pitch = atan2(-ax, sqrt(ay^2 + az^2))
            rollAcc  = atan2f(ayn, azn);
            pitchAcc = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));
        }

        // ---  Yaw---
        //  Roll/Pitch
        float magRoll  = rollAcc;
        float magPitch = pitchAcc;
        float cr = cosf(magRoll);
        float sr = sinf(magRoll);
        float cp = cosf(magPitch);
        float sp = sinf(magPitch);

        // 
        float mxh = mx * cp + my * sr * sp + mz * cr * sp;
        float myh = my * cr - mz * sr;

        float yawMag = 0.0f;
        float magHNorm = sqrt(mxh * mxh + myh * myh);
        if (magHNorm > 0.0f) {
            // 0
            yawMag = atan2f(-myh, mxh);
        }

        // yaw 
        //  yaw yaw  yaw 
        if (!m_magEnabled || magHNorm <= 0.0f) {
            yawMag = atan2f(2.0f * (gq0 * gq3 + gq1 * gq2),
                            1.0f - 2.0f * (gq2 * gq2 + gq3 * gq3));
        }

        // ---  ---
        float cr2 = cosf(rollAcc  * 0.5f);
        float sr2 = sinf(rollAcc  * 0.5f);
        float cp2 = cosf(pitchAcc * 0.5f);
        float sp2 = sinf(pitchAcc * 0.5f);
        float cy2 = cosf(yawMag   * 0.5f);
        float sy2 = sinf(yawMag   * 0.5f);

        // ZYX Tait-Bryan : q = q_yaw  q_pitch  q_roll
        float cq0 = cy2 * cp2 * cr2 + sy2 * sp2 * sr2;
        float cq1 = cy2 * cp2 * sr2 - sy2 * sp2 * cr2;
        float cq2 = sy2 * cp2 * sr2 + cy2 * sp2 * cr2;
        float cq3 = sy2 * cp2 * cr2 - cy2 * sp2 * sr2;

        // 
        float cNorm = sqrt(cq0 * cq0 + cq1 * cq1 + cq2 * cq2 + cq3 * cq3);
        if (cNorm > 0.0f) {
            cq0 /= cNorm;
            cq1 /= cNorm;
            cq2 /= cNorm;
            cq3 /= cNorm;
        }

        // ============================================================
        //  Slerp
        // ============================================================
        // q_final = slerp(q_correction, q_gyro, alpha)
        // alpha ~ 0.98 
        float fq0, fq1, fq2, fq3;
        slerp(cq0, cq1, cq2, cq3, gq0, gq1, gq2, gq3, m_alpha, fq0, fq1, fq2, fq3);

        // slerp 
        float fNorm = sqrt(fq0 * fq0 + fq1 * fq1 + fq2 * fq2 + fq3 * fq3);
        if (fNorm > 0.0f) {
            m_q0 = fq0 / fNorm;
            m_q1 = fq1 / fNorm;
            m_q2 = fq2 / fNorm;
            m_q3 = fq3 / fNorm;
        }
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
        float q0 = m_q0, q1 = m_q1, q2 = m_q2, q3 = m_q3;

        roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
        pitch = asinf (2.0f * (q0 * q2 - q3 * q1));
        yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

        roll  *= 180.0f / float(M_PI);
        pitch *= 180.0f / float(M_PI);
        yaw   *= 180.0f / float(M_PI);
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
     * @brief  (Spherical Linear Interpolation)
     *
     *  qa  qb  t 
     * t=0   qa, t=1   qb
     *
     * @param qa0..qa3 
     * @param qb0..qb3 
     * @param t         (0.0 ~ 1.0)
     * @param qr0..qr3 
     */
    void slerp(float qa0, float qa1, float qa2, float qa3,
               float qb0, float qb1, float qb2, float qb3,
               float t,
               float& qr0, float& qr1, float& qr2, float& qr3)
    {
        // 
        float cosHalfTheta = qa0 * qb0 + qa1 * qb1 + qa2 * qb2 + qa3 * qb3;

        // 
        if (cosHalfTheta < 0.0f) {
            qb0 = -qb0;
            qb1 = -qb1;
            qb2 = -qb2;
            qb3 = -qb3;
            cosHalfTheta = -cosHalfTheta;
        }

        // 
        if (cosHalfTheta > 0.9995f) {
            qr0 = qa0 + t * (qb0 - qa0);
            qr1 = qa1 + t * (qb1 - qa1);
            qr2 = qa2 + t * (qb2 - qa2);
            qr3 = qa3 + t * (qb3 - qa3);
            return;
        }

        // 
        float halfTheta    = acosf(cosHalfTheta);
        float sinHalfTheta = sqrtf(1.0f - cosHalfTheta * cosHalfTheta);

        float ratioA = sinf((1.0f - t) * halfTheta) / sinHalfTheta;
        float ratioB = sinf(t * halfTheta) / sinHalfTheta;

        qr0 = qa0 * ratioA + qb0 * ratioB;
        qr1 = qa1 * ratioA + qb1 * ratioB;
        qr2 = qa2 * ratioA + qb2 * ratioB;
        qr3 = qa3 * ratioA + qb3 * ratioB;
    }

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
     * @brief +
     *
     * IMU  update() 
     *  Roll/Pitch Yaw ZYX Tait-Bryan
     *  raw yaw 
     */
    void initializeFromSensors(float ax, float ay, float az, float mx, float my, float mz) {
        // ---   Roll/Pitch ---
        float accNorm = sqrtf(ax * ax + ay * ay + az * az);
        float roll = 0.0f;
        float pitch = 0.0f;
        if (accNorm > 0.0f) {
            float axn = ax / accNorm;
            float ayn = ay / accNorm;
            float azn = az / accNorm;
            roll  = atan2f(ayn, azn);
            pitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));
        }

        // ---   Yaw ---
        float cr = cosf(roll);
        float sr = sinf(roll);
        float cp = cosf(pitch);
        float sp = sinf(pitch);
        float mxh = mx * cp + my * sr * sp + mz * cr * sp;
        float myh = my * cr - mz * sr;
        float yaw = 0.0f;
        float magHNorm = sqrtf(mxh * mxh + myh * myh);
        if (magHNorm > 0.0f) {
            yaw = atan2f(-myh, mxh);
        }

        // --- ZYX Tait-Bryan : q = q_yaw  q_pitch  q_roll ---
        float cr2 = cosf(roll  * 0.5f);
        float sr2 = sinf(roll  * 0.5f);
        float cp2 = cosf(pitch * 0.5f);
        float sp2 = sinf(pitch * 0.5f);
        float cy2 = cosf(yaw   * 0.5f);
        float sy2 = sinf(yaw   * 0.5f);
        m_q0 = cy2 * cp2 * cr2 + sy2 * sp2 * sr2;
        m_q1 = cy2 * cp2 * sr2 - sy2 * sp2 * cr2;
        m_q2 = sy2 * cp2 * sr2 + cy2 * sp2 * cr2;
        m_q3 = sy2 * cp2 * cr2 - cy2 * sp2 * sr2;
    }

    float m_sampleRate;  //  (Hz)
    float m_alpha;       // 0.0~1.0
    float m_q0, m_q1, m_q2, m_q3;  // 
    bool m_initialized = false;    ///< 
    float m_dtOverride = -1.0f;    ///< -1 
    bool m_magEnabled = true;      ///<  yawfalse=6
};

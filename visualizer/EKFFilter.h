#pragma once

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @class EKFFilter
 * @brief  EKF MadgwickFilter/ComplementaryFilter
 *        
 *
 * 7 
 *    x = [q0,q1,q2,q3 | bx,by,bz]
 *   + rad/s
 *    x = [(3), b(3)]q_true  q  q()
 *
 * 
 *   q =  q  [0, _m  b] MadgwickFilter 
 *   P =  P  + Q  I + FdtF = [[[], I],[0,0]]
 *
 *  / 
 *    h(q) = R(q)d_world
 *    h/ = [h]h/b = 0
 *    y = z_norm  hK = P H(HPH+R)
 *   q  normalize(q  q())b += bP  Joseph 
 *   
 *
 * 
 *   sigmaGyro      (rad/s)
 *   sigmaAccel    
 *   sigmaMag      
 *   sigmaBiasWalk  (rad/s)
 *    100Hz  MEMS  beta=0.1 / alpha=0.98
 *
 * /
 *   ""SensorThread  LPMS 
 *    deg/s ZYX Tait-Bryan 180 =   
 *
 *  ~3 6x6/6x3 100Hz  CPU 
 */
class EKFFilter {
public:
    EKFFilter(float sampleRate, float sigmaGyro = 0.05f, float sigmaAccel = 0.08f,
              float sigmaMag = 0.15f, float sigmaBiasWalk = 0.001f)
        : m_sampleRate(sampleRate > 0.0f ? sampleRate : 100.0f)
        , m_sigmaGyro(sigmaGyro)
        , m_sigmaAccel(sigmaAccel)
        , m_sigmaMag(sigmaMag)
        , m_sigmaBiasWalk(sigmaBiasWalk)
    {
        reset();
    }

    void reset() {
        m_q0 = 1.0f; m_q1 = m_q2 = m_q3 = 0.0f;
        for (int i = 0; i < 3; ++i) m_bias[i] = 0.0f;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) m_P[i][j] = 0.0f;
        //  ~10 ~1/s
        for (int i = 0; i < 3; ++i) {
            m_P[i][i] = 0.03;           // (0.175 rad)
            m_P[3 + i][3 + i] = 3.0e-4; // (1/s)
        }
        m_initialized = false;
        m_dtOverride = -1.0f;
        m_magWorldSet = false;
    }

    void setDeltaT(float dt) {
        if (dt >= 0.0002f && dt <= 0.1f) m_dtOverride = dt;
    }

    /**
     * @brief 
     *        6  SensorThread 
     */
    void update(float ax, float ay, float az,
                float gx, float gy, float gz,
                float mx, float my, float mz) {
        const float DEG2RAD = (float)M_PI / 180.0f;
        const float wx = gx * DEG2RAD, wy = gy * DEG2RAD, wz = gz * DEG2RAD;

        if (!m_initialized) {
            initialize(ax, ay, az, mx, my, mz);
            m_initialized = true;
            return;
        }

        const float dt = currentDt();

        // ----------  ----------
        const float w0 = wx - m_bias[0], w1 = wy - m_bias[1], w2 = wz - m_bias[2];
        qDot(m_q0, m_q1, m_q2, m_q3, w0, w1, w2, dt);

        // ----------  ----------
        double F[6][6] = {};
        F[0][1] = -w2; F[0][2] = w1;
        F[1][0] = w2;  F[1][2] = -w0;
        F[2][0] = -w1; F[2][1] = w0;
        for (int i = 0; i < 3; ++i) F[i][3 + i] = -1.0;
        double Phi[6][6];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                double v = (i == j) ? 1.0 : 0.0;
                for (int k = 0; k < 6; ++k) v += F[i][k] * (double)dt;
                Phi[i][j] = v;
            }
        double P2[6][6];
        mat6mul(Phi, m_P, P2);
        double P3[6][6];
        mat6mulT2(P2, Phi, P3);  // P3 = PhiPPhi
        const double qg = (double)m_sigmaGyro * (double)m_sigmaGyro * (double)dt;
        const double qb = (double)m_sigmaBiasWalk * (double)m_sigmaBiasWalk * (double)dt;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) P3[i][j] += (i == j ? qg : 0.0);
            P3[3 + i][3 + i] += qb;
        }
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) m_P[i][j] = P3[i][j];

        // ---------- roll/pitch ----------
        updateAccel(ax, ay, az);

        // ---------- yaw = 6  ----------
        updateMag(mx, my, mz);
    }

    void getQuaternion(float& q0, float& q1, float& q2, float& q3) const {
        q0 = m_q0; q1 = m_q1; q2 = m_q2; q3 = m_q3;
    }

    void getEulerAngles(float& roll, float& pitch, float& yaw) const {
        const float R2D = 180.0f / (float)M_PI;
        roll = atan2f(2.0f * (m_q0 * m_q1 + m_q2 * m_q3),
                      1.0f - 2.0f * (m_q1 * m_q1 + m_q2 * m_q2));
        pitch = asinf(clampOne(2.0f * (m_q0 * m_q2 - m_q3 * m_q1)));
        yaw = atan2f(2.0f * (m_q0 * m_q3 + m_q1 * m_q2),
                     1.0f - 2.0f * (m_q2 * m_q2 + m_q3 * m_q3));
        roll *= R2D; pitch *= R2D; yaw *= R2D;
    }

    void getLinearAcceleration(float ax, float ay, float az,
                               float& lx, float& ly, float& lz) const {
        const float gX = 2.0f * (m_q1 * m_q3 - m_q0 * m_q2);
        const float gY = 2.0f * (m_q0 * m_q1 + m_q2 * m_q3);
        const float gZ = m_q0 * m_q0 - m_q1 * m_q1 - m_q2 * m_q2 + m_q3 * m_q3;
        lx = ax - gX; ly = ay - gY; lz = az - gZ;
    }

private:
    // ---------- /TRIAD  ----------
    void initialize(float ax, float ay, float az, float mx, float my, float mz) {
        const float aN = sqrtf(ax * ax + ay * ay + az * az);
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        if (aN > 1e-6f) {
            const float axn = ax / aN, ayn = ay / aN, azn = az / aN;
            roll = atan2f(ayn, azn);
            pitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));
        }
        const float mN = sqrtf(mx * mx + my * my + mz * mz);
        if (mN > 1e-6f) {
            const float cr = cosf(roll), sr = sinf(roll);
            const float cp = cosf(pitch), sp = sinf(pitch);
            const float mxh = mx * cp + my * sr * sp + mz * cr * sp;
            const float myh = my * cr - mz * sr;
            if (sqrtf(mxh * mxh + myh * myh) > 1e-6f)
                yaw = atan2f(-myh, mxh);
        }
        eulerToQuat(roll, pitch, yaw, m_q0, m_q1, m_q2, m_q3);

        //  = R(q_init)m_body
        m_magWorldSet = false;
        if (mN > 1e-6f) {
            worldFromBody(m_q0, m_q1, m_q2, m_q3, mx / mN, my / mN, mz / mN,
                          m_magW[0], m_magW[1], m_magW[2]);
            m_magWorldSet = true;
        }
    }

    // ----------  y = z  hH_ = [h]H_b = 0 ----------
    void measurementUpdate(const float h[3], const float z[3], float sigma) {
        const double R = (double)sigma * (double)sigma;
        double H[3][6] = {};
        H[0][0] = 0.0;   H[0][1] = -h[2]; H[0][2] = h[1];
        H[1][0] = h[2];  H[1][1] = 0.0;   H[1][2] = -h[0];
        H[2][0] = -h[1]; H[2][1] = h[0];  H[2][2] = 0.0;

        // S = H P H + R (3x3)
        double HP[3][6];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 6; ++j) {
                double v = 0.0;
                for (int k = 0; k < 6; ++k) v += H[i][k] * m_P[k][j];
                HP[i][j] = v;
            }
        double S[3][3];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double v = 0.0;
                for (int k = 0; k < 6; ++k) v += HP[i][k] * H[j][k];
                S[i][j] = v + (i == j ? R : 0.0);
            }
        // K = P H S (6x3)
        double PHt[6][3];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 3; ++j) {
                double v = 0.0;
                for (int k = 0; k < 6; ++k) v += m_P[i][k] * H[j][k];
                PHt[i][j] = v;
            }
        double Sinv[3][3];
        if (!invert3(S, Sinv)) return;  // 
        double K[6][3];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 3; ++j) {
                double v = 0.0;
                for (int k = 0; k < 3; ++k) v += PHt[i][k] * Sinv[k][j];
                K[i][j] = v;
            }

        // x = Ky
        double dx[6] = { 0, 0, 0, 0, 0, 0 };
        for (int i = 0; i < 6; ++i)
            for (int k = 0; k < 3; ++k)
                dx[i] += K[i][k] * ((double)z[k] - (double)h[k]);
        // : q <- q * dq(deltaTheta), b += deltaB
        const float dth[3] = { (float)dx[0], (float)dx[1], (float)dx[2] };
        quatMulSmall(m_q0, m_q1, m_q2, m_q3, dth);
        for (int i = 0; i < 3; ++i) m_bias[i] += (float)dx[3 + i];

        // Joseph P  (IKH)P(IKH) + KRK
        double IKH[6][6];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                double v = (i == j) ? 1.0 : 0.0;
                for (int k = 0; k < 3; ++k) v -= K[i][k] * H[k][j];
                IKH[i][j] = v;
            }
        double T1[6][6];
        mat6mul(IKH, m_P, T1);
        double T2[6][6];
        mat6mulT2(T1, IKH, T2);
        double KR[6][6] = {};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 3; ++k)
                    KR[i][j] += K[i][k] * K[j][k] * R;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) m_P[i][j] = T2[i][j] + KR[i][j];
    }

    void updateAccel(float ax, float ay, float az) {
        const float n = sqrtf(ax * ax + ay * ay + az * az);
        if (n < 1e-6f) return;
        const float z[3] = { ax / n, ay / n, az / n };
        float h[3];
        bodyFromWorld(m_q0, m_q1, m_q2, m_q3, 0.0f, 0.0f, 1.0f, h[0], h[1], h[2]);
        measurementUpdate(h, z, m_sigmaAccel);
    }

    void updateMag(float mx, float my, float mz) {
        const float n = sqrtf(mx * mx + my * my + mz * mz);
        if (n < 1e-6f) return;  // 6 
        const float z[3] = { mx / n, my / n, mz / n };
        if (!m_magWorldSet) {
            // 
            worldFromBody(m_q0, m_q1, m_q2, m_q3, z[0], z[1], z[2],
                          m_magW[0], m_magW[1], m_magW[2]);
            m_magWorldSet = true;
            return;
        }
        float h[3];
        bodyFromWorld(m_q0, m_q1, m_q2, m_q3, m_magW[0], m_magW[1], m_magW[2],
                      h[0], h[1], h[2]);
        measurementUpdate(h, z, m_sigmaMag);
    }

    // ----------  ----------
    static float clampOne(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }

    float currentDt() const { return (m_dtOverride > 0.0f) ? m_dtOverride : 1.0f / m_sampleRate; }

    // q =  q  [0,]
    void qDot(float& q0, float& q1, float& q2, float& q3,
              float wx, float wy, float wz, float dt) {
        const float d0 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
        const float d1 = 0.5f * (q0 * wx + q2 * wz - q3 * wy);
        const float d2 = 0.5f * (q0 * wy - q1 * wz + q3 * wx);
        const float d3 = 0.5f * (q0 * wz + q1 * wy - q2 * wx);
        q0 += d0 * dt; q1 += d1 * dt; q2 += d2 * dt; q3 += d3 * dt;
        const float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        if (n > 1e-9f) { q0 /= n; q1 /= n; q2 /= n; q3 /= n; }
    }

    // q  q  [1, /2]
    static void quatMulSmall(float& q0, float& q1, float& q2, float& q3,
                             const float dth[3]) {
        const float hx = 0.5f * dth[0], hy = 0.5f * dth[1], hz = 0.5f * dth[2];
        const float r0 = q0 - q1 * hx - q2 * hy - q3 * hz;
        const float r1 = q0 * hx + q1 + q2 * hz - q3 * hy;
        const float r2 = q0 * hy - q1 * hz + q2 + q3 * hx;
        const float r3 = q0 * hz + q1 * hy - q2 * hx + q3;
        const float n = sqrtf(r0 * r0 + r1 * r1 + r2 * r2 + r3 * r3);
        if (n > 1e-9f) { q0 = r0 / n; q1 = r1 / n; q2 = r2 / n; q3 = r3 / n; }
    }

    // v_body = R(q)v_world
    static void bodyFromWorld(float q0, float q1, float q2, float q3,
                              float wx, float wy, float wz,
                              float& bx, float& by, float& bz) {
        bx = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * wx + 2.0f * (q1 * q2 + q0 * q3) * wy + 2.0f * (q1 * q3 - q0 * q2) * wz;
        by = 2.0f * (q1 * q2 - q0 * q3) * wx + (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * wy + 2.0f * (q2 * q3 + q0 * q1) * wz;
        bz = 2.0f * (q1 * q3 + q0 * q2) * wx + 2.0f * (q2 * q3 - q0 * q1) * wy + (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * wz;
    }

    // v_world = R(q)v_body
    static void worldFromBody(float q0, float q1, float q2, float q3,
                              float bx, float by, float bz,
                              float& wx, float& wy, float& wz) {
        wx = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * bx + 2.0f * (q1 * q2 - q0 * q3) * by + 2.0f * (q1 * q3 + q0 * q2) * bz;
        wy = 2.0f * (q1 * q2 + q0 * q3) * bx + (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * by + 2.0f * (q2 * q3 - q0 * q1) * bz;
        wz = 2.0f * (q1 * q3 - q0 * q2) * bx + 2.0f * (q2 * q3 + q0 * q1) * by + (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * bz;
    }

    static void eulerToQuat(float roll, float pitch, float yaw,
                            float& q0, float& q1, float& q2, float& q3) {
        const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
        const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
        const float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
        q0 = cy * cp * cr + sy * sp * sr;
        q1 = cy * cp * sr - sy * sp * cr;
        q2 = sy * cp * sr + cy * sp * cr;
        q3 = sy * cp * cr - cy * sp * sr;
    }

    static void mat6mul(const double X[6][6], const double Y[6][6], double Z[6][6]) {
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                double v = 0.0;
                for (int k = 0; k < 6; ++k) v += X[i][k] * Y[k][j];
                Z[i][j] = v;
            }
    }

    static void mat6mulT2(const double X[6][6], const double Y[6][6], double Z[6][6]) {
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                double v = 0.0;
                for (int k = 0; k < 6; ++k) v += X[i][k] * Y[j][k];
                Z[i][j] = v;
            }
    }

    static bool invert3(const double M[3][3], double Minv[3][3]) {
        const double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                         - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                         + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
        if (!(std::fabs(det) > 1e-12)) return false;
        const double inv = 1.0 / det;
        Minv[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * inv;
        Minv[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) * inv;
        Minv[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * inv;
        Minv[1][0] = (M[1][2] * M[2][0] - M[1][0] * M[2][2]) * inv;
        Minv[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * inv;
        Minv[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) * inv;
        Minv[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) * inv;
        Minv[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) * inv;
        Minv[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * inv;
        return true;
    }

    float m_sampleRate;
    float m_sigmaGyro, m_sigmaAccel, m_sigmaMag, m_sigmaBiasWalk;
    float m_q0, m_q1, m_q2, m_q3;
    float m_bias[3];
    double m_P[6][6];
    float m_magW[3] = { 0, 0, 0 };
    bool m_magWorldSet = false;
    bool m_initialized = false;
    float m_dtOverride = -1.0f;
};

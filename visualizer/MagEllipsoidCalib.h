#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

/**
 * @class MagEllipsoidCalib
 * @brief  + 
 *
 * 
 *   LPMS  OpenZen  "MagCalib" 
 *    |B|  20% 
 *    90  55  190
 *    + 
 *
 * 
 *    x  (x - c)^T Q (x - c) = r^2 
 *    Qcr
 *       m = L^T (x - c) / r      Q = LL^TCholesky
 *    1  |m|  
 *
 * 
 *   
 *     x^T Q x + 2 b^T x + c = 110  10x10 
 *     center = -Q^{-1} br^2 = 1 - c - bcenter
 *
 * 
 *   -  >= 300
 *   - Q  Choleskyr^2 > 0
 *   -  = / >= 0.25
 */
class MagEllipsoidCalib {
public:
    struct Result {
        bool ok = false;
        double center[3] = { 0, 0, 0 };
        double M[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };  ///<  3x3 
        double radius = 1.0;
        double rmsResidual = 0.0;  ///<  |m|-1  RMS0
        double coverage = 0.0;     ///<  [0,1]
        int failStep = 0;          ///< 0=1= 2= 3=Q 4=r2 5= 6=

        // --- physical geometry, filled by deriveGeometry() ---
        // radius above is the quadric's internal normalization (r*r0), NOT a length
        // in the sensor's unit; it is off by 1/sqrt(lambda). m = M(x - center) maps
        // the ellipsoid onto the unit sphere, so the semi-axis along an eigenvector
        // of M^T M with eigenvalue mu is 1/sqrt(mu). Read these instead of radius.
        double semiAxis[3] = { 1, 1, 1 };  ///< semi-axes, same unit as input (uT)
        double fieldMag = 1.0;             ///< mean semi-axis == local field magnitude (uT)
        double anisotropy = 1.0;           ///< max/min semi-axis; 1.0 = perfect sphere
        double hardIron = 0.0;             ///< |center|, same unit as input (uT)
    };

    /** @brief Fill semiAxis / fieldMag / anisotropy / hardIron from center and M. */
    static void deriveGeometry(Result& r) {
        double G[3][3] = {};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k) G[i][j] += r.M[3 * k + i] * r.M[3 * k + j];

        double V[3][3], lam[3];
        eig3(G, V, lam);
        for (int i = 0; i < 3; ++i)
            r.semiAxis[i] = (lam[i] > 1e-30) ? 1.0 / std::sqrt(lam[i]) : 0.0;

        const double mx = std::max(r.semiAxis[0], std::max(r.semiAxis[1], r.semiAxis[2]));
        const double mn = std::min(r.semiAxis[0], std::min(r.semiAxis[1], r.semiAxis[2]));
        r.fieldMag = (r.semiAxis[0] + r.semiAxis[1] + r.semiAxis[2]) / 3.0;
        r.anisotropy = (mn > 1e-12) ? mx / mn : 0.0;
        r.hardIron = std::sqrt(r.center[0] * r.center[0]
                             + r.center[1] * r.center[1]
                             + r.center[2] * r.center[2]);
    }

    static Result fit(const std::vector<std::array<float, 3>>& samples, size_t minSamples = 300) {
        Result res;
        const size_t n = samples.size();
        if (n < minSamples) { res.failStep = 1; return res; }

        //  RMS  ~T 
        //  1e12+ r2 
        //  O(1) 1e4 
        double mean[3] = { 0, 0, 0 };
        for (const auto& s : samples)
            for (int i = 0; i < 3; ++i) mean[i] += s[i];
        for (int i = 0; i < 3; ++i) mean[i] /= n;

        double r0 = 0.0;
        for (const auto& s : samples) {
            const double dx = s[0] - mean[0], dy = s[1] - mean[1], dz = s[2] - mean[2];
            r0 += dx * dx + dy * dy + dz * dz;
        }
        r0 = std::sqrt(r0 / (3.0 * n));
        if (!(r0 > 1e-6)) { res.failStep = 4; return res; }

        // ----  (9x9) u = [Q(6), b(3)] ----
        //  RHS=1
        // "Q=0,b=0,c=1" 
        //   x^T Q x + 2 b^T x = 1
        // : [x2, 2xy, 2xz, y2, 2yz, z2, 2x, 2y, 2z]  u = 1
        double A[9][9] = {};
        double rhs[9] = {};
        for (const auto& s : samples) {
            const double x = (s[0] - mean[0]) / r0, y = (s[1] - mean[1]) / r0,
                         z = (s[2] - mean[2]) / r0;
            const double row[9] = { x * x, 2 * x * y, 2 * x * z, y * y, 2 * y * z,
                                    z * z, 2 * x, 2 * y, 2 * z };
            for (int i = 0; i < 9; ++i) {
                rhs[i] += row[i];
                for (int j = i; j < 9; ++j) A[i][j] += row[i] * row[j];
            }
        }
        for (int i = 0; i < 9; ++i)
            for (int j = 0; j < i; ++j) A[i][j] = A[j][i];
        // 
        for (int i = 0; i < 9; ++i) A[i][i] += 1e-9 * A[i][i] + 1e-12;

        double u[9];
        if (!solveLin<9>(A, rhs, u)) { res.failStep = 2; return res; }

        const double Q[3][3] = { { u[0], u[1], u[2] },
                                 { u[1], u[3], u[4] },
                                 { u[2], u[4], u[5] } };
        const double b[3] = { u[6], u[7], u[8] };

        double Qinv[3][3];
        if (!invert3(Q, Qinv)) { res.failStep = 3; return res; }

        // center = -Q^{-1} br = 1 + b^T Q^{-1} b = 1 - bcenterRel
        double centerRel[3] = { 0, 0, 0 };
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) centerRel[i] -= Qinv[i][j] * b[j];

        const double r2 = 1.0 - (b[0] * centerRel[0] + b[1] * centerRel[1] + b[2] * centerRel[2]);
        if (!(r2 > 1e-6)) { res.failStep = 4; return res; }
        const double r = std::sqrt(r2);

        //  S = Vdiag()Q = VV^T
        //  Cholesky  L^T
        // 
        double V[3][3], lam[3];
        if (!eig3(Q, V, lam)) { res.failStep = 5; return res; }
        for (int i = 0; i < 3; ++i) {
            if (!(lam[i] > 1e-12)) { res.failStep = 5; return res; }
        }

        //   
        //   x_s = (x - mean)/r0m = S_s(x_s - c_s)/r_s = (S_s/(r_sr0))(x - mean - r0c_s)
        res.ok = true;
        for (int i = 0; i < 3; ++i) res.center[i] = mean[i] + r0 * centerRel[i];
        double S[3][3] = {};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    S[i][j] += V[i][k] * std::sqrt(lam[k]) * V[j][k];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) res.M[3 * i + j] = S[i][j] / (r * r0);
        res.radius = r * r0;
        deriveGeometry(res);

        // ----  ----
        double sumErr = 0.0;
        for (const auto& s : samples) {
            double m[3];
            apply(res, s[0], s[1], s[2], m);
            const double norm = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
            sumErr += (norm - 1.0) * (norm - 1.0);
        }
        res.rmsResidual = std::sqrt(sumErr / n);

        // ""
        //   1   0
        // 
        {
            std::vector<std::array<float, 3>> dirs;
            dirs.reserve(samples.size());
            for (const auto& s : samples) {
                double m[3];
                apply(res, s[0], s[1], s[2], m);
                const double nn = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
                if (nn > 1e-9)
                    dirs.push_back({ (float)(m[0] / nn), (float)(m[1] / nn), (float)(m[2] / nn) });
            }
            double mean2[3] = { 0, 0, 0 };
            for (auto& d : dirs)
                for (int i = 0; i < 3; ++i) mean2[i] += d[i];
            for (int i = 0; i < 3; ++i) mean2[i] /= dirs.size();
            res.coverage = coverageRatio(dirs, mean2);
        }

        // GUI 
        // - coverage10
        //   "" 0.2~0.6
        // - rms |m|-1  RMS
        if (res.coverage < 0.15 || res.rmsResidual > 0.25) { res.ok = false; res.failStep = 6; }
        return res;
    }

    /**
     * @brief 
     */
    static double quickCoverage(const std::vector<std::array<float, 3>>& samples) {
        if (samples.size() < 50) return 0.0;
        double mean[3] = { 0, 0, 0 };
        for (const auto& s : samples)
            for (int i = 0; i < 3; ++i) mean[i] += s[i];
        for (int i = 0; i < 3; ++i) mean[i] /= samples.size();
        return coverageRatio(samples, mean);
    }

    static void apply(const Result& r, float mx, float my, float mz, double out[3]) {
        const double d[3] = { mx - r.center[0], my - r.center[1], mz - r.center[2] };
        for (int i = 0; i < 3; ++i)
            out[i] = r.M[3 * i + 0] * d[0] + r.M[3 * i + 1] * d[1] + r.M[3 * i + 2] * d[2];
    }

    /**
     * @brief |m| of one raw sample under this calibration.
     *
     * 1.0 means the sample lies on the fitted sphere. Watching this live is how you
     * detect a calibration that has gone stale: the fit was made in one magnetic
     * environment, and a hard-iron offset that large is usually environmental, so
     * moving the sensor makes live |m| drift away from 1.
     */
    static double mappedNorm(const Result& r, float mx, float my, float mz) {
        double m[3];
        apply(r, mx, my, mz, m);
        return std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    }

    /**
     * @brief Tally which of the 8 sign octants a sample cloud occupies about center.
     *
     * coverageRatio() measures the eigenvalue spread of the direction covariance,
     * which can still look acceptable for a cloud confined to a narrow cone - and a
     * cone leaves the 9-parameter quadric fit under-constrained. Counting octants
     * catches that directly: a proper tumble visits all 8.
     *
     * @return number of non-empty octants, 0..8
     */
    static int octantCounts(const std::vector<std::array<float, 3>>& samples,
                            const double center[3], int outCounts[8]) {
        for (int i = 0; i < 8; ++i) outCounts[i] = 0;
        for (const auto& s : samples) {
            const int idx = ((s[0] - center[0] > 0.0) ? 1 : 0)
                          | ((s[1] - center[1] > 0.0) ? 2 : 0)
                          | ((s[2] - center[2] > 0.0) ? 4 : 0);
            outCounts[idx]++;
        }
        int filled = 0;
        for (int i = 0; i < 8; ++i) if (outCounts[i] > 0) filled++;
        return filled;
    }

    /**
     * @brief C = M^{-1}, i.e. x = center + C*u maps a unit-sphere point back to raw uT.
     * @return false if M is singular
     */
    static bool inverseM(const Result& r, double out[9]) {
        const double M[3][3] = { { r.M[0], r.M[1], r.M[2] },
                                 { r.M[3], r.M[4], r.M[5] },
                                 { r.M[6], r.M[7], r.M[8] } };
        double C[3][3];
        if (!invert3(M, C)) return false;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) out[3 * i + j] = C[i][j];
        return true;
    }

    /**
     * @brief Exact orthographic silhouette of the fitted ellipsoid in one coordinate plane.
     *
     * The ellipsoid is { center + C*u : |u| = 1 } with C = M^{-1}. Dropping the third
     * axis leaves y = c2 + B*u where B holds the two kept rows of C, and the outline of
     * that image is the ellipse (y - c2)^T (B B^T)^{-1} (y - c2) = 1. Parametrising it
     * from the closed-form 2x2 eigen-decomposition of G = B B^T gives the true outline;
     * just plotting projected surface points instead would fill the interior and hide it.
     *
     * @param keepA, keepB  axis indices to plot on x and y (0=X, 1=Y, 2=Z)
     * @param segments      outline resolution
     * @return false if the projection degenerates or M is singular
     */
    static bool projectedEllipse(const Result& r, int keepA, int keepB,
                                 std::vector<double>& outA, std::vector<double>& outB,
                                 int segments = 160) {
        double C[9];
        if (!inverseM(r, C)) return false;

        // B = the two kept rows of C (2x3)
        double B[2][3];
        for (int j = 0; j < 3; ++j) { B[0][j] = C[3 * keepA + j]; B[1][j] = C[3 * keepB + j]; }

        double G[2][2] = {};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 3; ++k) G[i][j] += B[i][k] * B[j][k];

        // closed-form symmetric 2x2 eigen-decomposition
        const double tr = 0.5 * (G[0][0] + G[1][1]);
        const double dd = std::sqrt(std::max(0.0, tr * tr - (G[0][0] * G[1][1] - G[0][1] * G[1][0])));
        const double l0 = tr + dd, l1 = tr - dd;
        if (!(l0 > 1e-24) || !(l1 > 1e-24)) return false;

        double v0[2], v1[2];
        if (std::fabs(G[0][1]) > 1e-18) {
            v0[0] = l0 - G[1][1]; v0[1] = G[0][1];
            v1[0] = l1 - G[1][1]; v1[1] = G[0][1];
        } else {
            v0[0] = 1; v0[1] = 0;
            v1[0] = 0; v1[1] = 1;
        }
        const double n0 = std::sqrt(v0[0] * v0[0] + v0[1] * v0[1]);
        const double n1 = std::sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
        if (!(n0 > 1e-18) || !(n1 > 1e-18)) return false;
        for (int i = 0; i < 2; ++i) { v0[i] /= n0; v1[i] /= n1; }

        // boundary of {B u : |u|<=1} is sum_i (v_i . y)^2 / lambda_i = 1,
        // so the semi-axis along eigenvector i is sqrt(lambda_i)
        const double a0 = std::sqrt(l0), a1 = std::sqrt(l1);
        const double cA = r.center[keepA], cB = r.center[keepB];

        outA.resize(segments + 1);
        outB.resize(segments + 1);
        for (int i = 0; i <= segments; ++i) {
            const double t = 2.0 * 3.14159265358979323846 * i / segments;
            const double p = a0 * std::cos(t), q = a1 * std::sin(t);
            outA[i] = cA + v0[0] * p + v1[0] * q;
            outB[i] = cB + v0[1] * p + v1[1] * q;
        }
        return true;
    }

private:
    // /3x3  Jacobi
    static double coverageRatio(const std::vector<std::array<float, 3>>& samples,
                                const double center[3]) {
        double C[3][3] = {};
        for (const auto& s : samples) {
            const double d[3] = { s[0] - center[0], s[1] - center[1], s[2] - center[2] };
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) C[i][j] += d[i] * d[j];
        }
        const double k = 1.0 / samples.size();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) C[i][j] *= k;

        double a[3][3] = { { C[0][0], C[0][1], C[0][2] },
                           { C[0][1], C[1][1], C[1][2] },
                           { C[0][2], C[1][2], C[2][2] } };
        for (int sweep = 0; sweep < 50; ++sweep) {
            for (int p = 0; p < 2; ++p) {
                for (int q = p + 1; q < 3; ++q) {
                    if (std::fabs(a[p][q]) < 1e-15) continue;
                    const double theta = 0.5 * std::atan2(2 * a[p][q], a[q][q] - a[p][p]);
                    const double ct = std::cos(theta), st = std::sin(theta);
                    for (int k = 0; k < 3; ++k) {
                        const double akp = a[k][p], akq = a[k][q];
                        a[k][p] = ct * akp - st * akq;
                        a[k][q] = st * akp + ct * akq;
                    }
                    for (int k = 0; k < 3; ++k) {
                        const double apk = a[p][k], aqk = a[q][k];
                        a[p][k] = ct * apk - st * aqk;
                        a[q][k] = st * apk + ct * aqk;
                    }
                }
            }
        }
        const double l0 = a[0][0], l1 = a[1][1], l2 = a[2][2];
        const double mx = std::max(l0, std::max(l1, l2));
        const double mn = std::min(l0, std::min(l1, l2));
        if (mx <= 1e-12) return 0.0;
        return mn / mx;
    }

    // 3x3 Jacobi V lam 
    static bool eig3(const double Q[3][3], double V[3][3], double lam[3]) {
        double a[3][3] = { { Q[0][0], Q[0][1], Q[0][2] },
                           { Q[0][1], Q[1][1], Q[1][2] },
                           { Q[0][2], Q[1][2], Q[2][2] } };
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) V[i][j] = (i == j) ? 1.0 : 0.0;
        for (int sweep = 0; sweep < 100; ++sweep) {
            double off = std::fabs(a[0][1]) + std::fabs(a[0][2]) + std::fabs(a[1][2]);
            if (off < 1e-15 * (std::fabs(a[0][0]) + std::fabs(a[1][1]) + std::fabs(a[2][2]) + 1e-30))
                break;
            for (int p = 0; p < 2; ++p) {
                for (int q = p + 1; q < 3; ++q) {
                    if (std::fabs(a[p][q]) < 1e-300) continue;
                    const double theta = 0.5 * std::atan2(2 * a[p][q], a[q][q] - a[p][p]);
                    const double ct = std::cos(theta), st = std::sin(theta);
                    for (int k = 0; k < 3; ++k) {
                        const double akp = a[k][p], akq = a[k][q];
                        a[k][p] = ct * akp - st * akq;
                        a[k][q] = st * akp + ct * akq;
                    }
                    for (int k = 0; k < 3; ++k) {
                        const double apk = a[p][k], aqk = a[q][k];
                        a[p][k] = ct * apk - st * aqk;
                        a[q][k] = st * apk + ct * aqk;
                    }
                    for (int k = 0; k < 3; ++k) {
                        const double vkp = V[k][p], vkq = V[k][q];
                        V[k][p] = ct * vkp - st * vkq;
                        V[k][q] = st * vkp + ct * vkq;
                    }
                }
            }
        }
        lam[0] = a[0][0]; lam[1] = a[1][1]; lam[2] = a[2][2];
        return true;
    }

    static bool invert3(const double M[3][3], double Minv[3][3]) {
        const double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                         - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                         + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
        if (std::fabs(det) < 1e-15) return false;
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

    // N x N N  10
    template <int NN>
    static bool solveLin(double A[NN][NN], const double rhs[NN], double out[NN]) {
        double a[NN][NN + 1];
        for (int i = 0; i < NN; ++i) {
            for (int j = 0; j < NN; ++j) a[i][j] = A[i][j];
            a[i][NN] = rhs[i];
        }
        for (int col = 0; col < NN; ++col) {
            int piv = col;
            for (int i = col + 1; i < NN; ++i)
                if (std::fabs(a[i][col]) > std::fabs(a[piv][col])) piv = i;
            if (std::fabs(a[piv][col]) < 1e-18) return false;
            if (piv != col)
                for (int j = col; j <= NN; ++j) std::swap(a[col][j], a[piv][j]);
            const double d = a[col][col];
            for (int j = col; j <= NN; ++j) a[col][j] /= d;
            for (int i = 0; i < NN; ++i) {
                if (i == col) continue;
                const double f = a[i][col];
                if (f == 0.0) continue;
                for (int j = col; j <= NN; ++j) a[i][j] -= f * a[col][j];
            }
        }
        for (int i = 0; i < NN; ++i) out[i] = a[i][NN];
        return true;
    }
};

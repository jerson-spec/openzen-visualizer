#pragma once

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

/**
 * @class MagFieldMap
 * @brief 室内磁干扰场图（v0：按格点离散表，无插值）
 *
 * 椭球校准（MagEllipsoidCalib）解决传感器自身硬/软铁误差，但对环境场的
 * 方向"盲"：固定磁物体（显示器/机箱/桌架）让每个位置的"磁北"指向不同。
 * 本类把 Field Survey 逐格测得的场向量组织成场图：
 *
 *   - 每格一个"调查水平系"下的场向量 fieldLevel (uT)
 *   - 参考格 refIndex 的场向量即"干净北"的基准
 *   - 干扰场 dB(p) = fieldLevel(p) - fieldLevel(ref)
 *
 * 运行时（SensorThread::run）在喂滤波器之前，把活动格的 dB 旋到当前
 * 传感器系扣除——滤波器从此看到参考点的均匀场，航向在全工作区一致。
 *
 * 坐标系与公式（与 GuiRenderer 倾角补偿航向完全同源，保证一致）：
 *   传感器系 -> 水平系（roll phi, pitch theta）：
 *     Xh =  x*ct + y*st*st + z*ct*st
 *     Yh =         y*cp - z*st
 *     Zh = -x*st + y*st*ct + z*ct*ct
 *   航向 h = atan2(-Yh, Xh)
 *   水平系 -> 传感器系为上述变换的转置。
 *   运行时传感器在调查系中的方位角 psi = alpha(p) - h_raw，其中 alpha(p)
 *   为活动格场方位角（由 fieldLevel 得出），h_raw 为当前实测场的航向
 *   ——不需要迭代，一步即精确（psi 由测量直接给出）。
 *
 * 文件：
 *   field_survey.txt（Field Survey 窗口写）-> parseSurveyFile() 读入
 *   mag_field_map.txt（LPMS_MAG_MAP_V1） -> build() 后 save()，启动时 load()
 */
class MagFieldMap {
public:
    static constexpr double kPi = 3.14159265358979323846;

    /// 一个格点：同标签多次会话测量的平均
    struct Entry {
        char label[40] = "";
        double fieldLevel[3] = { 0, 0, 0 };  ///< 调查水平系场向量 (uT)
        double fieldMag = 0.0;               ///< 该格 |B| 均值 (uT)
        double headingDeg = 0.0;             ///< 场方位角（度，显示/校验用）
        double scatterDeg = 0.0;             ///< 跨会话航向最大散布（放置重复性指标）
        int nSamples = 0;                    ///< 累计静止样本数
    };

    struct Result {
        bool ok = false;
        double fieldMagAtBuild = 0.0;        ///< 建图时的椭球拟合场强 (uT)
        int refIndex = 0;                    ///< 参考格下标
        std::vector<Entry> entries;

        /// 指定格点的干扰 dB（调查水平系, uT）；下标越界返回 false
        bool deltaLevel(int idx, double out[3]) const {
            if (!ok || idx < 0 || idx >= (int)entries.size()
                || refIndex < 0 || refIndex >= (int)entries.size()) return false;
            for (int i = 0; i < 3; ++i)
                out[i] = entries[idx].fieldLevel[i] - entries[refIndex].fieldLevel[i];
            return true;
        }

        /// 指定格点的场方位角（rad）
        double alphaRad(int idx) const {
            return std::atan2(-entries[idx].fieldLevel[1], entries[idx].fieldLevel[0]);
        }
    };

    /// field_survey.txt 的一行（解析后的有效字段）
    struct SurveyRow {
        std::string label;
        double magUnit[3] = { 0, 0, 0 };  ///< 传感器系单位方向（椭球校准后，未做场图补偿）
        double bMag = 0.0;                ///< 校准后 |B| (uT)
        double acc[3] = { 0, 0, 0 };      ///< 加速度均值 (g，SensorData 约定)
        double headingDeg = 0.0;          ///< 记录的航向（校验用）
        int n = 0;                        ///< 静止样本数
    };

    /**
     * @brief 解析 field_survey.txt（TSV；跳过 # 注释行，容忍 UTF-8 BOM）。
     * 列序与 SensorThread::surveyMark 写出一致：
     *   DateTime Label SensorTime Heading HeadingDev Bmag BmagDev
     *   MagUnit_XYZ MagB_XYZ AccXYZ N
     */
    static bool parseSurveyFile(const char* path, std::vector<SurveyRow>& rows,
                                std::string& err) {
        std::ifstream f(path, std::ios::binary);
        if (!f.is_open()) {
            err = std::string("cannot open ") + path;
            return false;
        }
        rows.clear();
        std::string line;
        bool first = true;
        while (std::getline(f, line)) {
            if (first) {  // UTF-8 BOM
                if (line.size() >= 3 && (unsigned char)line[0] == 0xEF
                    && (unsigned char)line[1] == 0xBB && (unsigned char)line[2] == 0xBF)
                    line = line.substr(3);
                first = false;
            }
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            std::string c[17];
            for (int i = 0; i < 17; ++i)
                if (!std::getline(ss, c[i], '\t')) c[i].clear();
            if (c[1].empty()) continue;
            SurveyRow r;
            r.label = c[1];
            r.headingDeg = std::atof(c[3].c_str());
            r.bMag = std::atof(c[5].c_str());
            r.magUnit[0] = std::atof(c[7].c_str());
            r.magUnit[1] = std::atof(c[8].c_str());
            r.magUnit[2] = std::atof(c[9].c_str());
            r.acc[0] = std::atof(c[13].c_str());
            r.acc[1] = std::atof(c[14].c_str());
            r.acc[2] = std::atof(c[15].c_str());
            r.n = std::atoi(c[16].c_str());
            if (r.bMag > 1.0 && r.n > 0)
                rows.push_back(r);
        }
        if (rows.empty()) {
            err = std::string("no usable rows in ") + path;
            return false;
        }
        return true;
    }

    /**
     * @brief 由调查行建图：同标签跨会话聚合平均（水平系向量直接平均，
     * 航向用圆周平均），scatterDeg 记录该标签各次会话航向偏离均值的
     * 最大值——即放置重复性/环境漂移的直接指标。
     * @param refLabel    参考格标签；空或找不到时用第一个出现的标签
     * @param fieldMagNow 当前椭球拟合场强（写入 fieldMagAtBuild 供一致性检查）
     */
    static Result build(const std::vector<SurveyRow>& rows, const std::string& refLabel,
                        double fieldMagNow) {
        Result res;
        // 按标签聚合（保持首次出现顺序）
        std::vector<std::vector<const SurveyRow*>> groups;
        for (const auto& r : rows) {
            int gi = -1;
            for (size_t i = 0; i < groups.size(); ++i)
                if (groups[i].front()->label == r.label) { gi = (int)i; break; }
            if (gi < 0) groups.push_back({ &r });
            else groups[gi].push_back(&r);
        }
        if (groups.empty()) return res;

        res.entries.resize(groups.size());
        for (size_t g = 0; g < groups.size(); ++g) {
            Entry& e = res.entries[g];
            snprintf(e.label, sizeof(e.label), "%s", groups[g].front()->label.c_str());
            double sumL[3] = { 0, 0, 0 }, sumB = 0.0, sumSin = 0.0, sumCos = 0.0;
            int nTot = 0;
            std::vector<double> hs;
            hs.reserve(groups[g].size());
            for (const SurveyRow* r : groups[g]) {
                const double Bs[3] = { r->magUnit[0] * r->bMag, r->magUnit[1] * r->bMag,
                                       r->magUnit[2] * r->bMag };
                double roll, pitch, L[3];
                accToRollPitch(r->acc, roll, pitch);
                const double h = toLevel(Bs, roll, pitch, L);
                for (int i = 0; i < 3; ++i) sumL[i] += L[i];
                sumB += r->bMag;
                sumSin += std::sin(h);
                sumCos += std::cos(h);
                hs.push_back(h);
                nTot += r->n;
            }
            const int k = (int)groups[g].size();
            for (int i = 0; i < 3; ++i) e.fieldLevel[i] = sumL[i] / k;
            e.fieldMag = sumB / k;
            const double hMean = std::atan2(sumSin, sumCos);
            e.headingDeg = hMean * 180.0 / kPi;
            double mx = 0.0;
            for (double h : hs) {
                const double d = std::fabs(std::atan2(std::sin(h - hMean),
                                                      std::cos(h - hMean)));
                mx = std::max(mx, d);
            }
            e.scatterDeg = mx * 180.0 / kPi;
            e.nSamples = nTot;
        }

        res.fieldMagAtBuild = fieldMagNow;
        res.refIndex = 0;
        for (size_t i = 0; i < res.entries.size(); ++i)
            if (refLabel == res.entries[i].label) { res.refIndex = (int)i; break; }
        res.ok = !res.entries.empty();
        return res;
    }

    /// 保存为 mag_field_map.txt（LPMS_MAG_MAP_V1）
    static bool save(const char* path, const Result& r) {
        std::ofstream f(path, std::ios::out | std::ios::trunc);
        if (!f.is_open()) return false;
        f << "LPMS_MAG_MAP_V1\n";
        f << std::setprecision(10);
        f << r.fieldMagAtBuild << " " << r.refIndex << " " << r.entries.size() << "\n";
        for (const auto& e : r.entries)
            f << e.label << " " << e.fieldLevel[0] << " " << e.fieldLevel[1] << " "
              << e.fieldLevel[2] << " " << e.fieldMag << " " << e.headingDeg << " "
              << e.scatterDeg << " " << e.nSamples << "\n";
        return true;
    }

    /// 读回 mag_field_map.txt；格式不符返回 false
    static bool load(const char* path, Result& r) {
        std::ifstream f(path);
        if (!f.is_open()) return false;
        std::string tag;
        if (!(f >> tag) || tag != "LPMS_MAG_MAP_V1") return false;
        int n = 0;
        if (!(f >> r.fieldMagAtBuild >> r.refIndex >> n) || n <= 0 || n > 10000) return false;
        r.entries.resize(n);
        for (int i = 0; i < n; ++i) {
            Entry& e = r.entries[i];
            std::string label;
            if (!(f >> label >> e.fieldLevel[0] >> e.fieldLevel[1] >> e.fieldLevel[2]
                  >> e.fieldMag >> e.headingDeg >> e.scatterDeg >> e.nSamples))
                return false;
            snprintf(e.label, sizeof(e.label), "%s", label.c_str());
        }
        if (r.refIndex < 0 || r.refIndex >= n) r.refIndex = 0;
        r.ok = true;
        return true;
    }

    /// acc（SensorData 约定）-> roll/pitch，公式与 GUI 倾角补偿航向一致
    static void accToRollPitch(const double acc[3], double& roll, double& pitch) {
        const double aN = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (aN < 1e-9) { roll = 0.0; pitch = 0.0; return; }
        const double axn = -acc[0] / aN, ayn = -acc[1] / aN, azn = -acc[2] / aN;
        roll = std::atan2(ayn, azn);
        pitch = std::atan2(-axn, std::sqrt(ayn * ayn + azn * azn));
    }

    /// 传感器系向量 -> 调查水平系；返回航向（rad，atan2(-Yh,Xh) 约定）
    static double toLevel(const double vSensor[3], double roll, double pitch, double out[3]) {
        const double sr = std::sin(roll), cr = std::cos(roll);
        const double sp = std::sin(pitch), cp = std::cos(pitch);
        const double x = vSensor[0], y = vSensor[1], z = vSensor[2];
        out[0] =  x * cp + y * sr * sp + z * cr * sp;
        out[1] =  y * cr - z * sr;
        out[2] = -x * sp + y * sr * cp + z * cr * cp;
        return std::atan2(-out[1], out[0]);
    }

    /**
     * @brief 调查水平系的干扰 dB -> 当前传感器系。
     *
     * 先按 psi = alpha - hRaw 绕竖轴旋到传感器 heading 系（psi 是当前传感器
     * 在调查系中的方位角，由"活动格场方位角 - 实测场航向"直接给出，无需
     * 迭代），再做逆向倾斜变换（正向的转置）。
     */
    static void deltaToSensor(const double deltaLevel[3], double roll, double pitch,
                              double alphaRad, double hRawRad, double out[3]) {
        const double hv = std::hypot(deltaLevel[0], deltaLevel[1]);
        const double del = std::atan2(-deltaLevel[1], deltaLevel[0]);
        const double dp = del - (alphaRad - hRawRad);
        const double Xh = hv * std::cos(dp);
        const double Yh = -hv * std::sin(dp);
        const double Zh = deltaLevel[2];
        const double sr = std::sin(roll), cr = std::cos(roll);
        const double sp = std::sin(pitch), cp = std::cos(pitch);
        out[0] =  Xh * cp - Zh * sp;
        out[1] =  Xh * sr * sp + Yh * cr + Zh * sr * cp;
        out[2] =  Xh * cr * sp - Yh * sr + Zh * cr * cp;
    }
};

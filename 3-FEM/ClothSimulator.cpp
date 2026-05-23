#include "Labs/3-FEM/ClothSimulator.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_inverse.hpp>

namespace VCX::Labs::FEM {

    static inline float Trace2(glm::mat2 const & A) { return A[0][0] + A[1][1]; }

    static inline bool Finite3(glm::vec3 const & a) {
        return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
    }

    void ClothSimulator::SetMaterial(float young, float nu, float areaDensity) {
        m_young = std::max(1e-6f, young);
        m_nu    = std::clamp(nu, 0.0f, 0.49f);
        m_rhoA  = std::max(1e-6f, areaDensity);

        m_mu     = m_young / (2.0f * (1.0f + m_nu));
        m_lambda = (m_young * m_nu) / ((1.0f + m_nu) * (1.0f - 2.0f * m_nu));
    }

    void ClothSimulator::SetPinnedCorners(bool on) {
        m_pinCorners = on;
        ApplyPin(); 
    }

    void ClothSimulator::SetupCloth(std::size_t w, std::size_t h, float dx, glm::vec3 origin) {
        m_w  = std::max<std::size_t>(2, w);
        m_h  = std::max<std::size_t>(2, h);
        m_dx = dx;

        m_X.clear();
        m_x.clear();
        m_v.clear();
        m_UV.clear();
        m_mass.clear();
        m_fixed.clear();
        m_tris.clear();

        auto GetID = [&](std::size_t i, std::size_t j) -> int { return int(i * (m_h + 1) + j); };

        m_X.reserve((m_w + 1) * (m_h + 1));
        m_UV.reserve((m_w + 1) * (m_h + 1));
        for (std::size_t i = 0; i <= m_w; ++i)
            for (std::size_t j = 0; j <= m_h; ++j) {
                glm::vec2 uv(float(i) * m_dx, float(j) * m_dx);
                glm::vec3 X = origin + glm::vec3(uv.x, 0.0f, uv.y);
                m_UV.push_back(uv);
                m_X.push_back(X);
            }

        m_x = m_X;
        m_v.assign(m_X.size(), glm::vec3(0.0f));
        m_mass.assign(m_X.size(), 0.0f);
        m_fixed.assign(m_X.size(), false);

        BuildTris();
        PrecomputeTriData();

        for (auto const & t : m_tris) {
            float mTri = m_rhoA * t.area;
            for (int a = 0; a < 3; ++a) m_mass[(std::size_t) t.ids[a]] += mTri / 3.0f;
        }
        for (auto & mi : m_mass) mi = std::max(mi, 1e-8f);

        ApplyPin();

        m_grabVid   = GetID(m_w / 2, m_h / 2);
        m_grabForce = glm::vec3(0.0f);
    }

    void ClothSimulator::ApplyPin() {
        if (m_fixed.empty()) return; 
        std::fill(m_fixed.begin(), m_fixed.end(), false);
        if (! m_pinCorners) return;

        auto GetID = [&](std::size_t i, std::size_t j) -> int { return int(i * (m_h + 1) + j); };

        m_fixed[(std::size_t) GetID(0, m_h)]   = true;
        m_fixed[(std::size_t) GetID(m_w, m_h)] = true;
    }

    void ClothSimulator::ResetToRest() {
        m_x = m_X;
        std::fill(m_v.begin(), m_v.end(), glm::vec3(0.0f));
        m_grabForce = glm::vec3(0.0f);
    }

    void ClothSimulator::BuildTris() {
        m_tris.clear();
        m_tris.reserve(m_w * m_h * 2);

        auto GetID = [&](std::size_t i, std::size_t j) -> int { return int(i * (m_h + 1) + j); };

        for (std::size_t i = 0; i < m_w; ++i)
            for (std::size_t j = 0; j < m_h; ++j) {
                int a = GetID(i, j);
                int b = GetID(i + 1, j);
                int c = GetID(i + 1, j + 1);
                int d = GetID(i, j + 1);

                Tri t1;
                t1.ids = { a, b, c };
                m_tris.push_back(t1);
                Tri t2;
                t2.ids = { a, c, d };
                m_tris.push_back(t2);
            }
    }

    void ClothSimulator::PrecomputeTriData() {
        for (auto & t : m_tris) {
            glm::vec2 const & U1 = m_UV[(std::size_t) t.ids[0]];
            glm::vec2 const & U2 = m_UV[(std::size_t) t.ids[1]];
            glm::vec2 const & U3 = m_UV[(std::size_t) t.ids[2]];

            glm::mat2 Dm(1.0f);
            Dm[0] = U2 - U1;
            Dm[1] = U3 - U1;

            float det = glm::determinant(Dm);
            if (std::abs(det) < 1e-12f) {
                t.DmInv    = glm::mat2(1.0f);
                t.area     = 0.0f;
                t.gradN[0] = t.gradN[1] = t.gradN[2] = glm::vec2(0.0f);
                continue;
            }

            t.DmInv = glm::inverse(Dm);
            t.area  = 0.5f * std::abs(det);

            glm::mat2 DmInvT = glm::transpose(t.DmInv);
            t.gradN[1]       = DmInvT[0];
            t.gradN[2]       = DmInvT[1];
            t.gradN[0]       = -t.gradN[1] - t.gradN[2];
        }
    }

    void ClothSimulator::DeformationGradientCols(Tri const & t, glm::vec3 & f0, glm::vec3 & f1) const {
        glm::vec3 const & x1 = m_x[(std::size_t) t.ids[0]];
        glm::vec3 const & x2 = m_x[(std::size_t) t.ids[1]];
        glm::vec3 const & x3 = m_x[(std::size_t) t.ids[2]];

        glm::vec3 ds0 = x2 - x1;
        glm::vec3 ds1 = x3 - x1;

        glm::vec2 c0 = t.DmInv[0];
        glm::vec2 c1 = t.DmInv[1];

        f0 = ds0 * c0.x + ds1 * c0.y;
        f1 = ds0 * c1.x + ds1 * c1.y;
    }

    void ClothSimulator::ComputeForces(std::vector<glm::vec3> & f) {
        std::fill(f.begin(), f.end(), glm::vec3(0.0f));

        for (std::size_t i = 0; i < m_x.size(); ++i) f[i] += m_mass[i] * m_gravity;
        if (m_grabVid >= 0 && m_grabVid < (int) m_x.size()) f[(std::size_t) m_grabVid] += m_grabForce;

        glm::mat2 I2(1.0f);

        for (auto const & t : m_tris) {
            if (t.area <= 0.0f) continue;

            glm::vec3 f0, f1;
            DeformationGradientCols(t, f0, f1);

            float a00 = glm::dot(f0, f0);
            float a01 = glm::dot(f0, f1);
            float a10 = glm::dot(f1, f0);
            float a11 = glm::dot(f1, f1);

            glm::mat2 FtF(1.0f);
            FtF[0] = glm::vec2(a00, a10);
            FtF[1] = glm::vec2(a01, a11);

            glm::mat2 E = 0.5f * (FtF - I2);
            glm::mat2 S = 2.0f * m_mu * E + m_lambda * Trace2(E) * I2;

            glm::vec2 s0 = S[0];
            glm::vec2 s1 = S[1];
            glm::vec3 p0 = f0 * s0.x + f1 * s0.y;
            glm::vec3 p1 = f0 * s1.x + f1 * s1.y;

            for (int a = 0; a < 3; ++a) {
                glm::vec2 g  = t.gradN[a];
                glm::vec3 fa = -t.area * (p0 * g.x + p1 * g.y);
                f[(std::size_t) t.ids[a]] += fa;
            }
        }
    }

    void ClothSimulator::SimulateTimestep(float dt) {
        if (dt <= 0.0f) return;

        int   substeps = 10;
        float h        = dt / float(substeps);

        float vmax = 30.0f;
        float xmax = 200.0f;

        std::vector<glm::vec3> f(m_x.size());

        for (int s = 0; s < substeps; ++s) {
            ComputeForces(f);

            float velDamp = std::clamp(1.0f - m_damping, 0.0f, 1.0f);

            for (std::size_t i = 0; i < m_x.size(); ++i) {
                if (m_fixed[i]) {
                    m_x[i] = m_X[i];
                    m_v[i] = glm::vec3(0.0f);
                    continue;
                }

                glm::vec3 a = f[i] / m_mass[i];
                m_v[i] += h * a;

                float lv = glm::length(m_v[i]);
                if (lv > vmax && lv > 1e-6f) m_v[i] *= (vmax / lv);

                m_v[i] *= velDamp;
                m_x[i] += h * m_v[i];

                if (! Finite3(m_x[i]) || ! Finite3(m_v[i]) || glm::length(m_x[i]) > xmax) {
                    m_x[i] = m_X[i];
                    m_v[i] = glm::vec3(0.0f);
                }
            }
        }
    }

} // namespace VCX::Labs::FEM
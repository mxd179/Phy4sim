#include "Labs/3-FEM/FEMSimulator.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_inverse.hpp>

#include <Eigen/Dense>

namespace VCX::Labs::FEM {

    static inline float Trace(glm::mat3 const & A) {
        return A[0][0] + A[1][1] + A[2][2];
    }

    static inline bool Finite3(glm::vec3 const & a) {
        return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
    }

    static inline Eigen::Matrix3f ToEigen(glm::mat3 const & m) {
        Eigen::Matrix3f e;
        for (int c = 0; c < 3; ++c)
            for (int r = 0; r < 3; ++r)
                e(r, c) = m[c][r];
        return e;
    }

    static inline glm::mat3 ToGlm(Eigen::Matrix3f const & e) {
        glm::mat3 m(1.0f);
        for (int c = 0; c < 3; ++c)
            for (int r = 0; r < 3; ++r)
                m[c][r] = e(r, c);
        return m;
    }

    static inline glm::mat3 Cofactor(glm::mat3 const & F) {
        return glm::mat3(
            glm::cross(F[1], F[2]),
            glm::cross(F[2], F[0]),
            glm::cross(F[0], F[1]));
    }

    static inline glm::mat3 ClampF_SVD(glm::mat3 const & F, float smin, float smax) {
        Eigen::Matrix3f                   Fe = ToEigen(F);
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(Fe, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f V = svd.matrixV();
        Eigen::Vector3f s = svd.singularValues(); 

        Eigen::Matrix3f R = U * V.transpose();
        if (R.determinant() < 0.0f) {
            U.col(2) *= -1.0f;
            s(2) *= -1.0f;
        }

        for (int i = 0; i < 3; ++i) {
            float sign = (s(i) >= 0.0f) ? 1.0f : -1.0f;
            float a    = std::clamp(std::abs(s(i)), smin, smax);
            s(i)       = sign * a;
        }

        Eigen::Matrix3f Se = s.asDiagonal();
        return ToGlm(U * Se * V.transpose());
    }

    int Simulator::GetID(std::size_t i, std::size_t j, std::size_t k) const {
        return int(i * (m_wy + 1) * (m_wz + 1) + j * (m_wz + 1) + k);
    }

    void Simulator::SetMaterial(float young, float nu, float density) {
        m_young = std::max(1e-6f, young);
        m_nu    = std::clamp(nu, 0.0f, 0.49f);
        m_rho   = std::max(1e-6f, density);

        m_mu     = m_young / (2.0f * (1.0f + m_nu));
        m_lambda = (m_young * m_nu) / ((1.0f + m_nu) * (1.0f - 2.0f * m_nu));
    }

    void Simulator::EnableGround(bool on, float y) {
        m_ground.enabled = on;
        m_ground.n       = { 0, 1, 0 };
        m_ground.d       = y; 
    }

    void Simulator::EnableWallX(bool on, float x) {
        m_wallX.enabled = on;
        m_wallX.n       = { 1, 0, 0 };
        m_wallX.d       = x;
    }

    void Simulator::SetSphere(bool on, glm::vec3 c, float r) {
        m_sphere.enabled = on;
        m_sphere.center  = c;
        m_sphere.radius  = r;
    }

    void Simulator::SetAABB(bool on, glm::vec3 bmin, glm::vec3 bmax) {
        m_aabb.enabled = on;
        m_aabb.bmin    = glm::min(bmin, bmax);
        m_aabb.bmax    = glm::max(bmin, bmax);
    }

    void Simulator::SetupBox(std::size_t wx, std::size_t wy, std::size_t wz, float dx, glm::vec3 origin) {
        m_wx = std::max<std::size_t>(1, wx);
        m_wy = std::max<std::size_t>(1, wy);
        m_wz = std::max<std::size_t>(1, wz);
        m_dx = dx;

        m_X.clear();
        m_x.clear();
        m_v.clear();
        m_mass.clear();
        m_fixed.clear();
        m_tets.clear();

        m_X.reserve((m_wx + 1) * (m_wy + 1) * (m_wz + 1));
        for (std::size_t i = 0; i <= m_wx; ++i)
            for (std::size_t j = 0; j <= m_wy; ++j)
                for (std::size_t k = 0; k <= m_wz; ++k)
                    m_X.push_back(origin + glm::vec3(float(i) * m_dx, float(j) * m_dx, float(k) * m_dx));

        m_x = m_X;
        m_v.assign(m_X.size(), glm::vec3(0.0f));
        m_mass.assign(m_X.size(), 0.0f);
        m_fixed.assign(m_X.size(), false);

        BuildTets();
        PrecomputeTetData();

        for (auto const & t : m_tets) {
            float mTet = m_rho * t.volume;
            for (int a = 0; a < 4; ++a) m_mass[(std::size_t) t.ids[a]] += 0.25f * mTet;
        }
        for (auto & mi : m_mass) mi = std::max(mi, 1e-8f);
        for (std::size_t j = 0; j <= m_wy; ++j)
            for (std::size_t k = 0; k <= m_wz; ++k)
                m_fixed[(std::size_t) GetID(0, j, k)] = true;

        m_grabVid   = GetID(m_wx, m_wy / 2, m_wz / 2);
        m_grabForce = glm::vec3(0.0f);
    }

    void Simulator::ResetToRest() {
        m_x = m_X;
        std::fill(m_v.begin(), m_v.end(), glm::vec3(0.0f));
        m_grabForce = glm::vec3(0.0f);
    }

    void Simulator::BuildTets() {
        m_tets.clear();
        m_tets.reserve(m_wx * m_wy * m_wz * 6);

        for (std::size_t i = 0; i < m_wx; ++i)
            for (std::size_t j = 0; j < m_wy; ++j)
                for (std::size_t k = 0; k < m_wz; ++k) {
                    auto add = [&](int a, int b, int c, int d) {
                        Tet t;
                        t.ids = { a, b, c, d };
                        m_tets.push_back(t);
                    };

                    add(GetID(i, j, k), GetID(i, j, k + 1), GetID(i, j + 1, k + 1), GetID(i + 1, j + 1, k + 1));
                    add(GetID(i, j, k), GetID(i, j + 1, k), GetID(i, j + 1, k + 1), GetID(i + 1, j + 1, k + 1));
                    add(GetID(i, j, k), GetID(i, j, k + 1), GetID(i + 1, j, k + 1), GetID(i + 1, j + 1, k + 1));
                    add(GetID(i, j, k), GetID(i + 1, j, k), GetID(i + 1, j, k + 1), GetID(i + 1, j + 1, k + 1));
                    add(GetID(i, j, k), GetID(i, j + 1, k), GetID(i + 1, j + 1, k), GetID(i + 1, j + 1, k + 1));
                    add(GetID(i, j, k), GetID(i + 1, j, k), GetID(i + 1, j + 1, k), GetID(i + 1, j + 1, k + 1));
                }
    }

    void Simulator::PrecomputeTetData() {
        for (auto & t : m_tets) {
            auto ComputeDm = [&](glm::mat3 & Dm, float & detDm) {
                glm::vec3 const & X1 = m_X[(std::size_t) t.ids[0]];
                glm::vec3 const & X2 = m_X[(std::size_t) t.ids[1]];
                glm::vec3 const & X3 = m_X[(std::size_t) t.ids[2]];
                glm::vec3 const & X4 = m_X[(std::size_t) t.ids[3]];

                Dm    = glm::mat3(1.0f);
                Dm[0] = X2 - X1;
                Dm[1] = X3 - X1;
                Dm[2] = X4 - X1;
                detDm = glm::determinant(Dm);
            };

            glm::mat3 Dm(1.0f);
            float     detDm = 0.0f;
            ComputeDm(Dm, detDm);
            if (detDm < 0.0f) {
                std::swap(t.ids[1], t.ids[2]);
                ComputeDm(Dm, detDm);
            }

            if (std::abs(detDm) < 1e-12f) {
                t.DmInv  = glm::mat3(1.0f);
                t.volume = 0.0f;
                for (int a = 0; a < 4; ++a) t.gradN[a] = glm::vec3(0.0f);
                continue;
            }

            t.volume = detDm / 6.0f;
            t.DmInv  = glm::inverse(Dm);

            glm::mat3 DmInvT = glm::transpose(t.DmInv);
            t.gradN[1]       = DmInvT[0];
            t.gradN[2]       = DmInvT[1];
            t.gradN[3]       = DmInvT[2];
            t.gradN[0]       = -t.gradN[1] - t.gradN[2] - t.gradN[3];
        }
    }

    glm::mat3 Simulator::DeformationGradient(Tet const & t) const {
        glm::vec3 const & x1 = m_x[(std::size_t) t.ids[0]];
        glm::vec3 const & x2 = m_x[(std::size_t) t.ids[1]];
        glm::vec3 const & x3 = m_x[(std::size_t) t.ids[2]];
        glm::vec3 const & x4 = m_x[(std::size_t) t.ids[3]];

        glm::mat3 Ds(1.0f);
        Ds[0] = x2 - x1;
        Ds[1] = x3 - x1;
        Ds[2] = x4 - x1;

        return Ds * t.DmInv;
    }

    glm::mat3 Simulator::ComputeP(glm::mat3 const & Fin) const {
        glm::mat3 I(1.0f);

        switch (m_model) {
        case ElasticModel::Linear: {
            glm::mat3 eps = 0.5f * (Fin + glm::transpose(Fin)) - I;
            return 2.0f * m_mu * eps + m_lambda * Trace(eps) * I;
        }
        case ElasticModel::StVK: {
            glm::mat3 F = Fin;
            glm::mat3 C = glm::transpose(F) * F;
            glm::mat3 E = 0.5f * (C - I);
            glm::mat3 S = 2.0f * m_mu * E + m_lambda * Trace(E) * I;
            return F * S;
        }
        case ElasticModel::NeoHookean: {
            glm::mat3 F = ClampF_SVD(Fin, 0.4f, 2.5f);
            float     J = glm::determinant(F);
            J           = std::max(J, 1e-6f);

            glm::mat3 FinvT = Cofactor(F) / J;
            return m_mu * (F - FinvT) + m_lambda * (J - 1.0f) * J * FinvT;
        }
        case ElasticModel::Corotated: {
            Eigen::Matrix3f                   Fe = ToEigen(Fin);
            Eigen::JacobiSVD<Eigen::Matrix3f> svd(Fe, Eigen::ComputeFullU | Eigen::ComputeFullV);

            Eigen::Matrix3f U = svd.matrixU();
            Eigen::Matrix3f V = svd.matrixV();
            Eigen::Matrix3f R = U * V.transpose();
            if (R.determinant() < 0.0f) {
                U.col(2) *= -1.0f;
                R = U * V.transpose();
            }

            Eigen::Matrix3f RtF = R.transpose() * Fe;
            float           tr  = RtF.trace();
            Eigen::Matrix3f Pe  = 2.0f * m_mu * (Fe - R) + m_lambda * (tr - 3.0f) * R;
            return ToGlm(Pe);
        }
        default:
            return glm::mat3(0.0f);
        }
    }

    void Simulator::AddPenalty(std::vector<glm::vec3> & f, std::size_t i, float phi, glm::vec3 const & nIn) {
        if (phi >= 0.0f) return;
        glm::vec3 n = glm::normalize(nIn);
        if (! Finite3(n) || glm::length(n) < 1e-6f) return;

        float depth = std::min(-phi, m_col.maxDepth);
        float vn    = glm::dot(m_v[i], n);
        f[i] += m_col.stiffness * depth * n;
        if (vn < 0.0f) f[i] += (-m_col.damping * vn) * n;
    }

    bool Simulator::AABBPenetration(glm::vec3 const & x, float & phi, glm::vec3 & n) const {
        glm::vec3 mn = m_aabb.bmin;
        glm::vec3 mx = m_aabb.bmax;

        if (x.x < mn.x || x.x > mx.x || x.y < mn.y || x.y > mx.y || x.z < mn.z || x.z > mx.z) {
            return false;
        }

        float dx0 = x.x - mn.x;
        float dx1 = mx.x - x.x;
        float dy0 = x.y - mn.y;
        float dy1 = mx.y - x.y;
        float dz0 = x.z - mn.z;
        float dz1 = mx.z - x.z;

        float m0 = dx0;
        n        = { -1, 0, 0 };
        if (dx1 < m0) {
            m0 = dx1;
            n  = { 1, 0, 0 };
        }
        if (dy0 < m0) {
            m0 = dy0;
            n  = { 0, -1, 0 };
        }
        if (dy1 < m0) {
            m0 = dy1;
            n  = { 0, 1, 0 };
        }
        if (dz0 < m0) {
            m0 = dz0;
            n  = { 0, 0, -1 };
        }
        if (dz1 < m0) {
            m0 = dz1;
            n  = { 0, 0, 1 };
        }

        phi = -m0; 
        return true;
    }

    void Simulator::AddCollisionForces(std::vector<glm::vec3> & f) {
        bool any = m_ground.enabled || m_wallX.enabled || m_sphere.enabled || m_aabb.enabled;
        if (! any) return;

        for (std::size_t i = 0; i < m_x.size(); ++i) {
            if (m_fixed[i]) continue;

            glm::vec3 const & x = m_x[i];

            if (m_ground.enabled) {
                float phi = glm::dot(m_ground.n, x) - m_ground.d; // y - groundY
                AddPenalty(f, i, phi, m_ground.n);
            }

            if (m_wallX.enabled) {
                float phi = glm::dot(m_wallX.n, x) - m_wallX.d; // x - wallX
                AddPenalty(f, i, phi, m_wallX.n);
            }

            if (m_sphere.enabled) {
                glm::vec3 r    = x - m_sphere.center;
                float     dist = glm::length(r);
                float     phi  = dist - m_sphere.radius;
                glm::vec3 n    = (dist > 1e-6f) ? (r / dist) : glm::vec3(0, 1, 0);
                AddPenalty(f, i, phi, n);
            }

            if (m_aabb.enabled) {
                float     phi;
                glm::vec3 n;
                if (AABBPenetration(x, phi, n)) {
                    AddPenalty(f, i, phi, n);
                }
            }
        }
    }

    void Simulator::ComputeForces(std::vector<glm::vec3> & f) {
        std::fill(f.begin(), f.end(), glm::vec3(0.0f));

        for (std::size_t i = 0; i < m_x.size(); ++i)
            f[i] += m_mass[i] * m_gravity;

        if (m_grabVid >= 0 && m_grabVid < (int) m_x.size())
            f[(std::size_t) m_grabVid] += m_grabForce;

        for (auto const & t : m_tets) {
            if (t.volume <= 0.0f) continue;

            glm::mat3 F = DeformationGradient(t);
            glm::mat3 P = ComputeP(F);

            for (int a = 0; a < 4; ++a) {
                glm::vec3 fa = -t.volume * (P * t.gradN[a]);
                f[(std::size_t) t.ids[a]] += fa;
            }
        }
        AddCollisionForces(f);
    }

    void Simulator::SimulateTimestep(float dt) {
        if (dt <= 0.0f) return;

        bool collisionOn = m_ground.enabled || m_wallX.enabled || m_sphere.enabled || m_aabb.enabled;

        int substeps = 1;
        if (m_model == ElasticModel::Linear) substeps = 1;
        else if (m_model == ElasticModel::NeoHookean) substeps = 15;
        else substeps = 5;

        if (collisionOn) substeps = std::max(substeps, 10);

        substeps = std::clamp(substeps, 1, 30);
        float h  = dt / float(substeps);

        float vmax = 80.0f;
        float xmax = 500.0f;

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
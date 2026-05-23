#include "Labs/2-FluidSimulation/CaseFluid.h"
#include "Labs/Common/ImGuiHelper.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace VCX::Labs::Fluid {

    CaseFluid::CaseFluid():
        _program(Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _particleItem(
            Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Points),
        _boxLineItem(
            Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines) {
        const std::vector<std::uint32_t> line_index = {
            0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7
        };
        _boxLineItem.UpdateElementBuffer(line_index);

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        ResetScene();
    }

    void CaseFluid::ResetScene() {
        _res = std::clamp(_res, 8, 160);
        _sim.setupScene(_res);
        _sim.m_fRatio = std::clamp(_flipRatio, 0.0f, 1.0f);
        std::vector<std::uint32_t> indices(_sim.m_particlePos.size());
        std::iota(indices.begin(), indices.end(), 0u);
        _particleItem.UpdateElementBuffer(indices);
    }

    void CaseFluid::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Simulation / Control", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Simulate", &_simulate);
            ImGui::SameLine();
            if (ImGui::Button("Step")) _stepOnce = true;
            ImGui::SliderFloat("Time Step dt", &_dt, 1e-4f, 1.0f / 30.0f, "%.6f");
            ImGui::SliderFloat("flipRatio (0=PIC, 1=FLIP)", &_flipRatio, 0.0f, 1.0f, "%.3f");
            ImGui::Text("Preset: PIC=0, FLIP95=0.95, FLIP=1");

            ImGui::SliderInt("Resolution", &_res, 8, 160);
            if (ImGui::Button("Reset")) ResetScene();
        }

        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Particle Color", glm::value_ptr(_particleColor));
            ImGui::ColorEdit3("Tank Color", glm::value_ptr(_boxColor));
        }

        ImGui::Spacing();
    }

    void CaseFluid::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    Common::CaseRenderResult CaseFluid::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        float dt = std::clamp(_dt, 1e-5f, 1.0f / 10.0f);

        if (_simulate || _stepOnce) {
            _stepOnce     = false;
            _sim.m_fRatio = std::clamp(_flipRatio, 0.0f, 1.0f);
            _sim.SimulateTimestep(dt);
        }

        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(1.0f);
        std::vector<glm::vec3> boxVerts(8);
        boxVerts[0] = { -0.5f, 0.5f, 0.5f };
        boxVerts[1] = { 0.5f, 0.5f, 0.5f };
        boxVerts[2] = { 0.5f, 0.5f, -0.5f };
        boxVerts[3] = { -0.5f, 0.5f, -0.5f };
        boxVerts[4] = { -0.5f, -0.5f, 0.5f };
        boxVerts[5] = { 0.5f, -0.5f, 0.5f };
        boxVerts[6] = { 0.5f, -0.5f, -0.5f };
        boxVerts[7] = { -0.5f, -0.5f, -0.5f };

        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxLineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(boxVerts));
        _boxLineItem.Draw({ _program.Use() });
        glPointSize(3.0f);
        _program.GetUniforms().SetByName("u_Color", _particleColor);
        _particleItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_sim.m_particlePos));
        _particleItem.Draw({ _program.Use() });

        glPointSize(1.0f);
        glLineWidth(1.0f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    static inline int clampInt(int v, int lo, int hi) { return std::min(std::max(v, lo), hi); }

    int Simulator::index2GridOffset(glm::ivec3 index) {
        int n = m_iCellY * m_iCellZ;
        int m = m_iCellZ;
        return index.x * n + index.y * m + index.z;
    }

    bool Simulator::isValidVelocity(int i, int j, int k, int dir) {
        if (i < 0 || j < 0 || k < 0 || i >= m_iCellX || j >= m_iCellY || k >= m_iCellZ) return false;

        auto S = [&](int a, int b, int c) -> float {
            if (a < 0 || b < 0 || c < 0 || a >= m_iCellX || b >= m_iCellY || c >= m_iCellZ) return 0.0f;
            return m_s[index2GridOffset({ a, b, c })];
        };

        if (dir == 0) return (S(i - 1, j, k) > 0.0f) && (S(i, j, k) > 0.0f);
        if (dir == 1) return (S(i, j - 1, k) > 0.0f) && (S(i, j, k) > 0.0f);
        return (S(i, j, k - 1) > 0.0f) && (S(i, j, k) > 0.0f);
    }

    static inline glm::vec3 velGridCoord(glm::vec3 worldPos, float invH, int dir) {
        glm::vec3 gp = (worldPos + glm::vec3(0.5f)) * invH;   
        if (dir == 0) return gp - glm::vec3(0.0f, 0.5f, 0.5f); 
        if (dir == 1) return gp - glm::vec3(0.5f, 0.0f, 0.5f);
        return gp - glm::vec3(0.5f, 0.5f, 0.0f);           
    }

    static inline float component(glm::vec3 const & v, int dir) {
        return dir == 0 ? v.x : (dir == 1 ? v.y : v.z);
    }
    static inline void addToComponent(glm::vec3 & v, int dir, float a) {
        if (dir == 0) v.x += a;
        else if (dir == 1) v.y += a;
        else v.z += a;
    }

    static float sampleComponentTrilerp(
        Simulator  &              sim,
        std::vector<glm::vec3> const & gridVel,
        glm::vec3                      worldPos,
        int                            dir) {
        glm::vec3 gc = velGridCoord(worldPos, sim.m_fInvSpacing, dir);

        int i0 = (int) std::floor(gc.x);
        int j0 = (int) std::floor(gc.y);
        int k0 = (int) std::floor(gc.z);

        float fx = gc.x - (float) i0;
        float fy = gc.y - (float) j0;
        float fz = gc.z - (float) k0;

        i0 = clampInt(i0, 0, sim.m_iCellX - 2);
        j0 = clampInt(j0, 0, sim.m_iCellY - 2);
        k0 = clampInt(k0, 0, sim.m_iCellZ - 2);

        float sum = 0.0f, wsum = 0.0f;

        for (int di = 0; di <= 1; ++di)
            for (int dj = 0; dj <= 1; ++dj)
                for (int dk = 0; dk <= 1; ++dk) {
                    int   i = i0 + di, j = j0 + dj, k = k0 + dk;
                    float wx = (di ? fx : (1.0f - fx));
                    float wy = (dj ? fy : (1.0f - fy));
                    float wz = (dk ? fz : (1.0f - fz));
                    float w  = wx * wy * wz;

                    if (! sim.isValidVelocity(i, j, k, dir)) continue;

                    int idx = sim.index2GridOffset({ i, j, k });
                    sum += w * component(gridVel[idx], dir);
                    wsum += w;
                }

        return (wsum > 1e-8f) ? (sum / wsum) : 0.0f;
    }

    void Simulator::integrateParticles(float timeStep) {
        for (int p = 0; p < (int) m_particlePos.size(); ++p) {
            m_particleVel[p] += gravity * timeStep;
            m_particlePos[p] += m_particleVel[p] * timeStep;
        }
    }

    void Simulator::handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRadius, glm::vec3 obstacleVel) {
        glm::vec3 lo = glm::vec3(-0.5f + m_particleRadius);
        glm::vec3 hi = glm::vec3(0.5f - m_particleRadius);

        for (int p = 0; p < (int) m_particlePos.size(); ++p) {
            glm::vec3 & x = m_particlePos[p];
            glm::vec3 & v = m_particleVel[p];

            for (int d = 0; d < 3; ++d) {
                if (x[d] < lo[d]) {
                    x[d] = lo[d];
                    v[d] = 0.0f;
                }
                if (x[d] > hi[d]) {
                    x[d] = hi[d];
                    v[d] = 0.0f;
                }
            }

            if (obstacleRadius > 0.0f) {
                glm::vec3 r       = x - obstaclePos;
                float     dist    = glm::length(r);
                float     minDist = obstacleRadius + m_particleRadius;
                if (dist < minDist && dist > 1e-6f) {
                    glm::vec3 n = r / dist;
                    x           = obstaclePos + n * minDist;
                    v           = obstacleVel;
                }
            }
        }
    }

    void Simulator::pushParticlesApart(int numIters) {
        if (m_particlePos.empty()) return;

        std::vector<int> counts(m_iNumCells, 0);
        std::vector<int> pidSorted(m_particlePos.size(), 0);

        auto particleCell = [&](glm::vec3 const & x) -> glm::ivec3 {
            glm::vec3 g = (x + glm::vec3(0.5f)) * m_fInvSpacing;
            return glm::ivec3(
                clampInt((int) std::floor(g.x), 0, m_iCellX - 1),
                clampInt((int) std::floor(g.y), 0, m_iCellY - 1),
                clampInt((int) std::floor(g.z), 0, m_iCellZ - 1));
        };

        for (int it = 0; it < numIters; ++it) {
            std::fill(counts.begin(), counts.end(), 0);

            for (int p = 0; p < (int) m_particlePos.size(); ++p) {
                glm::ivec3 c   = particleCell(m_particlePos[p]);
                int        h   = index2GridOffset(c);
                m_hashtable[p] = h;
                counts[h]++;
            }

            m_hashtableindex[0] = 0;
            for (int c = 0; c < m_iNumCells; ++c) m_hashtableindex[c + 1] = m_hashtableindex[c] + counts[c];

            std::vector<int> offsets = m_hashtableindex;
            for (int p = 0; p < (int) m_particlePos.size(); ++p) {
                int h                   = m_hashtable[p];
                pidSorted[offsets[h]++] = p;
            }

            float minDist = 2.0f * m_particleRadius;

            for (int p = 0; p < (int) m_particlePos.size(); ++p) {
                glm::ivec3 c = particleCell(m_particlePos[p]);

                for (int di = -1; di <= 1; ++di)
                    for (int dj = -1; dj <= 1; ++dj)
                        for (int dk = -1; dk <= 1; ++dk) {
                            glm::ivec3 nc = c + glm::ivec3(di, dj, dk);
                            if (nc.x < 0 || nc.y < 0 || nc.z < 0 || nc.x >= m_iCellX || nc.y >= m_iCellY || nc.z >= m_iCellZ) continue;
                            int h = index2GridOffset(nc);

                            int begin = m_hashtableindex[h];
                            int end   = m_hashtableindex[h + 1];
                            for (int t = begin; t < end; ++t) {
                                int q = pidSorted[t];
                                if (q <= p) continue;

                                glm::vec3 d    = m_particlePos[q] - m_particlePos[p];
                                float     dist = glm::length(d);
                                if (dist < 1e-6f || dist >= minDist) continue;

                                glm::vec3 n    = d / dist;
                                float     corr = 0.5f * (minDist - dist);
                                m_particlePos[p] -= n * corr;
                                m_particlePos[q] += n * corr;
                            }
                        }
            }
        }
    }

    void Simulator::updateParticleDensity() {
        std::fill(m_particleDensity.begin(), m_particleDensity.end(), 0.0f);

        constexpr int EMPTY_CELL = 0;
        constexpr int FLUID_CELL = 1;
        constexpr int SOLID_CELL = 2;

        for (int idx = 0; idx < m_iNumCells; ++idx)
            m_type[idx] = (m_s[idx] == 0.0f) ? SOLID_CELL : EMPTY_CELL;

        for (int p = 0; p < (int) m_particlePos.size(); ++p) {
            glm::vec3 gp = (m_particlePos[p] + glm::vec3(0.5f)) * m_fInvSpacing - glm::vec3(0.5f);

            int i0 = (int) std::floor(gp.x);
            int j0 = (int) std::floor(gp.y);
            int k0 = (int) std::floor(gp.z);

            float fx = gp.x - (float) i0;
            float fy = gp.y - (float) j0;
            float fz = gp.z - (float) k0;

            i0 = clampInt(i0, 0, m_iCellX - 2);
            j0 = clampInt(j0, 0, m_iCellY - 2);
            k0 = clampInt(k0, 0, m_iCellZ - 2);

            for (int di = 0; di <= 1; ++di)
                for (int dj = 0; dj <= 1; ++dj)
                    for (int dk = 0; dk <= 1; ++dk) {
                        int i = i0 + di, j = j0 + dj, k = k0 + dk;

                        float wx = (di ? fx : (1.0f - fx));
                        float wy = (dj ? fy : (1.0f - fy));
                        float wz = (dk ? fz : (1.0f - fz));
                        float w  = wx * wy * wz;

                        int idx = index2GridOffset({ i, j, k });
                        if (m_s[idx] == 0.0f) continue;
                        m_particleDensity[idx] += w;
                    }
        }

        for (int idx = 0; idx < m_iNumCells; ++idx) {
            if (m_s[idx] == 0.0f) continue;
            if (m_particleDensity[idx] > 1e-6f) m_type[idx] = FLUID_CELL;
        }

        if (m_particleRestDensity <= 0.0f) {
            double sum = 0.0;
            int    cnt = 0;
            for (int idx = 0; idx < m_iNumCells; ++idx) {
                if (m_type[idx] == FLUID_CELL) {
                    sum += m_particleDensity[idx];
                    cnt++;
                }
            }
            m_particleRestDensity = (cnt > 0) ? float(sum / double(cnt)) : 1.0f;
        }
    }

    void Simulator::updateParticleColors() {
    }

    void Simulator::transferVelocities(bool toGrid, float flipRatio) {
        flipRatio = std::clamp(flipRatio, 0.0f, 1.0f);

        if (toGrid) {
            m_pre_vel = m_vel;

            std::fill(m_vel.begin(), m_vel.end(), glm::vec3(0.0f));
            for (int d = 0; d < 3; ++d) std::fill(m_near_num[d].begin(), m_near_num[d].end(), 0.0f);

            for (int p = 0; p < (int) m_particlePos.size(); ++p) {
                glm::vec3 x = m_particlePos[p];
                glm::vec3 v = m_particleVel[p];

                for (int dir = 0; dir < 3; ++dir) {
                    glm::vec3 gc = velGridCoord(x, m_fInvSpacing, dir);

                    int i0 = (int) std::floor(gc.x);
                    int j0 = (int) std::floor(gc.y);
                    int k0 = (int) std::floor(gc.z);

                    float fx = gc.x - (float) i0;
                    float fy = gc.y - (float) j0;
                    float fz = gc.z - (float) k0;

                    i0 = clampInt(i0, 0, m_iCellX - 2);
                    j0 = clampInt(j0, 0, m_iCellY - 2);
                    k0 = clampInt(k0, 0, m_iCellZ - 2);

                    for (int di = 0; di <= 1; ++di)
                        for (int dj = 0; dj <= 1; ++dj)
                            for (int dk = 0; dk <= 1; ++dk) {
                                int i = i0 + di, j = j0 + dj, k = k0 + dk;

                                float wx = (di ? fx : (1.0f - fx));
                                float wy = (dj ? fy : (1.0f - fy));
                                float wz = (dk ? fz : (1.0f - fz));
                                float w  = wx * wy * wz;

                                if (! isValidVelocity(i, j, k, dir)) continue;

                                int idx = index2GridOffset({ i, j, k });
                                addToComponent(m_vel[idx], dir, w * component(v, dir));
                                m_near_num[dir][idx] += w;
                            }
                }
            }

            for (int idx = 0; idx < m_iNumCells; ++idx) {
                m_vel[idx].x = (m_near_num[0][idx] > 1e-8f) ? (m_vel[idx].x / m_near_num[0][idx]) : 0.0f;
                m_vel[idx].y = (m_near_num[1][idx] > 1e-8f) ? (m_vel[idx].y / m_near_num[1][idx]) : 0.0f;
                m_vel[idx].z = (m_near_num[2][idx] > 1e-8f) ? (m_vel[idx].z / m_near_num[2][idx]) : 0.0f;
            }

            for (int i = 0; i < m_iCellX; ++i)
                for (int j = 0; j < m_iCellY; ++j)
                    for (int k = 0; k < m_iCellZ; ++k) {
                        int idx = index2GridOffset({ i, j, k });
                        if (! isValidVelocity(i, j, k, 0)) m_vel[idx].x = 0.0f;
                        if (! isValidVelocity(i, j, k, 1)) m_vel[idx].y = 0.0f;
                        if (! isValidVelocity(i, j, k, 2)) m_vel[idx].z = 0.0f;
                    }
        } else {
            for (int p = 0; p < (int) m_particlePos.size(); ++p) {
                glm::vec3 x = m_particlePos[p];

                glm::vec3 picVel(0.0f);
                picVel.x = sampleComponentTrilerp(*this, m_vel, x, 0);
                picVel.y = sampleComponentTrilerp(*this, m_vel, x, 1);
                picVel.z = sampleComponentTrilerp(*this, m_vel, x, 2);

                glm::vec3 delta(0.0f);
                delta.x = sampleComponentTrilerp(*this, m_vel, x, 0) - sampleComponentTrilerp(*this, m_pre_vel, x, 0);
                delta.y = sampleComponentTrilerp(*this, m_vel, x, 1) - sampleComponentTrilerp(*this, m_pre_vel, x, 1);
                delta.z = sampleComponentTrilerp(*this, m_vel, x, 2) - sampleComponentTrilerp(*this, m_pre_vel, x, 2);

                glm::vec3 flipVel = m_particleVel[p] + delta;

                m_particleVel[p] = (1.0f - flipRatio) * picVel + flipRatio * flipVel;
            }
        }
    }

    void Simulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
        if (dt <= 0.0f) return;

        constexpr int SOLID_CELL = 2;
        std::fill(m_p.begin(), m_p.end(), 0.0f);

        float invH  = m_fInvSpacing;
        float h     = m_h;
        float omega = 1.0f + std::clamp(overRelaxation, 0.0f, 1.9f);

        auto P = [&](int i, int j, int k) -> float & {
            return m_p[index2GridOffset({ i, j, k })];
        };
        auto S = [&](int i, int j, int k) -> float {
            if (i < 0 || j < 0 || k < 0 || i >= m_iCellX || j >= m_iCellY || k >= m_iCellZ) return 0.0f;
            return m_s[index2GridOffset({ i, j, k })];
        };

        for (int it = 0; it < numIters; ++it) {
            for (int i = 1; i < m_iCellX - 1; ++i)
                for (int j = 1; j < m_iCellY - 1; ++j)
                    for (int k = 1; k < m_iCellZ - 1; ++k) {
                        int idx = index2GridOffset({ i, j, k });
                        if (m_type[idx] == SOLID_CELL || m_s[idx] == 0.0f) continue;

                        float uR = m_vel[index2GridOffset({ i + 1, j, k })].x;
                        float uL = m_vel[index2GridOffset({ i, j, k })].x;
                        float vT = m_vel[index2GridOffset({ i, j + 1, k })].y;
                        float vB = m_vel[index2GridOffset({ i, j, k })].y;
                        float wF = m_vel[index2GridOffset({ i, j, k + 1 })].z;
                        float wB = m_vel[index2GridOffset({ i, j, k })].z;

                        float div = (uR - uL + vT - vB + wF - wB) * invH;

                        if (compensateDrift && m_particleRestDensity > 1e-6f) {
                            float err = (m_particleDensity[idx] - m_particleRestDensity) / m_particleRestDensity;
                            if (err > 0.0f) div -= err * 0.1f / dt;
                        }

                        float aL = S(i - 1, j, k), aR = S(i + 1, j, k);
                        float aB = S(i, j - 1, k), aT = S(i, j + 1, k);
                        float aD = S(i, j, k - 1), aU = S(i, j, k + 1);

                        float diag = aL + aR + aB + aT + aD + aU;
                        if (diag < 1e-6f) continue;

                        float sumN =
                            aL * P(i - 1, j, k) + aR * P(i + 1, j, k) + aB * P(i, j - 1, k) + aT * P(i, j + 1, k) + aD * P(i, j, k - 1) + aU * P(i, j, k + 1);

                        float b    = div * (h * h) / dt;
                        float pNew = (sumN - b) / diag;

                        float pOld = P(i, j, k);
                        P(i, j, k) = pOld + omega * (pNew - pOld);
                    }
        }

        float scale = dt * invH;

        for (int i = 0; i < m_iCellX; ++i)
            for (int j = 0; j < m_iCellY; ++j)
                for (int k = 0; k < m_iCellZ; ++k) {
                    int idx = index2GridOffset({ i, j, k });

                    if (! isValidVelocity(i, j, k, 0)) m_vel[idx].x = 0.0f;
                    else m_vel[idx].x -= scale * (P(i, j, k) - P(i - 1, j, k));

                    if (! isValidVelocity(i, j, k, 1)) m_vel[idx].y = 0.0f;
                    else m_vel[idx].y -= scale * (P(i, j, k) - P(i, j - 1, k));

                    if (! isValidVelocity(i, j, k, 2)) m_vel[idx].z = 0.0f;
                    else m_vel[idx].z -= scale * (P(i, j, k) - P(i, j, k - 1));
                }
    }

} // namespace VCX::Labs::Fluid

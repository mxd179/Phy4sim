#include "Labs/2-FluidSimulation/Bonus2.h"
#include "Labs/Common/ImGuiHelper.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace VCX::Labs::Fluid {

    static inline int clampInt(int v, int lo, int hi) { return std::min(std::max(v, lo), hi); }

    static inline float getComp(glm::vec3 const & v, int dir) {
        return dir == 0 ? v.x : (dir == 1 ? v.y : v.z);
    }
    static inline void addComp(glm::vec3 & v, int dir, float a) {
        if (dir == 0) v.x += a;
        else if (dir == 1) v.y += a;
        else v.z += a;
    }

    static inline glm::vec3 velGridCoord(glm::vec3 const & worldPos, float invH, int dir) {
        glm::vec3 gp = (worldPos + glm::vec3(0.5f)) * invH;   
        if (dir == 0) return gp - glm::vec3(0.0f, 0.5f, 0.5f); 
        if (dir == 1) return gp - glm::vec3(0.5f, 0.0f, 0.5f);
        return gp - glm::vec3(0.5f, 0.5f, 0.0f);              
    }

    static inline glm::vec3 faceWorldPos(Simulator const & sim, int i, int j, int k, int dir) {
        float h = sim.m_h;
        if (dir == 0) { 
            return glm::vec3(-0.5f + i * h, -0.5f + (j + 0.5f) * h, -0.5f + (k + 0.5f) * h);
        }
        if (dir == 1) { 
            return glm::vec3(-0.5f + (i + 0.5f) * h, -0.5f + j * h, -0.5f + (k + 0.5f) * h);
        }
        return glm::vec3(-0.5f + (i + 0.5f) * h, -0.5f + (j + 0.5f) * h, -0.5f + k * h);
    }

    static float sampleComponentTrilerp(
        Simulator &              sim,
        std::vector<glm::vec3> const & gridVel,
        glm::vec3 const &              worldPos,
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
                    int i = i0 + di, j = j0 + dj, k = k0 + dk;

                    float wx = di ? fx : (1.0f - fx);
                    float wy = dj ? fy : (1.0f - fy);
                    float wz = dk ? fz : (1.0f - fz);
                    float w  = wx * wy * wz;

                    if (! sim.isValidVelocity(i, j, k, dir)) continue;

                    int idx = sim.index2GridOffset({ i, j, k });
                    sum += w * getComp(gridVel[idx], dir);
                    wsum += w;
                }

        return (wsum > 1e-8f) ? (sum / wsum) : 0.0f;
    }


    Bonus2::Bonus2():
        _program(Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _particleItem(
            Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Points),
        _tankLineItem(
            Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines) {
        const std::vector<std::uint32_t> cubeLines = {
            0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7
        };
        _tankLineItem.UpdateElementBuffer(cubeLines);

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        ResetScene();
    }

    void Bonus2::ResetScene() {
        _res = std::clamp(_res, 8, 160);
        _sim.setupScene(_res);
        std::vector<std::uint32_t> indices(_sim.m_particlePos.size());
        std::iota(indices.begin(), indices.end(), 0u);
        _particleItem.UpdateElementBuffer(indices);
        _apicA.assign(_sim.m_particlePos.size(), { glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f) });
    }

    void Bonus2::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Simulation / Control", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Simulate", &_simulate);
            ImGui::SameLine();
            if (ImGui::Button("Step")) _stepOnce = true;

            ImGui::SliderFloat("Time Step dt", &_dt, 1e-4f, 1.0f / 30.0f, "%.6f");
            ImGui::SliderInt("Resolution", &_res, 8, 160);

            const char * methods[] = { "PIC", "FLIP95", "APIC" };
            int          m         = (int) _method;
            if (ImGui::Combo("Method", &m, methods, 3)) _method = (Method) m;

            if (_method == Method::FLIP95) {
                ImGui::SliderFloat("flipRatio", &_flipRatio, 0.0f, 1.0f, "%.3f");
            }

            ImGui::SliderFloat("Point Size", &_pointSize, 1.0f, 10.0f);

            if (ImGui::Button("Reset")) ResetScene();
        }

        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Tank Color", glm::value_ptr(_tankColor));
            ImGui::ColorEdit3("Particle Color", glm::value_ptr(_particleColor));
        }

        ImGui::Spacing();
    }

    void Bonus2::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void Bonus2::Step(float dt) {
        int   numSubSteps       = 1;
        int   numParticleIters  = 5;
        int   numPressureIters  = 30;
        bool  separateParticles = true;
        float overRelaxation    = 0.5f;
        bool  compensateDrift   = true;

        float sdt = dt / float(numSubSteps);

        for (int step = 0; step < numSubSteps; ++step) {
            _sim.integrateParticles(sdt);
            _sim.handleParticleCollisions(glm::vec3(0.0f), 0.0f, glm::vec3(0.0f));

            if (separateParticles) _sim.pushParticlesApart(numParticleIters);

            _sim.handleParticleCollisions(glm::vec3(0.0f), 0.0f, glm::vec3(0.0f));

            if (_method == Method::APIC) {
                APIC_ToGrid();
                _sim.updateParticleDensity();
                _sim.solveIncompressibility(numPressureIters, sdt, overRelaxation, compensateDrift);
                APIC_ToParticles();
            } else {
                float flip = (_method == Method::PIC) ? 0.0f : std::clamp(_flipRatio, 0.0f, 1.0f);
                _sim.transferVelocities(true, flip);
                _sim.updateParticleDensity();
                _sim.solveIncompressibility(numPressureIters, sdt, overRelaxation, compensateDrift);
                _sim.transferVelocities(false, flip);
            }
        }
    }

    void Bonus2::APIC_ToGrid() {
        _sim.m_pre_vel = _sim.m_vel;

        std::fill(_sim.m_vel.begin(), _sim.m_vel.end(), glm::vec3(0.0f));
        for (int d = 0; d < 3; ++d)
            std::fill(_sim.m_near_num[d].begin(), _sim.m_near_num[d].end(), 0.0f);

        for (int p = 0; p < (int) _sim.m_particlePos.size(); ++p) {
            glm::vec3 const xp = _sim.m_particlePos[p];
            glm::vec3 const vp = _sim.m_particleVel[p];

            for (int dir = 0; dir < 3; ++dir) {
                glm::vec3 gc = velGridCoord(xp, _sim.m_fInvSpacing, dir);

                int i0 = (int) std::floor(gc.x);
                int j0 = (int) std::floor(gc.y);
                int k0 = (int) std::floor(gc.z);

                float fx = gc.x - (float) i0;
                float fy = gc.y - (float) j0;
                float fz = gc.z - (float) k0;

                i0 = clampInt(i0, 0, _sim.m_iCellX - 2);
                j0 = clampInt(j0, 0, _sim.m_iCellY - 2);
                k0 = clampInt(k0, 0, _sim.m_iCellZ - 2);

                for (int di = 0; di <= 1; ++di)
                    for (int dj = 0; dj <= 1; ++dj)
                        for (int dk = 0; dk <= 1; ++dk) {
                            int i = i0 + di, j = j0 + dj, k = k0 + dk;

                            float wx = di ? fx : (1.0f - fx);
                            float wy = dj ? fy : (1.0f - fy);
                            float wz = dk ? fz : (1.0f - fz);
                            float w  = wx * wy * wz;

                            if (! _sim.isValidVelocity(i, j, k, dir)) continue;

                            glm::vec3 xi = faceWorldPos(_sim, i, j, k, dir);
                            glm::vec3 r  = xi - xp;

                            float vAff = getComp(vp, dir) + glm::dot(_apicA[p][dir], r);

                            int idx = _sim.index2GridOffset({ i, j, k });
                            addComp(_sim.m_vel[idx], dir, w * vAff);
                            _sim.m_near_num[dir][idx] += w;
                        }
            }
        }

        for (int idx = 0; idx < _sim.m_iNumCells; ++idx) {
            _sim.m_vel[idx].x = (_sim.m_near_num[0][idx] > 1e-8f) ? (_sim.m_vel[idx].x / _sim.m_near_num[0][idx]) : 0.0f;
            _sim.m_vel[idx].y = (_sim.m_near_num[1][idx] > 1e-8f) ? (_sim.m_vel[idx].y / _sim.m_near_num[1][idx]) : 0.0f;
            _sim.m_vel[idx].z = (_sim.m_near_num[2][idx] > 1e-8f) ? (_sim.m_vel[idx].z / _sim.m_near_num[2][idx]) : 0.0f;
        }

        for (int i = 0; i < _sim.m_iCellX; ++i)
            for (int j = 0; j < _sim.m_iCellY; ++j)
                for (int k = 0; k < _sim.m_iCellZ; ++k) {
                    int idx = _sim.index2GridOffset({ i, j, k });
                    if (! _sim.isValidVelocity(i, j, k, 0)) _sim.m_vel[idx].x = 0.0f;
                    if (! _sim.isValidVelocity(i, j, k, 1)) _sim.m_vel[idx].y = 0.0f;
                    if (! _sim.isValidVelocity(i, j, k, 2)) _sim.m_vel[idx].z = 0.0f;
                }
    }

    void Bonus2::APIC_ToParticles() {
        float const eps = 1e-6f;

        for (int p = 0; p < (int) _sim.m_particlePos.size(); ++p) {
            glm::vec3 const xp = _sim.m_particlePos[p];
            glm::vec3 vNew(0.0f);
            vNew.x                = sampleComponentTrilerp(_sim, _sim.m_vel, xp, 0);
            vNew.y                = sampleComponentTrilerp(_sim, _sim.m_vel, xp, 1);
            vNew.z                = sampleComponentTrilerp(_sim, _sim.m_vel, xp, 2);
            _sim.m_particleVel[p] = vNew;

            for (int dir = 0; dir < 3; ++dir) {
                glm::vec3 gc = velGridCoord(xp, _sim.m_fInvSpacing, dir);

                int i0 = (int) std::floor(gc.x);
                int j0 = (int) std::floor(gc.y);
                int k0 = (int) std::floor(gc.z);

                float fx = gc.x - (float) i0;
                float fy = gc.y - (float) j0;
                float fz = gc.z - (float) k0;

                i0 = clampInt(i0, 0, _sim.m_iCellX - 2);
                j0 = clampInt(j0, 0, _sim.m_iCellY - 2);
                k0 = clampInt(k0, 0, _sim.m_iCellZ - 2);

                glm::mat3 D(0.0f);
                glm::vec3 m(0.0f); 

                for (int di = 0; di <= 1; ++di)
                    for (int dj = 0; dj <= 1; ++dj)
                        for (int dk = 0; dk <= 1; ++dk) {
                            int i = i0 + di, j = j0 + dj, k = k0 + dk;

                            float wx = di ? fx : (1.0f - fx);
                            float wy = dj ? fy : (1.0f - fy);
                            float wz = dk ? fz : (1.0f - fz);
                            float w  = wx * wy * wz;

                            if (! _sim.isValidVelocity(i, j, k, dir)) continue;

                            int   idx = _sim.index2GridOffset({ i, j, k });
                            float vi  = getComp(_sim.m_vel[idx], dir);

                            glm::vec3 xi = faceWorldPos(_sim, i, j, k, dir);
                            glm::vec3 r  = xi - xp;
                            D += w * glm::outerProduct(r, r);
                            m += w * vi * r;
                        }
                D[0][0] += eps;
                D[1][1] += eps;
                D[2][2] += eps;

                glm::mat3 invD = glm::inverse(D);
                glm::vec3 a = glm::transpose(invD) * m;

                _apicA[p][dir] = a;
            }
        }
    }

    Common::CaseRenderResult Bonus2::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        float dt = std::clamp(_dt, 1e-5f, 1.0f / 10.0f);

        if (_simulate || _stepOnce) {
            _stepOnce = false;
            if (_res != _sim.m_iCellY - 1) {
                ResetScene();
            }
            Step(dt);
        }

        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        std::vector<glm::vec3> tankVerts(8);
        tankVerts[0] = { -0.5f, 0.5f, 0.5f };
        tankVerts[1] = { 0.5f, 0.5f, 0.5f };
        tankVerts[2] = { 0.5f, 0.5f, -0.5f };
        tankVerts[3] = { -0.5f, 0.5f, -0.5f };
        tankVerts[4] = { -0.5f, -0.5f, 0.5f };
        tankVerts[5] = { 0.5f, -0.5f, 0.5f };
        tankVerts[6] = { 0.5f, -0.5f, -0.5f };
        tankVerts[7] = { -0.5f, -0.5f, -0.5f };

        _program.GetUniforms().SetByName("u_Color", _tankColor);
        _tankLineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(tankVerts));
        _tankLineItem.Draw({ _program.Use() });
        glPointSize(_pointSize);
        _program.GetUniforms().SetByName("u_Color", _particleColor);
        _particleItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_sim.m_particlePos));
        _particleItem.Draw({ _program.Use() });

        glPointSize(1.0f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

} // namespace VCX::Labs::Fluid
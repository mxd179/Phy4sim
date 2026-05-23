#include "Labs/2-FluidSimulation/Bonus1.h"
#include "Labs/Common/ImGuiHelper.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/type_ptr.hpp>

namespace VCX::Labs::Fluid {

    static constexpr float PI = 3.1415926f;

    static glm::vec3 ColorMapBlueGreenRed(float t) {
        t = std::clamp(t, 0.0f, 1.0f);
        glm::vec3 blue { 0.05f, 0.25f, 1.00f };
        glm::vec3 green { 0.10f, 1.00f, 0.35f };
        glm::vec3 red { 1.00f, 0.15f, 0.05f };
        if (t < 0.5f) return glm::mix(blue, green, t * 2.0f);
        return glm::mix(green, red, (t - 0.5f) * 2.0f);
    }

    static std::vector<glm::vec3> BuildWireSphere(glm::vec3 center, float radius, int seg = 48) {
        std::vector<glm::vec3> verts;
        verts.reserve(3 * seg * 2);

        auto addCircle = [&](int axis) {
            for (int i = 0; i < seg; ++i) {
                float a0 = 2.0f * PI * float(i) / float(seg);
                float a1 = 2.0f * PI * float(i + 1) / float(seg);

                glm::vec3 p0(0.0f), p1(0.0f);
                if (axis == 0) { // YZ
                    p0 = { 0.0f, std::cos(a0), std::sin(a0) };
                    p1 = { 0.0f, std::cos(a1), std::sin(a1) };
                } else if (axis == 1) { // XZ
                    p0 = { std::cos(a0), 0.0f, std::sin(a0) };
                    p1 = { std::cos(a1), 0.0f, std::sin(a1) };
                } else { // XY
                    p0 = { std::cos(a0), std::sin(a0), 0.0f };
                    p1 = { std::cos(a1), std::sin(a1), 0.0f };
                }

                verts.push_back(center + radius * p0);
                verts.push_back(center + radius * p1);
            }
        };

        addCircle(0);
        addCircle(1);
        addCircle(2);
        return verts;
    }

    static int NearestCellIndexForParticle(Simulator const & sim, glm::vec3 const & x) {
        glm::vec3 g = (x + glm::vec3(0.5f)) * sim.m_fInvSpacing; 
        int       i = std::clamp(int(std::floor(g.x)), 0, sim.m_iCellX - 1);
        int       j = std::clamp(int(std::floor(g.y)), 0, sim.m_iCellY - 1);
        int       k = std::clamp(int(std::floor(g.z)), 0, sim.m_iCellZ - 1);
        return i * sim.m_iCellY * sim.m_iCellZ + j * sim.m_iCellZ + k;
    }

    Bonus1::Bonus1():
        _flatProgram(Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _particleProgram(Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/particle.vert"), Engine::GL::SharedShader("assets/shaders/particle.frag") })),
        _particleItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0)
                .Add<glm::vec3>("color", Engine::GL::DrawFrequency::Stream, 1),
            Engine::GL::PrimitiveType::Points),
        _tankLineItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines),
        _sphereLineItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines) {
        const std::vector<std::uint32_t> cubeLines = {
            0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7
        };
        _tankLineItem.UpdateElementBuffer(cubeLines);
        constexpr int              seg             = 48;
        const int                  sphereVertCount = 3 * seg * 2;
        std::vector<std::uint32_t> sphereIdx(sphereVertCount);
        std::iota(sphereIdx.begin(), sphereIdx.end(), 0u);
        _sphereLineItem.UpdateElementBuffer(sphereIdx);

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        ResetScene();
    }

    void Bonus1::ResetScene() {
        _res = std::clamp(_res, 8, 160);
        _sim.setupScene(_res);

        if (_sim.m_particleColor.size() != _sim.m_particlePos.size())
            _sim.m_particleColor.resize(_sim.m_particlePos.size(), glm::vec3(1.0f));

        std::vector<std::uint32_t> indices(_sim.m_particlePos.size());
        std::iota(indices.begin(), indices.end(), 0u);
        _particleItem.UpdateElementBuffer(indices);

        _obstaclePos = glm::vec3(0.0f, 0.0f, 0.0f);
        _obstacleVel = glm::vec3(0.0f);
        _dragging    = false;

        UpdateParticleColorsBonus();
    }

    void Bonus1::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Simulation / Control", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Simulate", &_simulate);
            ImGui::SameLine();
            if (ImGui::Button("Step")) _stepOnce = true;

            ImGui::SliderFloat("Time Step dt", &_dt, 1e-4f, 1.0f / 30.0f, "%.6f");
            ImGui::SliderFloat("flipRatio (0=PIC, 1=FLIP)", &_flipRatio, 0.0f, 1.0f, "%.3f");
            ImGui::SliderInt("Resolution", &_res, 8, 160);
            if (ImGui::Button("Reset")) ResetScene();
        }

        if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen)) {
            const char * modes[] = { "Velocity Magnitude", "Density", "Pressure" };
            ImGui::Combo("Particle Coloring", &_colorMode, modes, 3);
            ImGui::SliderFloat("Point Size", &_pointSize, 1.0f, 10.0f);
        }

        if (ImGui::CollapsingHeader("Mouse Sphere Obstacle", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Enable Obstacle", &_enableObstacle);
            ImGui::SliderFloat("Obstacle Radius", &_obstacleRadius, 0.02f, 0.25f);
            ImGui::ColorEdit3("Obstacle Color", glm::value_ptr(_obstacleColor));
            ImGui::Text("Drag: Hold Shift + Left Mouse Button");
        }

        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Tank Color", glm::value_ptr(_tankColor));
        }

        ImGui::Spacing();
    }

    void Bonus1::OnProcessInput(ImVec2 const & pos) {
        bool wantDrag = _enableObstacle && ImGui::IsKeyDown(ImGuiKey_LeftShift) && ImGui::IsMouseDown(ImGuiMouseButton_Left);

        if (! wantDrag) {
            _dragging    = false;
            _obstacleVel = glm::vec3(0.0f);
            _cameraManager.ProcessInput(_camera, pos);
            return;
        }

        std::uint32_t W      = std::max<std::uint32_t>(_lastRenderSize.first, 1);
        std::uint32_t H      = std::max<std::uint32_t>(_lastRenderSize.second, 1);
        float         aspect = float(W) / float(H);

        glm::mat4 proj = _camera.GetProjectionMatrix(aspect);
        glm::mat4 view = _camera.GetViewMatrix();
        glm::vec4 viewport(0.0f, 0.0f, float(W), float(H));

        glm::vec3 winNear(pos.x, float(H) - pos.y, 0.0f);
        glm::vec3 winFar(pos.x, float(H) - pos.y, 1.0f);

        glm::vec3 rayNear = glm::unProject(winNear, view, proj, viewport);
        glm::vec3 rayFar  = glm::unProject(winFar, view, proj, viewport);
        glm::vec3 rayDir  = glm::normalize(rayFar - rayNear);

        glm::vec3 planePoint  = _obstaclePos;
        glm::vec3 planeNormal = glm::normalize(_camera.Eye - _obstaclePos);

        float denom = glm::dot(rayDir, planeNormal);
        if (std::abs(denom) < 1e-6f) return;

        float t = glm::dot(planePoint - rayNear, planeNormal) / denom;
        if (t < 0.0f) return;

        glm::vec3 newPos = rayNear + t * rayDir;

        float margin = _obstacleRadius;
        newPos.x     = std::clamp(newPos.x, -0.5f + margin, 0.5f - margin);
        newPos.y     = std::clamp(newPos.y, -0.5f + margin, 0.5f - margin);
        newPos.z     = std::clamp(newPos.z, -0.5f + margin, 0.5f - margin);

        float     frameDt = std::max(ImGui::GetIO().DeltaTime, 1e-5f);
        glm::vec3 vel     = (newPos - _obstaclePos) / frameDt;

        float maxVel = 8.0f;
        float len    = glm::length(vel);
        if (len > maxVel) vel = vel / len * maxVel;

        _obstacleVel = vel;
        _obstaclePos = newPos;
        _dragging    = true;
    }

    void Bonus1::StepFluid(float dt) {
        int   numSubSteps       = 1;
        int   numParticleIters  = 5;
        int   numPressureIters  = 30;
        bool  separateParticles = true;
        float overRelaxation    = 0.5f;
        bool  compensateDrift   = true;

        float flipRatio = std::clamp(_flipRatio, 0.0f, 1.0f);

        float sdt = dt / float(numSubSteps);

        float     obsR = _enableObstacle ? _obstacleRadius : 0.0f;
        glm::vec3 obsP = _obstaclePos;
        glm::vec3 obsV = _obstacleVel;

        for (int step = 0; step < numSubSteps; ++step) {
            _sim.integrateParticles(sdt);
            _sim.handleParticleCollisions(obsP, obsR, obsV);

            if (separateParticles) _sim.pushParticlesApart(numParticleIters);

            _sim.handleParticleCollisions(obsP, obsR, obsV);

            _sim.transferVelocities(true, flipRatio);
            _sim.updateParticleDensity();
            _sim.solveIncompressibility(numPressureIters, sdt, overRelaxation, compensateDrift);
            _sim.transferVelocities(false, flipRatio);
        }
    }

    void Bonus1::UpdateParticleColorsBonus() {
        if (_sim.m_particleColor.size() != _sim.m_particlePos.size())
            _sim.m_particleColor.resize(_sim.m_particlePos.size(), glm::vec3(1.0f));

        float maxAbsP = 1e-6f;
        for (float p : _sim.m_p) maxAbsP = std::max(maxAbsP, std::abs(p));

        float rest = std::max(_sim.m_particleRestDensity, 1e-6f);

        for (int pid = 0; pid < int(_sim.m_particlePos.size()); ++pid) {
            glm::vec3 c(1.0f);

            if (_colorMode == 0) { 
                float spd = glm::length(_sim.m_particleVel[pid]);
                float t   = std::clamp(spd / 4.0f, 0.0f, 1.0f);
                c         = ColorMapBlueGreenRed(t);
            } else if (_colorMode == 1) { 
                int   idx   = NearestCellIndexForParticle(_sim, _sim.m_particlePos[pid]);
                float ratio = _sim.m_particleDensity[idx] / rest;
                float t     = std::clamp(0.5f + (ratio - 1.0f) * 0.75f, 0.0f, 1.0f);
                c           = ColorMapBlueGreenRed(t);
            } else { 
                int   idx = NearestCellIndexForParticle(_sim, _sim.m_particlePos[pid]);
                float t   = 0.5f + 0.5f * (_sim.m_p[idx] / maxAbsP);
                c         = ColorMapBlueGreenRed(std::clamp(t, 0.0f, 1.0f));
            }

            _sim.m_particleColor[pid] = c;
        }
    }

    Common::CaseRenderResult Bonus1::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        _lastRenderSize = desiredSize;

        float dt = std::clamp(_dt, 1e-5f, 1.0f / 10.0f);

        if (_simulate || _stepOnce) {
            _stepOnce = false;
            StepFluid(dt);
        }
        UpdateParticleColorsBonus();

        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);

        glm::mat4 projection = _camera.GetProjectionMatrix(float(desiredSize.first) / float(desiredSize.second));
        glm::mat4 view       = _camera.GetViewMatrix();

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_PROGRAM_POINT_SIZE);
        std::vector<glm::vec3> tankVerts(8);
        tankVerts[0] = { -0.5f, 0.5f, 0.5f };
        tankVerts[1] = { 0.5f, 0.5f, 0.5f };
        tankVerts[2] = { 0.5f, 0.5f, -0.5f };
        tankVerts[3] = { -0.5f, 0.5f, -0.5f };
        tankVerts[4] = { -0.5f, -0.5f, 0.5f };
        tankVerts[5] = { 0.5f, -0.5f, 0.5f };
        tankVerts[6] = { 0.5f, -0.5f, -0.5f };
        tankVerts[7] = { -0.5f, -0.5f, -0.5f };

        _flatProgram.GetUniforms().SetByName("u_Projection", projection);
        _flatProgram.GetUniforms().SetByName("u_View", view);
        _flatProgram.GetUniforms().SetByName("u_Color", _tankColor);

        _tankLineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(tankVerts));
        _tankLineItem.Draw({ _flatProgram.Use() });
        if (_enableObstacle) {
            auto sphereVerts = BuildWireSphere(_obstaclePos, _obstacleRadius, 48);

            _flatProgram.GetUniforms().SetByName("u_Color", _obstacleColor);
            _sphereLineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(sphereVerts));
            _sphereLineItem.Draw({ _flatProgram.Use() });
        }
        _particleProgram.GetUniforms().SetByName("u_Projection", projection);
        _particleProgram.GetUniforms().SetByName("u_View", view);
        _particleProgram.GetUniforms().SetByName("u_PointSize", _pointSize);

        _particleItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_sim.m_particlePos));
        _particleItem.UpdateVertexBuffer("color", Engine::make_span_bytes<glm::vec3>(_sim.m_particleColor));
        _particleItem.Draw({ _particleProgram.Use() });

        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

} // namespace VCX::Labs::Fluid
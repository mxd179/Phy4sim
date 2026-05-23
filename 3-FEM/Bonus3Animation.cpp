#include "Labs/3-FEM/Bonus3Animation.h"
#include "Labs/Common/ImGuiHelper.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace VCX::Labs::FEM {

    Bonus3Animation::Bonus3Animation():
        _program(Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _softPointItem(
            Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Points),
        _softLineItem(
            Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines),
        _sphereLineItem(
            Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        ResetScene();
    }

    int Bonus3Animation::GetID(int i, int j, int k) const {
        return i * (_wy + 1) * (_wz + 1) + j * (_wz + 1) + k;
    }

    float Bonus3Animation::Ramp(float t, float t0, float t1) const {
        if (t <= t0) return 0.0f;
        if (t >= t1) return 1.0f;
        float x = (t - t0) / (t1 - t0);
        return x * x * (3.0f - 2.0f * x);
    }

    void Bonus3Animation::ResetScene() {
        _time = 0.0f;

        _wx = std::clamp(_wx, 2, 64);
        _wy = std::clamp(_wy, 1, 32);
        _wz = std::clamp(_wz, 1, 32);
        _dx = std::clamp(_dx, 0.01f, 0.2f);

        glm::vec3 size   = glm::vec3(_wx, _wy, _wz) * _dx;
        glm::vec3 origin = -0.5f * size;

        _sim.SetModel((ElasticModel) std::clamp(_model, 0, 3));
        _sim.SetMaterial(_young, _nu, _rho);
        _sim.SetGravity(_gravity);
        _sim.SetDamping(_damping);

        _sim.SetupBox((std::size_t) _wx, (std::size_t) _wy, (std::size_t) _wz, _dx, origin);

        _forceVid = GetID(_wx, _wy / 2, _wz / 2);

        if (_forceVid < 0 || _forceVid >= (int) _sim.m_x.size() || _sim.m_fixed[(std::size_t) _forceVid]) {
            _forceVid = _sim.GetGrabVertex(); 
        }

        _sim.SetGrabVertex(_forceVid);
        _sim.SetGrabForce(glm::vec3(0.0f));

        _sim.SetCollisionParams(CollisionParams { _colK, _colDamp, _colMaxDepth });
        _sim.SetSphere(_sphereOn, _sphereC, _sphereR);
        _sim.EnableGround(_groundOn, _groundY);

        std::vector<std::uint32_t> pidx(_sim.m_x.size());
        std::iota(pidx.begin(), pidx.end(), 0u);
        _softPointItem.UpdateElementBuffer(pidx);

        RebuildSoftLineIndex();
        RebuildSphereViz();
    }

    void Bonus3Animation::RebuildSoftLineIndex() {
        std::vector<std::uint32_t> lidx;
        lidx.reserve(_sim.m_tets.size() * 6 * 2);

        auto addEdge = [&](int a, int b) {
            lidx.push_back((std::uint32_t) a);
            lidx.push_back((std::uint32_t) b);
        };

        for (auto const & t : _sim.m_tets) {
            int a = t.ids[0], b = t.ids[1], c = t.ids[2], d = t.ids[3];
            addEdge(a, b);
            addEdge(a, c);
            addEdge(a, d);
            addEdge(b, c);
            addEdge(b, d);
            addEdge(c, d);
        }
        _softLineItem.UpdateElementBuffer(lidx);
    }

    void Bonus3Animation::RebuildSphereViz() {
        _sphereVerts.clear();
        _sphereIdx.clear();

        auto addLoop = [&](std::vector<glm::vec3> const & pts) {
            std::uint32_t base = (std::uint32_t) _sphereVerts.size();
            _sphereVerts.insert(_sphereVerts.end(), pts.begin(), pts.end());
            for (std::uint32_t i = 0; i + 1 < (std::uint32_t) pts.size(); ++i) {
                _sphereIdx.push_back(base + i);
                _sphereIdx.push_back(base + i + 1);
            }
            if (pts.size() >= 2) {
                _sphereIdx.push_back(base + (std::uint32_t) pts.size() - 1);
                _sphereIdx.push_back(base);
            }
        };

        if (! _sphereOn) {
            _sphereLineItem.UpdateElementBuffer(_sphereIdx);
            return;
        }

        int  N      = 64;
        auto circle = [&](glm::vec3 a, glm::vec3 b) {
            std::vector<glm::vec3> pts;
            pts.reserve(N);
            for (int i = 0; i < N; ++i) {
                float t = float(i) / float(N) * 6.28318530718f;
                pts.push_back(_sphereC + _sphereR * (std::cos(t) * a + std::sin(t) * b));
            }
            addLoop(pts);
        };

        circle({ 1, 0, 0 }, { 0, 1, 0 });
        circle({ 1, 0, 0 }, { 0, 0, 1 });
        circle({ 0, 1, 0 }, { 0, 0, 1 });

        _sphereLineItem.UpdateElementBuffer(_sphereIdx);
    }

    void Bonus3Animation::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Control", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Simulate", &_simulate);
            ImGui::SameLine();
            if (ImGui::Button("Step")) _stepOnce = true;
            ImGui::SliderFloat("dt", &_dt, 1e-4f, 5e-3f, "%.6f");
            if (ImGui::Button("Reset")) ResetScene();

            ImGui::Text("time = %.3f", _time);
            ImGui::Text("forceVid=%d fixed=%d", _forceVid, (_forceVid >= 0 && _forceVid < (int) _sim.m_fixed.size()) ? int(_sim.m_fixed[(std::size_t) _forceVid]) : -1);
        }

        if (ImGui::CollapsingHeader("Force", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Enable Force", &_forceOn);
            ImGui::SliderFloat("Magnitude (N)", &_forceMagnitude, 0.0f, 500.0f, "%.2f");
            ImGui::SliderFloat3("Dir", glm::value_ptr(_forceDir), -1.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("Ramp (s)", &_tRamp, 0.0f, 2.0f, "%.2f");
        }

        if (ImGui::CollapsingHeader("Collision", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Sphere On", &_sphereOn);
            ImGui::SliderFloat3("sphereC", glm::value_ptr(_sphereC), -2.0f, 2.0f, "%.3f");
            ImGui::SliderFloat("sphereR", &_sphereR, 0.01f, 1.0f, "%.3f");

            ImGui::Checkbox("Ground On", &_groundOn);
            ImGui::SliderFloat("groundY", &_groundY, -2.0f, 2.0f, "%.3f");

            ImGui::SliderFloat("k", &_colK, 0.0f, 50000.0f, "%.1f");
            ImGui::SliderFloat("damp", &_colDamp, 0.0f, 500.0f, "%.1f");
            ImGui::SliderFloat("maxDepth", &_colMaxDepth, 0.001f, 0.1f, "%.4f");

            if (ImGui::Button("Apply Collision Params")) {
                _sim.SetCollisionParams(CollisionParams { _colK, _colDamp, _colMaxDepth });
                _sim.SetSphere(_sphereOn, _sphereC, _sphereR);
                _sim.EnableGround(_groundOn, _groundY);
                RebuildSphereViz();
            }
        }

        if (ImGui::CollapsingHeader("Material", ImGuiTreeNodeFlags_DefaultOpen)) {
            const char * models[] = { "Linear", "StVK", "NeoHookean", "Corotated" };
            ImGui::Combo("Model", &_model, models, 4);
            ImGui::SliderFloat("Young", &_young, 100.0f, 200000.0f, "%.1f");
            ImGui::SliderFloat("Poisson", &_nu, 0.0f, 0.49f, "%.3f");
            ImGui::SliderFloat("Density", &_rho, 10.0f, 2000.0f, "%.1f");
            ImGui::SliderFloat("Damping", &_damping, 0.0f, 0.2f, "%.3f");
            ImGui::SliderFloat3("Gravity", glm::value_ptr(_gravity), -10.0f, 10.0f, "%.3f");

            if (ImGui::Button("Apply Material Params")) {
                _sim.SetModel((ElasticModel) std::clamp(_model, 0, 3));
                _sim.SetMaterial(_young, _nu, _rho);
                _sim.SetGravity(_gravity);
                _sim.SetDamping(_damping);
            }
        }
    }

    void Bonus3Animation::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    Common::CaseRenderResult Bonus3Animation::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        float dt = std::clamp(_dt, 1e-5f, 1e-2f);

        bool doStep = _simulate || _stepOnce;
        if (doStep) {
            _stepOnce = false;
            _time += dt;
        }

        glm::vec3 F(0.0f);
        if (_forceOn) {
            glm::vec3 dir = _forceDir;
            float     len = glm::length(dir);
            if (len > 1e-6f) dir /= len;
            float w = Ramp(_time, 0.0f, std::max(1e-4f, _tRamp)); 
            F       = (w * _forceMagnitude) * dir;
        }
        _sim.SetGrabVertex(_forceVid);
        _sim.SetGrabForce(F);

        _sim.SetCollisionParams(CollisionParams { _colK, _colDamp, _colMaxDepth });
        _sim.SetSphere(_sphereOn, _sphereC, _sphereR);
        _sim.EnableGround(_groundOn, _groundY);

        if (doStep) _sim.SimulateTimestep(dt);

        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);

        if (_sphereOn) {
            glLineWidth(1.2f);
            _program.GetUniforms().SetByName("u_Color", _sphereColor);
            _sphereLineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_sphereVerts));
            _sphereLineItem.Draw({ _program.Use() });
        }

        _program.GetUniforms().SetByName("u_Color", _softColor);

        glLineWidth(0.8f);
        _softLineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_sim.m_x));
        _softLineItem.Draw({ _program.Use() });

        glPointSize(3.5f);
        _softPointItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_sim.m_x));
        _softPointItem.Draw({ _program.Use() });

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

} // namespace VCX::Labs::FEM
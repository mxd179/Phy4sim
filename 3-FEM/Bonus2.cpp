#include "Labs/3-FEM/Bonus2.h"
#include "Labs/Common/ImGuiHelper.h"

#include <algorithm>
#include <numeric>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace VCX::Labs::FEM {

    Bonus2::Bonus2():
        _program(Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _pointItem(
            Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Points),
        _lineItem(
            Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        ResetScene();
    }

    glm::vec3 Bonus2::ClampVec(glm::vec3 v, float maxLen) {
        float len = glm::length(v);
        if (len > maxLen && len > 1e-6f) v *= (maxLen / len);
        return v;
    }

    void Bonus2::ResetScene() {
        _w  = std::clamp(_w, 2, 120);
        _h  = std::clamp(_h, 2, 120);
        _dx = std::clamp(_dx, 0.01f, 0.2f);

        glm::vec3 size(float(_w) * _dx, 0.0f, float(_h) * _dx);
        glm::vec3 origin = -0.5f * size + glm::vec3(0.0f, 1.0f, 0.0f);
        _sim.SetupCloth((std::size_t) _w, (std::size_t) _h, _dx, origin);

        _sim.SetMaterial(_young, _nu, _rhoA);
        _sim.SetGravity(_gravity);
        _sim.SetDamping(_damping);
        _sim.SetPinnedCorners(_pinCorners);

        std::vector<std::uint32_t> pidx(_sim.m_x.size());
        std::iota(pidx.begin(), pidx.end(), 0u);
        _pointItem.UpdateElementBuffer(pidx);

        RebuildLineIndex();

        _sim.SetGrabForce(glm::vec3(0.0f));
    }

    void Bonus2::RebuildLineIndex() {
        std::vector<std::uint32_t> lidx;
        lidx.reserve(_sim.m_tris.size() * 3 * 2);

        auto addEdge = [&](int a, int b) {
            lidx.push_back((std::uint32_t) a);
            lidx.push_back((std::uint32_t) b);
        };

        for (auto const & t : _sim.m_tris) {
            int a = t.ids[0], b = t.ids[1], c = t.ids[2];
            addEdge(a, b);
            addEdge(b, c);
            addEdge(c, a);
        }
        _lineItem.UpdateElementBuffer(lidx);
    }

    void Bonus2::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Simulate", &_simulate);
            ImGui::SameLine();
            if (ImGui::Button("Step")) _stepOnce = true;

            ImGui::SliderFloat("dt", &_dt, 1e-4f, 5e-3f, "%.6f");
            ImGui::SliderFloat("Damping", &_damping, 0.0f, 0.2f, "%.3f");

            ImGui::Separator();
            ImGui::SliderFloat("Young", &_young, 1.0f, 500.0f, "%.2f");
            ImGui::SliderFloat("Poisson", &_nu, 0.0f, 0.49f, "%.3f");
            ImGui::SliderFloat("Area Density", &_rhoA, 0.01f, 5.0f, "%.3f");
            ImGui::SliderFloat3("Gravity", glm::value_ptr(_gravity), -20.0f, 20.0f, "%.2f");

            ImGui::Separator();
            ImGui::SliderInt("w", &_w, 2, 120);
            ImGui::SliderInt("h", &_h, 2, 120);
            ImGui::SliderFloat("dx", &_dx, 0.01f, 0.2f, "%.3f");

            ImGui::Checkbox("Pin Corners", &_pinCorners);

            if (ImGui::Button("Reset")) ResetScene();
            ImGui::SameLine();
            if (ImGui::Button("Reset To Rest")) _sim.ResetToRest();

            int vid = _sim.GetGrabVertex();
            if (ImGui::InputInt("Grab Vertex ID", &vid)) {
                vid = std::clamp(vid, 0, (int) _sim.m_x.size() - 1);
                _sim.SetGrabVertex(vid);
            }
        }

        if (ImGui::CollapsingHeader("Keyboard Force (WASDQE)", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Enable Keyboard Force", &_enableKeyboardForce);

            ImGui::SliderFloat("Key Accel (m/s^2)", &_keyAccel, 0.0f, 500.0f, "%.1f");
            ImGui::SliderFloat("Shift Boost", &_keyAccelBoost, 1.0f, 10.0f, "%.1f");
            ImGui::SliderFloat("Max Accel Clamp", &_maxKeyAccel, 10.0f, 3000.0f, "%.1f");

            ImGui::Text("W/S forward/back, A/D left/right, Q/E down/up (camera frame).");
        }

        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Color", glm::value_ptr(_color));
        }
    }

    void Bonus2::OnProcessInput(ImVec2 const & pos) {
        // mouse camera input still works
        _cameraManager.ProcessInput(_camera, pos);
    }

    bool Bonus2::ApplyKeyboardForce() {
        _suppressCameraKeysThisFrame = false;

        if (! _enableKeyboardForce) {
            _sim.SetGrabForce(glm::vec3(0.0f));
            return false;
        }

        auto & io = ImGui::GetIO();
        if (io.WantCaptureKeyboard) {
            _sim.SetGrabForce(glm::vec3(0.0f));
            return false;
        }

        if (_requireCtrlForForce && ! io.KeyCtrl) {
            _sim.SetGrabForce(glm::vec3(0.0f));
            return false;
        }

        int vid = _sim.GetGrabVertex();
        if (vid < 0 || vid >= (int) _sim.m_x.size()) {
            _sim.SetGrabForce(glm::vec3(0.0f));
            return false;
        }
        if (! _sim.m_fixed.empty() && _sim.m_fixed[(std::size_t) vid]) {
            _sim.SetGrabForce(glm::vec3(0.0f));
            return false;
        }

        // camera basis in world
        glm::mat4 invV  = glm::inverse(_camera.GetViewMatrix());
        glm::vec3 right = glm::normalize(glm::vec3(invV[0]));
        glm::vec3 up    = glm::normalize(glm::vec3(invV[1]));
        glm::vec3 fwd   = -glm::normalize(glm::vec3(invV[2])); 

        glm::vec3 dir(0.0f);
        bool      keyUsed = false;

        if (ImGui::IsKeyDown(ImGuiKey_W)) {
            dir += fwd;
            keyUsed = true;
        }
        if (ImGui::IsKeyDown(ImGuiKey_S)) {
            dir -= fwd;
            keyUsed = true;
        }
        if (ImGui::IsKeyDown(ImGuiKey_D)) {
            dir += right;
            keyUsed = true;
        }
        if (ImGui::IsKeyDown(ImGuiKey_A)) {
            dir -= right;
            keyUsed = true;
        }
        if (ImGui::IsKeyDown(ImGuiKey_E)) {
            dir += up;
            keyUsed = true;
        }
        if (ImGui::IsKeyDown(ImGuiKey_Q)) {
            dir -= up;
            keyUsed = true;
        }

        if (! keyUsed || glm::length(dir) < 1e-6f) {
            _sim.SetGrabForce(glm::vec3(0.0f));
            return false;
        }

        dir = glm::normalize(dir);

        float accel = _keyAccel;
        if (io.KeyShift) accel *= _keyAccelBoost;
        accel = std::min(accel, _maxKeyAccel);

        float     m = _sim.m_mass[(std::size_t) vid];
        glm::vec3 F = m * accel * dir;

        _sim.SetGrabForce(F);
        _suppressCameraKeysThisFrame = true;
        return true;
    }

    void Bonus2::SuppressCameraKeysBegin() {
        if (! _suppressCameraKeysThisFrame) return;

        auto & io = ImGui::GetIO();
        ImGuiKey keys[6] = { ImGuiKey_W, ImGuiKey_A, ImGuiKey_S, ImGuiKey_D, ImGuiKey_Q, ImGuiKey_E };
        for (int i = 0; i < 6; ++i) {
            _savedKeyDown[i]          = io.KeysData[keys[i]].Down;
            io.KeysData[keys[i]].Down = false;
        }
    }

    void Bonus2::SuppressCameraKeysEnd() {
        if (! _suppressCameraKeysThisFrame) return;

        auto &   io      = ImGui::GetIO();
        ImGuiKey keys[6] = { ImGuiKey_W, ImGuiKey_A, ImGuiKey_S, ImGuiKey_D, ImGuiKey_Q, ImGuiKey_E };
        for (int i = 0; i < 6; ++i) {
            io.KeysData[keys[i]].Down = _savedKeyDown[i];
        }
    }

    Common::CaseRenderResult Bonus2::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        ApplyKeyboardForce();

        _sim.SetMaterial(_young, _nu, _rhoA);
        _sim.SetGravity(_gravity);
        _sim.SetDamping(_damping);
        _sim.SetPinnedCorners(_pinCorners);

        float dt = std::clamp(_dt, 1e-5f, 1e-2f);
        if (_simulate || _stepOnce) {
            _stepOnce = false;
            _sim.SimulateTimestep(dt);
        }

        _frame.Resize(desiredSize);
        SuppressCameraKeysBegin();
        _cameraManager.Update(_camera);
        SuppressCameraKeysEnd();

        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);

        _program.GetUniforms().SetByName("u_Color", _color);

        glLineWidth(0.8f);
        _lineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_sim.m_x));
        _lineItem.Draw({ _program.Use() });

        glPointSize(3.0f);
        _pointItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_sim.m_x));
        _pointItem.Draw({ _program.Use() });

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
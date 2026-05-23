#include "Labs/3-FEM/Bonus1.h"
#include "Labs/Common/ImGuiHelper.h"

#include <algorithm>
#include <numeric>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace VCX::Labs::FEM {

    Bonus1::Bonus1():
        _program(Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _pointItem(
            Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Points),
        _lineItem(
            Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        _sim.SetModel(ElasticModel::StVK); 
        ResetScene();
    }

    void Bonus1::SetModelAndReset(ElasticModel m) {
        _sim.SetModel(m);
        _sim.ResetToRest();
        _dragging = false;
        _sim.SetGrabForce(glm::vec3(0.0f));
    }

    void Bonus1::ResetScene() {
        _wx = std::clamp(_wx, 2, 64);
        _wy = std::clamp(_wy, 1, 32);
        _wz = std::clamp(_wz, 1, 32);
        _dx = std::clamp(_dx, 0.01f, 0.2f);

        _sim.SetMaterial(_young, _nu, _rho);
        _sim.SetGravity(_gravity);
        _sim.SetDamping(_damping);

        glm::vec3 size   = glm::vec3(_wx, _wy, _wz) * _dx;
        glm::vec3 origin = -0.5f * size;

        _sim.SetupBox((std::size_t) _wx, (std::size_t) _wy, (std::size_t) _wz, _dx, origin);

        std::vector<std::uint32_t> pidx(_sim.m_x.size());
        std::iota(pidx.begin(), pidx.end(), 0u);
        _pointItem.UpdateElementBuffer(pidx);

        RebuildLineIndex();

        _dragging   = false;
        _dragTarget = glm::vec3(0.0f);
        _sim.SetGrabForce(glm::vec3(0.0f));
    }

    void Bonus1::RebuildLineIndex() {
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
        _lineItem.UpdateElementBuffer(lidx);
    }

    void Bonus1::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Model Switch", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Click to switch model (auto ResetToRest):");

            if (ImGui::Button("Linear")) SetModelAndReset(ElasticModel::Linear);
            ImGui::SameLine();
            if (ImGui::Button("StVK")) SetModelAndReset(ElasticModel::StVK);
            ImGui::SameLine();
            if (ImGui::Button("Neo-Hookean")) SetModelAndReset(ElasticModel::NeoHookean);
            ImGui::SameLine();
            if (ImGui::Button("Corotated")) SetModelAndReset(ElasticModel::Corotated);

            ImGui::Text("Current: %d", int(_sim.GetModel()));
        }

        if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Simulate", &_simulate);
            ImGui::SameLine();
            if (ImGui::Button("Step")) _stepOnce = true;

            ImGui::SliderFloat("dt", &_dt, 1e-4f, 5e-3f, "%.6f");
            ImGui::SliderFloat("Damping", &_damping, 0.0f, 0.2f, "%.3f");
            ImGui::SliderFloat3("Gravity", glm::value_ptr(_gravity), -5.0f, 5.0f, "%.3f");

            ImGui::Separator();
            ImGui::SliderFloat("Young (Pa)", &_young, 100.0f, 100000.0f, "%.1f");
            ImGui::SliderFloat("Poisson", &_nu, 0.0f, 0.49f, "%.3f");
            ImGui::SliderFloat("Density", &_rho, 10.0f, 2000.0f, "%.1f");

            ImGui::Separator();
            ImGui::SliderInt("wx", &_wx, 2, 64);
            ImGui::SliderInt("wy", &_wy, 1, 32);
            ImGui::SliderInt("wz", &_wz, 1, 32);
            ImGui::SliderFloat("dx", &_dx, 0.01f, 0.2f, "%.3f");

            if (ImGui::Button("Reset Scene")) ResetScene();
            ImGui::SameLine();
            if (ImGui::Button("Reset To Rest")) _sim.ResetToRest();

            int vid = _sim.GetGrabVertex();
            if (ImGui::InputInt("Grab Vertex ID", &vid)) {
                vid = std::clamp(vid, 0, (int) _sim.m_x.size() - 1);
                _sim.SetGrabVertex(vid);
            }

            ImGui::Checkbox("Draw Lines", &_drawLines);
            ImGui::SameLine();
            ImGui::Checkbox("Draw Points", &_drawPoints);
        }

        if (ImGui::CollapsingHeader("Interaction (RMB Drag)", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Enable Drag", &_enableDrag);
            ImGui::SliderFloat("dragMoveScale", &_dragMoveScale, 0.001f, 1.0f, "%.3f");
            ImGui::SliderFloat("dragK", &_dragK, 0.0f, 20000.0f, "%.1f");
            ImGui::SliderFloat("dragDamp", &_dragDamp, 0.0f, 500.0f, "%.1f");
            ImGui::SliderFloat("maxDragForce", &_maxDragForce, 100.0f, 50000.0f, "%.1f");
        }

        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Color", glm::value_ptr(_color));
        }
    }

    void Bonus1::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void Bonus1::OnProcessMouseControl(glm::vec3 mouseDelta) {
        bool wantDrag = _enableDrag && ImGui::IsMouseDown(ImGuiMouseButton_Right);

        int vid = _sim.GetGrabVertex();
        if (vid < 0 || vid >= (int) _sim.m_x.size()) {
            _sim.SetGrabForce(glm::vec3(0.0f));
            _dragging = false;
            return;
        }

        if (wantDrag && ! _dragging) {
            _dragging   = true;
            _dragTarget = _sim.m_x[(std::size_t) vid];
        }
        if (! wantDrag) {
            _dragging = false;
            _sim.SetGrabForce(glm::vec3(0.0f));
            return;
        }

        _dragTarget += mouseDelta * _dragMoveScale;

        glm::vec3 x = _sim.m_x[(std::size_t) vid];
        glm::vec3 v = _sim.m_v[(std::size_t) vid];

        glm::vec3 F = _dragK * (_dragTarget - x) - _dragDamp * v;

        float len = glm::length(F);
        if (len > _maxDragForce && len > 1e-6f) F *= (_maxDragForce / len);

        _sim.SetGrabForce(F);
    }

    Common::CaseRenderResult Bonus1::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        OnProcessMouseControl(_cameraManager.getMouseMove());
        _sim.SetMaterial(_young, _nu, _rho);
        _sim.SetGravity(_gravity);
        _sim.SetDamping(_damping);

        float dt = std::clamp(_dt, 1e-5f, 1e-2f);
        if (_simulate || _stepOnce) {
            _stepOnce = false;
            _sim.SimulateTimestep(dt);
        }

        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);

        _program.GetUniforms().SetByName("u_Color", _color);

        if (_drawLines) {
            glLineWidth(0.8f);
            _lineItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_sim.m_x));
            _lineItem.Draw({ _program.Use() });
        }

        if (_drawPoints) {
            glPointSize(3.5f);
            _pointItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_sim.m_x));
            _pointItem.Draw({ _program.Use() });
        }

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
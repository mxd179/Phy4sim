#include "Labs/1-RigidBody/CaseBox.h"
#include "Labs/Common/ImGuiHelper.h"

#include <algorithm>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

namespace VCX::Labs::RigidBody {

    CaseBox::CaseBox():
        _program(Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        const std::vector<std::uint32_t> line_index = {
            0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7
        };
        _lineItem.UpdateElementBuffer(line_index);

        const std::vector<std::uint32_t> tri_index = {
            0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6
        };
        _boxItem.UpdateElementBuffer(tri_index);

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        ResetRigidBody();
    }

    glm::mat3 CaseBox::ComputeInertiaBody(float mass, glm::vec3 const & dim) const {
        float     dx = dim.x, dy = dim.y, dz = dim.z;
        float     Ixx = (mass / 12.0f) * (dy * dy + dz * dz);
        float     Iyy = (mass / 12.0f) * (dx * dx + dz * dz);
        float     Izz = (mass / 12.0f) * (dx * dx + dy * dy);
        glm::mat3 I(0.0f);
        I[0][0] = Ixx;
        I[1][1] = Iyy;
        I[2][2] = Izz;
        return I;
    }

    void CaseBox::ClearAccumulators() {
        _forceWorld  = glm::vec3(0.f);
        _torqueWorld = glm::vec3(0.f);
    }

    void CaseBox::AddForceWorld(glm::vec3 const & f) {
        _forceWorld += f;
    }

    void CaseBox::AddTorqueWorld(glm::vec3 const & t) {
        _torqueWorld += t;
    }

    void CaseBox::ResetRigidBody() {
        _center = glm::vec3(0.f, 0.f, 0.f);

        _v     = glm::vec3(velocity, 0.f, 0.f);
        _q     = glm::quat(1.f, 0.f, 0.f, 0.f);
        _wBody = glm::vec3(0.f, omega, 0.f);

        _Ibody    = ComputeInertiaBody(_mass, _dim);
        _IbodyInv = glm::inverse(_Ibody);

        ClearAccumulators();
    }

    void CaseBox::StepPhysicsExplicitEuler(float dt) {
        if (! _simulate) {
            ClearAccumulators();
            return;
        }
        if (dt <= 0.f) {
            ClearAccumulators();
            return;
        }
        _center += _v * dt;
        _v += (_forceWorld / _mass) * dt;
        glm::mat3 R  = glm::mat3_cast(_q); 
        glm::mat3 RT = glm::transpose(R);
        glm::vec3 tauBody = RT * _torqueWorld;
        glm::vec3 w    = _wBody;
        glm::vec3 Iw   = _Ibody * w;
        glm::vec3 wDot = _IbodyInv * (tauBody - glm::cross(w, Iw));
        _wBody += wDot * dt;
        glm::vec3 wWorld = R * _wBody;
        glm::quat wq(0.f, wWorld.x, wWorld.y, wWorld.z);
        glm::quat qdot = 0.5f * (wq * _q);

        _q = glm::normalize(_q + qdot * dt);

        ClearAccumulators();
    }

    void CaseBox::Moving() {
        StepPhysicsExplicitEuler(1.0f / 60.0f);
    }

    void CaseBox::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
        }

        if (ImGui::CollapsingHeader("Initial Condition", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("v0 (+X)", &velocity, -4.f, 4.f);
            ImGui::SliderFloat("omega0 (+Y, rad/s)", &omega, -8.f, 8.f);
            if (ImGui::Button("Reset")) ResetRigidBody();
        }

        if (ImGui::CollapsingHeader("Simulation / Control", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Simulate", &_simulate);
            ImGui::SliderFloat("Force Magnitude", &_forceMag, 0.f, 80.f);
            ImGui::SliderFloat("Torque Magnitude", &_torqueMag, 0.f, 30.f);

            ImGui::Text("Keyboard force: WASD (XZ), RF (Y)");
            ImGui::Text("Keyboard torque: Q/E (+/-Y)");\
        }

        ImGui::Spacing();
    }

    void CaseBox::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
        glm::vec3 f(0.f);
        if (ImGui::IsKeyDown(ImGuiKey_W)) f += glm::vec3(0, 0, -1);
        if (ImGui::IsKeyDown(ImGuiKey_S)) f += glm::vec3(0, 0, 1);
        if (ImGui::IsKeyDown(ImGuiKey_A)) f += glm::vec3(-1, 0, 0);
        if (ImGui::IsKeyDown(ImGuiKey_D)) f += glm::vec3(1, 0, 0);
        if (ImGui::IsKeyDown(ImGuiKey_R)) f += glm::vec3(0, 1, 0);
        if (ImGui::IsKeyDown(ImGuiKey_F)) f += glm::vec3(0, -1, 0);

        if (glm::dot(f, f) > 1e-8f) {
            AddForceWorld(glm::normalize(f) * _forceMag);
        }
        if (ImGui::IsKeyDown(ImGuiKey_Q)) AddTorqueWorld(glm::vec3(0, _torqueMag, 0));
        if (ImGui::IsKeyDown(ImGuiKey_E)) AddTorqueWorld(glm::vec3(0, -_torqueMag, 0));
    }


    Common::CaseRenderResult CaseBox::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        float dt = ImGui::GetIO().DeltaTime;
        dt       = std::clamp(dt, 0.0f, 1.0f / 30.0f);
        StepPhysicsExplicitEuler(dt);
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);
        std::vector<glm::vec3> VertsPosition;
        VertsPosition.resize(8);

        glm::mat3 R = glm::mat3_cast(_q);

        glm::vec3 new_x = R * (0.5f * _dim[0] * glm::vec3(1.f, 0.f, 0.f));
        glm::vec3 new_y = R * (0.5f * _dim[1] * glm::vec3(0.f, 1.f, 0.f));
        glm::vec3 new_z = R * (0.5f * _dim[2] * glm::vec3(0.f, 0.f, 1.f));

        VertsPosition[0] = _center - new_x + new_y + new_z;
        VertsPosition[1] = _center + new_x + new_y + new_z;
        VertsPosition[2] = _center + new_x + new_y - new_z;
        VertsPosition[3] = _center - new_x + new_y - new_z;
        VertsPosition[4] = _center - new_x - new_y + new_z;
        VertsPosition[5] = _center + new_x - new_y + new_z;
        VertsPosition[6] = _center + new_x - new_y - new_z;
        VertsPosition[7] = _center - new_x - new_y - new_z;

        auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);

        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", span_bytes);
        _boxItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes);
        _lineItem.Draw({ _program.Use() });

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

} // namespace VCX::Labs::RigidBody
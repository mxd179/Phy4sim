#include "Labs/1-RigidBody/CaseCollision.h"
#include "Labs/Common/ImGuiHelper.h"

#include <algorithm>
#include <imgui.h>

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace VCX::Labs::RigidBody {

    static inline Eigen::Vector3d ToEigen(glm::vec3 const & v) {
        return Eigen::Vector3d(v.x, v.y, v.z);
    }
    static inline glm::vec3 ToGlm(Eigen::Vector3d const & v) {
        return glm::vec3(float(v.x()), float(v.y()), float(v.z()));
    }
    static inline Eigen::Quaterniond ToEigen(glm::quat const & q) {
        return Eigen::Quaterniond(double(q.w), double(q.x), double(q.y), double(q.z));
    }

    CaseCollision::CaseCollision():
        _program(Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _geomA(std::make_shared<fcl::Boxd>(1.0, 1.0, 1.0)),
        _geomB(std::make_shared<fcl::Boxd>(1.0, 1.0, 1.0)),
        _objA(_geomA),
        _objB(_geomB) {
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

        _A.mass    = 1.f;
        _A.invMass = 1.f / _A.mass;
        _B.mass    = 1.f;
        _B.invMass = 1.f / _B.mass;

        _A.color = glm::vec3(0.2f, 0.8f, 0.6f);
        _B.color = glm::vec3(0.8f, 0.4f, 0.3f);

        ResetScenario(_scenario);
    }

    glm::mat3 CaseCollision::ComputeInertiaBody(float mass, glm::vec3 const & dim) const {
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

    glm::mat3 CaseCollision::WorldInertiaInv(RigidBox const & b) const {
        glm::mat3 R     = glm::mat3_cast(b.q);
        glm::mat3 IwInv = R * b.IbodyInv * glm::transpose(R);
        return IwInv;
    }

    void CaseCollision::ResetScenario(Scenario s) {
        _scenario = s;
        _contacts.clear();
        _A.dim = glm::vec3(1.f, 1.5f, 1.f);
        _B.dim = glm::vec3(1.f, 1.0f, 2.f);
        _A.Ibody    = ComputeInertiaBody(_A.mass, _A.dim);
        _A.IbodyInv = glm::inverse(_A.Ibody);
        _B.Ibody    = ComputeInertiaBody(_B.mass, _B.dim);
        _B.IbodyInv = glm::inverse(_B.Ibody);
        _A.ClearAcc();
        _B.ClearAcc();
        _A.q = glm::quat(1.f, 0.f, 0.f, 0.f);
        _B.q = glm::quat(1.f, 0.f, 0.f, 0.f);
        _A.w = glm::vec3(0.f);
        _B.w = glm::vec3(0.f);
        if (s == Scenario::HeadOn) {
            _A.x = glm::vec3(-2.0f, 0.f, 0.f);
            _B.x = glm::vec3(2.0f, 0.f, 0.f);
            _A.v = glm::vec3(2.0f, 0.f, 0.f);
            _B.v = glm::vec3(-2.0f, 0.f, 0.f);
        } else if (s == Scenario::CornerFace) {
            _A.x = glm::vec3(-1.5f, 0.0f, 0.0f);
            _B.x = glm::vec3(1.0f, 0.0f, 0.0f);
            _B.q = glm::normalize(glm::angleAxis(glm::radians(25.0f), glm::vec3(0, 1, 0)) * glm::angleAxis(glm::radians(15.0f), glm::vec3(0, 0, 1)));
            _A.v = glm::vec3(2.5f, 0.3f, 0.2f);
            _B.v = glm::vec3(-0.8f, 0.0f, 0.0f);
        } else if (s == Scenario::EdgeEdge) {
            _A.x = glm::vec3(-1.8f, 0.2f, 0.6f);
            _B.x = glm::vec3(1.8f, 0.0f, -0.6f);
            _A.q = glm::normalize(glm::angleAxis(glm::radians(30.0f), glm::vec3(0, 0, 1)));
            _B.q = glm::normalize(glm::angleAxis(glm::radians(-20.0f), glm::vec3(0, 1, 0)));
            _A.v = glm::vec3(2.0f, 0.0f, -0.3f);
            _B.v = glm::vec3(-2.0f, 0.0f, 0.3f);
        } else if (s == Scenario::FaceFace) {
            _A.dim      = glm::vec3(1.2f, 1.2f, 1.2f);
            _B.dim      = glm::vec3(1.2f, 1.2f, 1.2f);
            _A.Ibody    = ComputeInertiaBody(_A.mass, _A.dim);
            _A.IbodyInv = glm::inverse(_A.Ibody);
            _B.Ibody    = ComputeInertiaBody(_B.mass, _B.dim);
            _B.IbodyInv = glm::inverse(_B.Ibody);
            _A.x = glm::vec3(-2.0f, 0.0f, 0.0f);
            _B.x = glm::vec3(2.0f, 0.0f, 0.0f);
            _A.v = glm::vec3(2.0f, 0.0f, 0.0f);
            _B.v = glm::vec3(-2.0f, 0.0f, 0.0f);
            _B.q = glm::normalize(glm::angleAxis(glm::radians(10.0f), glm::vec3(0, 1, 0)));
        }
        _geomA = std::make_shared<fcl::Boxd>(_A.dim.x, _A.dim.y, _A.dim.z);
        _geomB = std::make_shared<fcl::Boxd>(_B.dim.x, _B.dim.y, _B.dim.z);
        _objA  = fcl::CollisionObjectd(_geomA);
        _objB  = fcl::CollisionObjectd(_geomB);

        UpdateFCLObjects();
    }

    void CaseCollision::IntegrateExplicitEuler(RigidBox & b, float dt) {
        b.x += b.v * dt;
        b.v += (b.force * b.invMass) * dt;
        glm::mat3 IwInv = WorldInertiaInv(b);
        b.w += (IwInv * b.torque) * dt;
        glm::quat wq(0.f, b.w.x, b.w.y, b.w.z);
        glm::quat qdot = 0.5f * (wq * b.q);
        b.q            = glm::normalize(b.q + qdot * dt);
        b.ClearAcc();
    }

    void CaseCollision::ApplyImpulseAtWorldPoint(RigidBox & b, glm::vec3 const & p, glm::vec3 const & J) {
        b.v += J * b.invMass;
        glm::vec3 r     = p - b.x;
        glm::mat3 IwInv = WorldInertiaInv(b);
        b.w += IwInv * glm::cross(r, J);
    }

    void CaseCollision::UpdateFCLObjects() {
        {
            fcl::Transform3d   tf = fcl::Transform3d::Identity();
            Eigen::Quaterniond qe = ToEigen(_A.q);
            tf.linear()           = qe.toRotationMatrix();
            tf.translation()      = ToEigen(_A.x);
            _objA.setTransform(tf);
            _objA.computeAABB();
        }
        {
            fcl::Transform3d   tf = fcl::Transform3d::Identity();
            Eigen::Quaterniond qe = ToEigen(_B.q);
            tf.linear()           = qe.toRotationMatrix();
            tf.translation()      = ToEigen(_B.x);
            _objB.setTransform(tf);
            _objB.computeAABB();
        }
    }

    void CaseCollision::SolveCollisionImpulse(float dt) {
        _contacts.clear();

        fcl::CollisionRequestd request;
        request.enable_contact   = true;
        request.num_max_contacts = 16;

        fcl::CollisionResultd result;
        fcl::collide(&_objA, &_objB, request, result);

        std::vector<fcl::Contactd> contacts;
        result.getContacts(contacts);

        if (contacts.empty()) return;
        float     maxPen = 0.f;
        glm::vec3 nCorr(0.f);
        for (auto const & c : contacts) {
            maxPen      = std::max(maxPen, float(c.penetration_depth));
            glm::vec3 n = ToGlm(c.normal);
            if (glm::dot(n, _B.x - _A.x) < 0.f) n = -n;
            nCorr = n;
        }
        if (maxPen > 0.f) {
            float const slop    = 0.001f;
            float const percent = 0.6f;
            float const corrMag = percent * std::max(maxPen - slop, 0.0f) / (_A.invMass + _B.invMass);
            glm::vec3   corr    = corrMag * nCorr;
            _A.x -= corr * _A.invMass;
            _B.x += corr * _B.invMass;
            UpdateFCLObjects();
        }

        for (int it = 0; it < _solverIters; ++it) {
            fcl::CollisionResultd r2;
            fcl::collide(&_objA, &_objB, request, r2);
            contacts.clear();
            r2.getContacts(contacts);
            if (contacts.empty()) break;
            for (auto const & c : contacts) {
                glm::vec3 p   = ToGlm(c.pos);
                glm::vec3 n   = ToGlm(c.normal);
                float     pen = float(c.penetration_depth);
                if (glm::dot(n, _B.x - _A.x) < 0.f) n = -n;
                glm::vec3 rA   = p - _A.x;
                glm::vec3 rB   = p - _B.x;
                glm::vec3 vA   = _A.v + glm::cross(_A.w, rA);
                glm::vec3 vB   = _B.v + glm::cross(_B.w, rB);
                glm::vec3 vRel = vB - vA;
                float vn = glm::dot(vRel, n);
                if (vn > 0.f) {
                    continue;
                }
                glm::mat3 IAInv = WorldInertiaInv(_A);
                glm::mat3 IBInv = WorldInertiaInv(_B);
                glm::vec3 raXn = glm::cross(rA, n);
                glm::vec3 rbXn = glm::cross(rB, n);
                float denom =
                    _A.invMass + _B.invMass + glm::dot(n, glm::cross(IAInv * raXn, rA) + glm::cross(IBInv * rbXn, rB));

                if (denom < 1e-8f) continue;

                float e = _restitution;
                float j = -(1.f + e) * vn / denom;
                glm::vec3 J = j * n;
                ApplyImpulseAtWorldPoint(_A, p, -J);
                ApplyImpulseAtWorldPoint(_B, p, J);
                UpdateFCLObjects();
                if (it == _solverIters - 1) {
                    _contacts.push_back({ p, n, pen });
                }
            }
        }
    }

    void CaseCollision::BuildBoxVertsWorld(RigidBox const & b, std::vector<glm::vec3> & V) const {
        V.resize(8);
        glm::mat3 R = glm::mat3_cast(b.q);
        glm::vec3 ex = R * (0.5f * b.dim.x * glm::vec3(1, 0, 0));
        glm::vec3 ey = R * (0.5f * b.dim.y * glm::vec3(0, 1, 0));
        glm::vec3 ez = R * (0.5f * b.dim.z * glm::vec3(0, 0, 1));
        V[0] = b.x - ex + ey + ez;
        V[1] = b.x + ex + ey + ez;
        V[2] = b.x + ex + ey - ez;
        V[3] = b.x - ex + ey - ez;
        V[4] = b.x - ex - ey + ez;
        V[5] = b.x + ex - ey + ez;
        V[6] = b.x + ex - ey - ez;
        V[7] = b.x - ex - ey - ez;
    }

    void CaseCollision::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Scenario", ImGuiTreeNodeFlags_DefaultOpen)) {
            int s = int(_scenario);
            ImGui::RadioButton("HeadOn", &s, int(Scenario::HeadOn));
            ImGui::RadioButton("Corner-Face", &s, int(Scenario::CornerFace));
            ImGui::RadioButton("Edge-Edge", &s, int(Scenario::EdgeEdge));
            ImGui::RadioButton("Face-Face", &s, int(Scenario::FaceFace));
            if (s != int(_scenario)) ResetScenario(Scenario(s));
            if (ImGui::Button("Reset Scenario")) ResetScenario(_scenario);
        }

        if (ImGui::CollapsingHeader("Solver", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Simulate", &_simulate);
            ImGui::SliderFloat("Restitution", &_restitution, 0.f, 1.f);
            ImGui::SliderInt("Solver Iterations", &_solverIters, 1, 30);
            ImGui::SliderFloat("Fixed dt", &_fixedDt, 1.f / 1000.f, 1.f / 60.f, "%.6f");
            ImGui::Text("Contacts (last): %d", int(_contacts.size()));
        }

        if (ImGui::CollapsingHeader("Interaction", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderInt("Active Body (0:A 1:B)", &_activeBody, 0, 1);
            ImGui::SliderFloat("Force Mag", &_forceMag, 0.f, 100.f);
            ImGui::SliderFloat("Torque Mag", &_torqueMag, 0.f, 50.f);
            ImGui::Text("Force: WASD (XZ) RF (Y)");
            ImGui::Text("Torque: Q/E (+/-Y)");
            ImGui::Text("Mouse: Shift+RightDrag -> Force; +Ctrl -> Torque");
        }
    }

    void CaseCollision::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);

        RigidBox & b = (_activeBody == 0) ? _A : _B;

        glm::vec3 f(0.f);
        if (ImGui::IsKeyDown(ImGuiKey_W)) f += glm::vec3(0, 0, -1);
        if (ImGui::IsKeyDown(ImGuiKey_S)) f += glm::vec3(0, 0, 1);
        if (ImGui::IsKeyDown(ImGuiKey_A)) f += glm::vec3(-1, 0, 0);
        if (ImGui::IsKeyDown(ImGuiKey_D)) f += glm::vec3(1, 0, 0);
        if (ImGui::IsKeyDown(ImGuiKey_R)) f += glm::vec3(0, 1, 0);
        if (ImGui::IsKeyDown(ImGuiKey_F)) f += glm::vec3(0, -1, 0);

        if (glm::dot(f, f) > 1e-8f) {
            b.force += glm::normalize(f) * _forceMag;
        }

        if (ImGui::IsKeyDown(ImGuiKey_Q)) b.torque += glm::vec3(0, _torqueMag, 0);
        if (ImGui::IsKeyDown(ImGuiKey_E)) b.torque += glm::vec3(0, -_torqueMag, 0);
    }

    Common::CaseRenderResult CaseCollision::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        float frameDt = ImGui::GetIO().DeltaTime;
        frameDt       = std::clamp(frameDt, 0.0f, 1.0f / 20.0f);

        if (_simulate) {
            int   steps = std::max(1, int(std::ceil(frameDt / _fixedDt)));
            float dt    = frameDt / float(steps);

            for (int i = 0; i < steps; ++i) {
                IntegrateExplicitEuler(_A, dt);
                IntegrateExplicitEuler(_B, dt);
                UpdateFCLObjects();
                SolveCollisionImpulse(dt);
            }
        } else {
            UpdateFCLObjects();
            SolveCollisionImpulse(0.f);
        }

        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        std::vector<glm::vec3> verts;

        BuildBoxVertsWorld(_A, verts);
        auto spanA = Engine::make_span_bytes<glm::vec3>(verts);
        _program.GetUniforms().SetByName("u_Color", _A.color);
        _boxItem.UpdateVertexBuffer("position", spanA);
        _boxItem.Draw({ _program.Use() });
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f));
        _lineItem.UpdateVertexBuffer("position", spanA);
        _lineItem.Draw({ _program.Use() });
        BuildBoxVertsWorld(_B, verts);
        auto spanB = Engine::make_span_bytes<glm::vec3>(verts);
        _program.GetUniforms().SetByName("u_Color", _B.color);
        _boxItem.UpdateVertexBuffer("position", spanB);
        _boxItem.Draw({ _program.Use() });
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f));
        _lineItem.UpdateVertexBuffer("position", spanB);
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
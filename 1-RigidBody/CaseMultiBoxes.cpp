#include "Labs/1-RigidBody/CaseMultiBoxes.h"
#include "Labs/Common/ImGuiHelper.h"

#include <algorithm>
#include <cmath>
#include <imgui.h>

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

    CaseMultiBoxes::CaseMultiBoxes():
        _program(Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
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

        ResetScene();
    }

    glm::mat3 CaseMultiBoxes::ComputeInertiaBody(float mass, glm::vec3 const & dim) const {
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

    glm::mat3 CaseMultiBoxes::WorldInertiaInv(Body const & b) const {
        if (b.isStatic) return glm::mat3(0.0f);
        glm::mat3 R = glm::mat3_cast(b.q);
        return R * b.IbodyInv * glm::transpose(R);
    }

    void CaseMultiBoxes::IntegrateExplicitEuler(Body & b, float dt) {
        if (b.isStatic) return;

        if (_useGravity) b.force += glm::vec3(0.f, -9.8f * b.mass, 0.f);

        b.x += b.v * dt;
        b.v += (b.force * b.invMass) * dt;

        glm::mat3 IwInv = WorldInertiaInv(b);
        b.w += (IwInv * b.torque) * dt;

        glm::quat wq(0.f, b.w.x, b.w.y, b.w.z);
        glm::quat qdot = 0.5f * (wq * b.q);
        b.q            = glm::normalize(b.q + qdot * dt);

        b.ClearAcc();
    }

    void CaseMultiBoxes::ApplyImpulseAtWorldPoint(Body & b, glm::vec3 const & p, glm::vec3 const & J) {
        if (b.isStatic) return;
        b.v += J * b.invMass;

        glm::vec3 r     = p - b.x;
        glm::mat3 IwInv = WorldInertiaInv(b);
        b.w += IwInv * glm::cross(r, J);
    }

    void CaseMultiBoxes::ResetScene() {
        _bodies.clear();
        _geoms.clear();
        _objs.clear();

        auto addStaticBox = [&](glm::vec3 const & center, glm::vec3 const & dim, glm::vec3 const & color) {
            Body b;
            b.isStatic = true;
            b.mass     = 0.f;
            b.invMass  = 0.f;
            b.dim      = dim;
            b.x        = center;
            b.q        = glm::quat(1, 0, 0, 0);
            b.v        = glm::vec3(0.f);
            b.w        = glm::vec3(0.f);
            b.Ibody    = glm::mat3(0.f);
            b.IbodyInv = glm::mat3(0.f);
            b.color    = color;
            _bodies.push_back(b);
        };

        addStaticBox(glm::vec3(0.f, -3.0f, 0.f), glm::vec3(20.f, 1.0f, 20.f), glm::vec3(0.25f, 0.25f, 0.25f));
        addStaticBox(glm::vec3(0.f, 0.0f, -10.0f), glm::vec3(20.f, 8.f, 1.0f), glm::vec3(0.20f, 0.20f, 0.28f));
        addStaticBox(glm::vec3(-10.0f, 0.0f, 0.f), glm::vec3(1.0f, 8.f, 20.f), glm::vec3(0.20f, 0.28f, 0.20f));
        addStaticBox(glm::vec3(10.0f, 0.0f, 0.f), glm::vec3(1.0f, 8.f, 20.f), glm::vec3(0.28f, 0.20f, 0.20f));

        int staticCount = (int) _bodies.size();

        _dynamicCount = std::max(4, _dynamicCount);

        for (int i = 0; i < _dynamicCount; ++i) {
            Body b;
            b.isStatic = false;
            b.mass     = 1.f;
            b.invMass  = 1.f / b.mass;

            float sx = 0.8f + 0.2f * float((i % 3));
            float sy = 0.8f + 0.1f * float((i % 2));
            float sz = 0.8f + 0.2f * float(((i + 1) % 3));
            b.dim    = glm::vec3(sx, sy, sz);

            b.Ibody    = ComputeInertiaBody(b.mass, b.dim);
            b.IbodyInv = glm::inverse(b.Ibody);

            int row = i % 3;
            int col = i / 3;
            b.x     = glm::vec3(-1.0f + row * 1.2f, -1.0f + col * 1.2f, 0.5f * float((i % 2) ? 1 : -1));
            b.v     = glm::vec3(0.0f);
            b.w     = glm::vec3(0.0f);

            float ang = 0.15f * float(i);
            b.q       = glm::normalize(glm::angleAxis(ang, glm::vec3(0, 1, 0)) * glm::angleAxis(0.1f * float(i), glm::vec3(1, 0, 0)));

            b.color = glm::vec3(0.2f + 0.1f * (i % 6), 0.7f - 0.08f * (i % 5), 0.4f + 0.06f * (i % 4));
            b.ClearAcc();

            _bodies.push_back(b);
        }

        _activeBody = staticCount;

        _geoms.reserve(_bodies.size());
        _objs.reserve(_bodies.size());
        for (auto const & b : _bodies) {
            auto geom = std::make_shared<fcl::Boxd>(b.dim.x, b.dim.y, b.dim.z);
            _geoms.push_back(geom);
            _objs.emplace_back(geom);
        }

        UpdateFCLObjects();
    }

    void CaseMultiBoxes::UpdateFCLObjects() {
        for (std::size_t i = 0; i < _bodies.size(); ++i) {
            auto const &       b  = _bodies[i];
            fcl::Transform3d   tf = fcl::Transform3d::Identity();
            Eigen::Quaterniond qe = ToEigen(b.q);
            tf.linear()           = qe.toRotationMatrix();
            tf.translation()      = ToEigen(b.x);
            _objs[i].setTransform(tf);
            _objs[i].computeAABB();
        }
    }

    void CaseMultiBoxes::DetectContacts(std::vector<Contact> & contacts) {
        contacts.clear();

        fcl::CollisionRequestd req;
        req.enable_contact   = true;
        req.num_max_contacts = 12; 
        for (int i = 0; i < (int) _bodies.size(); ++i) {
            for (int j = i + 1; j < (int) _bodies.size(); ++j) {
                if (_bodies[i].isStatic && _bodies[j].isStatic) continue;

                fcl::CollisionResultd res;
                fcl::collide(&_objs[i], &_objs[j], req, res);

                std::vector<fcl::Contactd> cs;
                res.getContacts(cs);
                for (auto const & c : cs) {
                    Contact ct;
                    ct.a     = i;
                    ct.b     = j;
                    ct.p     = ToGlm(c.pos);
                    ct.n     = ToGlm(c.normal);
                    ct.depth = float(c.penetration_depth);
                    if (glm::dot(ct.n, _bodies[j].x - _bodies[i].x) < 0.f) ct.n = -ct.n;
                    contacts.push_back(ct);
                }
            }
        }
    }

    void CaseMultiBoxes::SolveContactsSequentialImpulse(std::vector<Contact> const & contacts) {
        for (int it = 0; it < _solverIters; ++it) {
            for (auto const & c : contacts) {
                Body & A = _bodies[c.a];
                Body & B = _bodies[c.b];

                glm::vec3 n = c.n;

                glm::vec3 rA = c.p - A.x;
                glm::vec3 rB = c.p - B.x;

                glm::vec3 vA   = A.v + glm::cross(A.w, rA);
                glm::vec3 vB   = B.v + glm::cross(B.w, rB);
                glm::vec3 vRel = vB - vA;

                float vn = glm::dot(vRel, n);
                if (vn > 0.f) continue;

                float e = _restitution;

                glm::mat3 IAInv = WorldInertiaInv(A);
                glm::mat3 IBInv = WorldInertiaInv(B);

                glm::vec3 raXn = glm::cross(rA, n);
                glm::vec3 rbXn = glm::cross(rB, n);

                float denom =
                    A.invMass + B.invMass + glm::dot(n, glm::cross(IAInv * raXn, rA) + glm::cross(IBInv * rbXn, rB));

                if (denom < 1e-8f) continue;

                float     j = -(1.f + e) * vn / denom;
                glm::vec3 J = j * n;

                ApplyImpulseAtWorldPoint(A, c.p, -J);
                ApplyImpulseAtWorldPoint(B, c.p, J);
            }
        }
    }

    void CaseMultiBoxes::PositionCorrection(std::vector<Contact> const & contacts) {
        float const slop    = 0.002f;
        float const percent = 0.6f;

        for (auto const & c : contacts) {
            Body & A = _bodies[c.a];
            Body & B = _bodies[c.b];

            float invMassSum = A.invMass + B.invMass;
            if (invMassSum <= 0.f) continue;

            float depth = c.depth;
            if (depth <= slop) continue;

            glm::vec3 corr = percent * (depth - slop) / invMassSum * c.n;

            if (! A.isStatic) A.x -= corr * A.invMass;
            if (! B.isStatic) B.x += corr * B.invMass;
        }
    }

    void CaseMultiBoxes::BuildBoxVertsWorld(Body const & b, std::vector<glm::vec3> & V) const {
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

    void CaseMultiBoxes::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Scene", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Simulate", &_simulate);
            ImGui::Checkbox("Gravity", &_useGravity);
            ImGui::Checkbox("Draw Static Env", &_drawStatic);
            ImGui::SliderInt("Dynamic Count", &_dynamicCount, 4, 20);
            if (ImGui::Button("Reset Scene")) ResetScene();
            ImGui::Text("Last contacts: %d", _lastContactCount);
        }

        if (ImGui::CollapsingHeader("Solver", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("Restitution", &_restitution, 0.f, 1.f);
            ImGui::SliderInt("Solver Iterations", &_solverIters, 1, 60);
            ImGui::SliderFloat("Fixed dt", &_fixedDt, 1.f / 1000.f, 1.f / 60.f, "%.6f");
        }

        if (ImGui::CollapsingHeader("Interaction", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderInt("Active Body Index", &_activeBody, 0, (int) _bodies.size() - 1);
            ImGui::SliderFloat("Force Mag", &_forceMag, 0.f, 200.f);
            ImGui::SliderFloat("Torque Mag", &_torqueMag, 0.f, 100.f);
            ImGui::Text("Force: WASD (XZ), RF (Y)");
            ImGui::Text("Torque: Q/E (+/-Y)");
        }
    }

    void CaseMultiBoxes::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);

        if (_activeBody < 0 || _activeBody >= (int) _bodies.size()) return;
        Body & b = _bodies[_activeBody];
        if (b.isStatic) return;

        glm::vec3 f(0.f);
        if (ImGui::IsKeyDown(ImGuiKey_W)) f += glm::vec3(0, 0, -1);
        if (ImGui::IsKeyDown(ImGuiKey_S)) f += glm::vec3(0, 0, 1);
        if (ImGui::IsKeyDown(ImGuiKey_A)) f += glm::vec3(-1, 0, 0);
        if (ImGui::IsKeyDown(ImGuiKey_D)) f += glm::vec3(1, 0, 0);
        if (ImGui::IsKeyDown(ImGuiKey_R)) f += glm::vec3(0, 1, 0);
        if (ImGui::IsKeyDown(ImGuiKey_F)) f += glm::vec3(0, -1, 0);

        if (glm::dot(f, f) > 1e-8f) b.force += glm::normalize(f) * _forceMag;

        if (ImGui::IsKeyDown(ImGuiKey_Q)) b.torque += glm::vec3(0, _torqueMag, 0);
        if (ImGui::IsKeyDown(ImGuiKey_E)) b.torque += glm::vec3(0, -_torqueMag, 0);
    }

    void CaseMultiBoxes::OnProcessMouseControl(glm::vec3 mouseDelta) {
        if (! ImGui::IsMouseDown(ImGuiMouseButton_Right)) return;
        if (! (ImGui::IsKeyDown(ImGuiKey_LeftShift) || ImGui::IsKeyDown(ImGuiKey_RightShift))) return;

        if (_activeBody < 0 || _activeBody >= (int) _bodies.size()) return;
        Body & b = _bodies[_activeBody];
        if (b.isStatic) return;

        bool ctrl = ImGui::IsKeyDown(ImGuiKey_LeftCtrl) || ImGui::IsKeyDown(ImGuiKey_RightCtrl);
        if (ctrl) b.torque += mouseDelta * (_torqueMag * 20.0f);
        else b.force += mouseDelta * (_forceMag * 50.0f);
    }

    Common::CaseRenderResult CaseMultiBoxes::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        OnProcessMouseControl(_cameraManager.getMouseMove());

        float frameDt = ImGui::GetIO().DeltaTime;
        frameDt       = std::clamp(frameDt, 0.0f, 1.0f / 20.0f);

        _lastContactCount = 0;

        if (_simulate) {
            int   steps = std::max(1, (int) std::ceil(frameDt / _fixedDt));
            float dt    = frameDt / float(steps);

            std::vector<Contact> contacts;

            for (int s = 0; s < steps; ++s) {
                for (auto & b : _bodies) IntegrateExplicitEuler(b, dt);

                UpdateFCLObjects();
                DetectContacts(contacts);
                _lastContactCount = (int) contacts.size();

                SolveContactsSequentialImpulse(contacts);
                PositionCorrection(contacts);

                UpdateFCLObjects();
            }
        } else {
            UpdateFCLObjects();
            std::vector<Contact> contacts;
            DetectContacts(contacts);
            _lastContactCount = (int) contacts.size();
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

        for (int i = 0; i < (int) _bodies.size(); ++i) {
            auto const & b = _bodies[i];
            if (b.isStatic && ! _drawStatic) continue;

            BuildBoxVertsWorld(b, verts);
            auto span = Engine::make_span_bytes<glm::vec3>(verts);

            _program.GetUniforms().SetByName("u_Color", b.color);
            _boxItem.UpdateVertexBuffer("position", span);
            _boxItem.Draw({ _program.Use() });

            _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f));
            _lineItem.UpdateVertexBuffer("position", span);
            _lineItem.Draw({ _program.Use() });
        }

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
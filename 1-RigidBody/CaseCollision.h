#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <fcl/fcl.h>
#include <string_view>
#include <vector>

namespace VCX::Labs::RigidBody {

    class CaseCollision : public Common::ICase {
    public:
        CaseCollision();

        std::string_view const GetName() override { return "CaseCollision: Box-Box (FCL + Impulse)"; }

        void                     OnSetupPropsUI() override;
        Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        struct RigidBox {
            float mass { 1.f };
            float invMass { 1.f };
            glm::vec3 dim { 1.f, 1.f, 1.f };
            glm::vec3 x { 0.f };               
            glm::vec3 v { 0.f };          
            glm::quat q { 1.f, 0.f, 0.f, 0.f }; 
            glm::vec3 w { 0.f };             
            glm::mat3 Ibody { 1.f };
            glm::mat3 IbodyInv { 1.f };
            glm::vec3 force { 0.f };
            glm::vec3 torque { 0.f };
            glm::vec3 color { 0.7f, 0.7f, 0.7f };

            void ClearAcc() {
                force  = glm::vec3(0.f);
                torque = glm::vec3(0.f);
            }
        };

        enum class Scenario : int {
            HeadOn = 0,
            CornerFace,
            EdgeEdge,
            FaceFace,
        };

    private:
        glm::mat3 ComputeInertiaBody(float mass, glm::vec3 const & dim) const;
        void      ResetScenario(Scenario s);
        void      IntegrateExplicitEuler(RigidBox & b, float dt);
        void      ApplyImpulseAtWorldPoint(RigidBox & b, glm::vec3 const & p, glm::vec3 const & J);
        glm::mat3 WorldInertiaInv(RigidBox const & b) const;
        void UpdateFCLObjects();
        void SolveCollisionImpulse(float dt);
        void BuildBoxVertsWorld(RigidBox const & b, std::vector<glm::vec3> & verts) const;

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-5, 4, 5) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;
        bool  _simulate { true };
        float _restitution { 0.2f };
        int   _solverIters { 10 };
        float _fixedDt { 1.f / 60.f };
        int   _activeBody { 0 }; 
        float _forceMag { 25.f };
        float _torqueMag { 10.f };
        Scenario _scenario { Scenario::HeadOn };
        RigidBox _A;
        RigidBox _B;
        std::shared_ptr<fcl::CollisionGeometryd> _geomA;
        std::shared_ptr<fcl::CollisionGeometryd> _geomB;
        fcl::CollisionObjectd                    _objA;
        fcl::CollisionObjectd                    _objB;
        struct DebugContact {
            glm::vec3 p;
            glm::vec3 n;
            float     depth;
        };
        std::vector<DebugContact> _contacts;
    };

} // namespace VCX::Labs::RigidBody
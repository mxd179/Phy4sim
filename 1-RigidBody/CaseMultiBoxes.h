#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <fcl/fcl.h>

#include <memory>
#include <string_view>
#include <vector>

namespace VCX::Labs::RigidBody {

    class CaseMultiBoxes : public Common::ICase {
    public:
        CaseMultiBoxes();

        std::string_view const GetName() override { return "CaseMultiBoxes: Many Boxes (FCL + Impulse)"; }

        void                     OnSetupPropsUI() override;
        Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        void                     OnProcessInput(ImVec2 const & pos) override;
        void                     OnProcessMouseControl(glm::vec3 mouseDelta);

    private:
        struct Body {
            bool isStatic { false };

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

        struct Contact {
            int       a { -1 };
            int       b { -1 };
            glm::vec3 p { 0.f };          
            glm::vec3 n { 0.f, 1.f, 0.f };
            float     depth { 0.f };
        };

    private:
        glm::mat3 ComputeInertiaBody(float mass, glm::vec3 const & dim) const;
        glm::mat3 WorldInertiaInv(Body const & b) const;
        void      IntegrateExplicitEuler(Body & b, float dt);
        void      ApplyImpulseAtWorldPoint(Body & b, glm::vec3 const & p, glm::vec3 const & J);
        void ResetScene();
        void UpdateFCLObjects();
        void DetectContacts(std::vector<Contact> & contacts);
        void SolveContactsSequentialImpulse(std::vector<Contact> const & contacts);
        void PositionCorrection(std::vector<Contact> const & contacts);
        void BuildBoxVertsWorld(Body const & b, std::vector<glm::vec3> & verts) const;

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-7, 6, 7) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;
        std::vector<Body> _bodies;
        std::vector<std::shared_ptr<fcl::CollisionGeometryd>> _geoms;
        std::vector<fcl::CollisionObjectd>                    _objs;
        bool  _simulate { true };
        bool  _useGravity { true };
        float _restitution { 0.05f };
        int   _solverIters { 20 };
        float _fixedDt { 1.f / 60.f };
        int   _activeBody { 0 }; 
        float _forceMag { 40.f };
        float _torqueMag { 15.f };
        int  _dynamicCount { 6 };
        bool _drawStatic { true };
        int _lastContactCount { 0 };
    };

} // namespace VCX::Labs::RigidBody
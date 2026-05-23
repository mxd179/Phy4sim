#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <glm/gtc/quaternion.hpp>

namespace VCX::Labs::RigidBody {
    class CaseBox : public Common::ICase {
    public:
        CaseBox();

        virtual std::string_view const GetName() override { return "Single Box"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;
        void Moving();

    private:
        void      ResetRigidBody();
        void      StepPhysicsExplicitEuler(float dt);
        void      ClearAccumulators();
        void      AddForceWorld(glm::vec3 const & f);
        void      AddTorqueWorld(glm::vec3 const & t);
        glm::mat3 ComputeInertiaBody(float mass, glm::vec3 const & dim) const;

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem _lineItem; // render line on box

        glm::vec3 _center { 0.f, 0.f, 0.f }; 
        glm::vec3 _dim { 1.f, 2.f, 3.f };

        float velocity = 0.f; 
        float omega    = 0.f;

        glm::vec3 _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };

        float     _mass { 1.0f };
        glm::vec3 _v { 0.f, 0.f, 0.f };      
        glm::quat _q { 1.f, 0.f, 0.f, 0.f }; 
        glm::vec3 _wBody { 0.f, 0.f, 0.f }; 

        glm::mat3 _Ibody { 1.0f };
        glm::mat3 _IbodyInv { 1.0f };

        glm::vec3 _forceWorld { 0.f, 0.f, 0.f };
        glm::vec3 _torqueWorld { 0.f, 0.f, 0.f };

        bool  _simulate { true };
        float _forceMag { 20.f };
        float _torqueMag { 8.f };
    };
} // namespace VCX::Labs::RigidBody
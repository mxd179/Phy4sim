#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"

#include "Labs/3-FEM/FEMSimulator.h"

#include <cstdint>
#include <vector>

namespace VCX::Labs::FEM {

    class Bonus3Animation : public Common::ICase {
    public:
        Bonus3Animation();
        virtual std::string_view const GetName() override { return "Bonus3: Collision Animation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        void ResetScene();
        void RebuildSoftLineIndex();
        void RebuildSphereViz();

        int GetID(int i, int j, int k) const;

        float Ramp(float t, float t0, float t1) const;

    private:
        Engine::GL::UniqueProgram     _program;
        Engine::GL::UniqueRenderFrame _frame;

        Engine::Camera             _camera { .Eye = glm::vec3(-3.2f, 2.2f, 3.2f) };
        Common::OrbitCameraManager _cameraManager;

        Engine::GL::UniqueIndexedRenderItem _softPointItem;
        Engine::GL::UniqueIndexedRenderItem _softLineItem;
        Engine::GL::UniqueIndexedRenderItem _sphereLineItem;

        Simulator _sim;

        bool  _simulate { true };
        bool  _stepOnce { false };
        float _dt { 0.001f };
        float _time { 0.0f };

        int   _wx { 16 }, _wy { 4 }, _wz { 4 };
        float _dx { 0.05f };

        int       _model { 3 };   
        float     _young { 8000.0f }; 
        float     _nu { 0.2f };
        float     _rho { 400.0f };
        float     _damping { 0.03f };
        glm::vec3 _gravity { 0.0f, -0.5f, 0.0f }; 
        bool      _sphereOn { true };
        glm::vec3 _sphereC { 0.48f, -0.05f, 0.0f };
        float     _sphereR { 0.14f };
        bool  _groundOn { false };
        float _groundY { -0.25f };
        float _colK { 12000.0f };
        float _colDamp { 120.0f };
        float _colMaxDepth { 0.03f };
        bool      _forceOn { true };
        float     _forceMagnitude { 120.0f }; 
        glm::vec3 _forceDir { 1.0f, 0.0f, 0.0f };
        float _tRamp { 0.30f };
        int _forceVid { 0 }; 
        glm::vec3 _softColor { 0.25f, 0.65f, 0.95f };
        glm::vec3 _sphereColor { 1.0f, 1.0f, 1.0f };

        std::vector<glm::vec3>     _sphereVerts;
        std::vector<std::uint32_t> _sphereIdx;
    };

} // namespace VCX::Labs::FEM
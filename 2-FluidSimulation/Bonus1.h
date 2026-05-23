#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"

#include "Labs/2-FluidSimulation/FluidSimulator.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace VCX::Labs::Fluid {

    class Bonus1 : public Common::ICase {
    public:
        Bonus1();

        virtual std::string_view const GetName() override { return "Fluid Bonus 1 (Viz + Obstacle)"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        void ResetScene();
        void StepFluid(float dt);
        void UpdateParticleColorsBonus();

    private:
        Engine::GL::UniqueProgram _flatProgram;    
        Engine::GL::UniqueProgram _particleProgram;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::Camera                _camera { .Eye = glm::vec3(-2.2f, 1.6f, 2.2f) };
        Common::OrbitCameraManager    _cameraManager;

        Engine::GL::UniqueIndexedRenderItem _particleItem;   
        Engine::GL::UniqueIndexedRenderItem _tankLineItem;   
        Engine::GL::UniqueIndexedRenderItem _sphereLineItem;
        Simulator _sim;
        bool _simulate { true };
        bool _stepOnce { false };
        int  _res { 50 };
        float _dt { 1.0f / 60.0f }; 
        float _flipRatio { 0.95f }; 
        int   _colorMode { 0 };
        float _pointSize { 4.0f };
        glm::vec3 _tankColor { 1.0f, 1.0f, 1.0f };
        glm::vec3 _obstacleColor { 1.0f, 0.8f, 0.1f };
        bool      _enableObstacle { true };
        float     _obstacleRadius { 0.08f };
        glm::vec3 _obstaclePos { 0.0f, 0.0f, 0.0f };
        glm::vec3 _obstacleVel { 0.0f, 0.0f, 0.0f };
        bool      _dragging { false };

        std::pair<std::uint32_t, std::uint32_t> _lastRenderSize { 1, 1 };
    };

} // namespace VCX::Labs::Fluid
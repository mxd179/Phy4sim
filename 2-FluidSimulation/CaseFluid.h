
#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"

#include "Labs/2-FluidSimulation/FluidSimulator.h" 

#include <cstdint>
#include <vector>

namespace VCX::Labs::Fluid {

    class CaseFluid : public Common::ICase {
    public:
        CaseFluid();

        virtual std::string_view const GetName() override { return "Fluid (PIC/FLIP)"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        void ResetScene();

    private:
        Engine::GL::UniqueProgram     _program;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::Camera                _camera { .Eye = glm::vec3(-2.2f, 1.6f, 2.2f) };
        Common::OrbitCameraManager    _cameraManager;

        Engine::GL::UniqueIndexedRenderItem _particleItem; 
        Engine::GL::UniqueIndexedRenderItem _boxLineItem;  

        Simulator _sim;
        bool _simulate { true };
        bool _stepOnce { false };
        int  _res { 50 };

        float _dt { 1.0f / 60.0f }; 
        float _flipRatio { 0.95f }; 

        glm::vec3 _particleColor { 0.2f, 0.6f, 0.95f };
        glm::vec3 _boxColor { 1.0f, 1.0f, 1.0f };
    };

} // namespace VCX::Labs::Fluid
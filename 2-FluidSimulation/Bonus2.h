#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"

#include "Labs/2-FluidSimulation/FluidSimulator.h"

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace VCX::Labs::Fluid {

    class Bonus2 : public Common::ICase {
    public:
        Bonus2();

        virtual std::string_view const GetName() override { return "Fluid Bonus 2 (APIC)"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        void ResetScene();
        void Step(float dt);

        void APIC_ToGrid();
        void APIC_ToParticles();

    private:
        enum class Method {
            PIC    = 0,
            FLIP95 = 1,
            APIC   = 2
        };

        Engine::GL::UniqueProgram     _program;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::Camera                _camera { .Eye = glm::vec3(-2.2f, 1.6f, 2.2f) };
        Common::OrbitCameraManager    _cameraManager;

        Engine::GL::UniqueIndexedRenderItem _particleItem;
        Engine::GL::UniqueIndexedRenderItem _tankLineItem;

        Simulator _sim;

        std::vector<std::array<glm::vec3, 3>> _apicA;

        bool  _simulate { true };
        bool  _stepOnce { false };
        int   _res { 50 };
        float _dt { 1.0f / 60.0f };

        Method _method { Method::APIC };
        float  _flipRatio { 0.95f }; 

        float     _pointSize { 4.0f };
        glm::vec3 _tankColor { 1.0f, 1.0f, 1.0f };
        glm::vec3 _particleColor { 0.2f, 0.6f, 0.95f };
    };

} // namespace VCX::Labs::Fluid
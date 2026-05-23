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

    class CaseFEM : public Common::ICase {
    public:
        CaseFEM();

        virtual std::string_view const GetName() override { return "FEM Softbody (Explicit, Linear)"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        void ResetScene();
        void RebuildLineIndex();
        void OnProcessMouseControl(glm::vec3 mouseDelta);

    private:
        Engine::GL::UniqueProgram     _program;
        Engine::GL::UniqueRenderFrame _frame;

        Engine::Camera             _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager _cameraManager;

        Engine::GL::UniqueIndexedRenderItem _pointItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;

        Simulator _sim;

        bool _simulate { true };
        bool _stepOnce { false };
        int   _wx { 16 }, _wy { 4 }, _wz { 4 };
        float _dx { 0.05f };
        float     _dt { 0.001f };
        float     _young { 20000.0f };
        float     _nu { 0.2f };
        float     _rho { 400.0f };
        float     _damping { 0.03f };
        glm::vec3 _gravity { 0.0f, -0.05f, 0.0f };
        bool _enableDrag { true };

        float _dragMoveScale { 0.1f };

        float _dragK { 1500.0f };
        float _dragDamp { 80.0f };
        float _maxDragForce { 8000.0f };

        bool      _dragging { false };
        glm::vec3 _dragTarget { 0.0f };
        glm::vec3 _pointColor { 0.2f, 0.8f, 0.4f };
        glm::vec3 _lineColor { 1.0f, 1.0f, 1.0f };
    };

} // namespace VCX::Labs::FEM
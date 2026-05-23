#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"

#include "Labs/3-FEM/ClothSimulator.h"

#include <cstdint>
#include <vector>

namespace VCX::Labs::FEM {

    class Bonus2 : public Common::ICase {
    public:
        Bonus2();
        virtual std::string_view const GetName() override { return "Bonus2: Cloth FEM"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        void ResetScene();
        void RebuildLineIndex();

        bool ApplyKeyboardForce();

        void SuppressCameraKeysBegin();
        void SuppressCameraKeysEnd();

        static glm::vec3 ClampVec(glm::vec3 v, float maxLen);

    private:
        Engine::GL::UniqueProgram     _program;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::Camera                _camera { .Eye = glm::vec3(-3, 2.5f, 3) };
        Common::OrbitCameraManager    _cameraManager;

        Engine::GL::UniqueIndexedRenderItem _pointItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;

        ClothSimulator _sim;

        bool  _simulate { true };
        bool  _stepOnce { false };
        float _dt { 0.001f };

        int   _w { 25 }, _h { 25 };
        float _dx { 0.05f };

        float     _young { 10.0f };
        float     _nu { 0.3f };
        float     _rhoA { 0.5f };
        float     _damping { 0.02f };
        glm::vec3 _gravity { 0.0f, -9.8f, 0.0f };
        bool      _pinCorners { true };

        bool _enableKeyboardForce { true };
        float _keyAccel { 80.0f };
        float _keyAccelBoost { 3.0f }; 
        float _maxKeyAccel { 300.0f }; 

        bool _requireCtrlForForce { false };

        bool _suppressCameraKeysThisFrame { false };
        bool _savedKeyDown[6] { false, false, false, false, false, false };

        glm::vec3 _color { 0.9f, 0.9f, 0.9f };
    };

} // namespace VCX::Labs::FEM
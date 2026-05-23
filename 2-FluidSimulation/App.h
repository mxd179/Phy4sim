#pragma once

#include "CaseFluid.h"
#include "Engine/app.h"
#include <vector>
#include "Labs/Common/UI.h"
#include "Bonus1.h"
#include "Bonus2.h"
#include "CaseFEM.h"

namespace VCX::Labs::Fluid {
    class App : public Engine::IApp {
    private:
        Common::UI     _ui;
        CaseFluid   _caseFluid;
        Bonus1         _bonus1;
        Bonus2         _bonus2;
        CaseFEM        _caseFEM;
        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseFluid,_bonus1,_bonus2,_caseFEM };

    public:
        App();

        void OnFrame() override;
    };
}
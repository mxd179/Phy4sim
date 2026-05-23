#pragma once


#include "CaseFEM.h"
#include "Bonus1.h"
#include "Bonus2.h"
#include "Bonus3Animation.h"
#include "Engine/app.h"
#include "Labs/Common/UI.h"
#include <vector>

namespace VCX::Labs::FEM {
    class App : public Engine::IApp {
    private:
        Common::UI  _ui;
        CaseFEM     _caseFEM;  
        Bonus1                                                      _bonus1;
        Bonus2      _bonus2;
        Bonus3Animation                                             _bonus3animation;
        std::size_t _caseId = 0;
        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseFEM,_bonus1,_bonus2,_bonus3animation};

    public:
        App();

        void OnFrame() override;
    };
}
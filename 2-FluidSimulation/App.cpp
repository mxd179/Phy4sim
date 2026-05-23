#include "Labs/2-FluidSimulation/App.h"
#include "Assets/bundled.h"

namespace VCX::Labs::Fluid {

    App::App():
        _ui(Labs::Common::UIOptions {}),
        _caseFluid(),
        _bonus1(),
        _bonus2(){
        _cases = { _caseFluid,_bonus1,_bonus2 };
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
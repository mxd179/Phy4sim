#include "Labs/3-FEM/App.h"
#include "Assets/bundled.h"

namespace VCX::Labs::FEM {
    App::App():
        _ui(Labs::Common::UIOptions {}),
        _caseFEM(),
        _bonus1(),
    _bonus2(),
    _bonus3animation(){
        _cases = { _caseFEM,_bonus1,_bonus2,_bonus3animation };
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
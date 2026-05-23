#pragma once

#include <vector>
#include "CaseBox.h"
#include "Engine/app.h"
#include "CaseCollision.h"
#include "CaseMultiBoxes.h"
//#include"CaseStacking.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;
        CaseBox        _caseBox;
        CaseCollision _caseCollision;
        CaseMultiBoxes _caseMultiBoxes;
        //CaseStacking      _caseStacking;
        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseBox, _caseCollision, _caseMultiBoxes};

    public:
        App();

        void OnFrame() override;
    };
}

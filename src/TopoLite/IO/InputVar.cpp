//
// Created by ziqwang on 10.03.19.
//
#include "InputVar.h"

void InitVar(InputVarList *varList){
    varList->clear();
    varList->add(1e-8, "equilibrium_infeasibility_tol", "equilibrium_infeasibility_tol");
    varList->add(0.5, "friction_coeff", "friction_coeff");
    varList->add(1e-10, "intpntTolRelGap", "intpntTolRelGap");
}
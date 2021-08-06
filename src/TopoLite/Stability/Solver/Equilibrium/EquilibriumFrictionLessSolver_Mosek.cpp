//
// Created by ziqwang on 14.08.20.
//

#include "EquilibriumFrictionLessSolver_Mosek.h"


template<int dim>
void EquilibriumFrictionLessSolver_Mosek<dim>::unpackStaticData(const vector<double> &xx,
                                                           shared_ptr<StaticData> &data){

    if(data) data.reset();
    getEquilibriumData(data);

    for(int id = 0; id < xx.size(); id++)
    {
        //std::cout << "[ " << id << " ]:\t" << forces_norm[id] << std::endl;
        data->forces[id] *= xx[id];
    }
}

template<int dim>
void EquilibriumFrictionLessSolver_Mosek<dim>::unpackKinematicData(const vector<double> &velocities,
                                                              shared_ptr<KinematicData> &data) {
    getKinetamticData(data, velocities);
}

template<int dim>
bool EquilibriumFrictionLessSolver_Mosek<dim>::isEquilibrium_Variables(Vector3 gravity, shared_ptr<StaticData> &data){

    Eigen::MatrixXd Aeq;

    computeEquilibriumMatrix(Aeq, true, false);

    int NUMVAR = Aeq.cols();
    int NUMCON = Aeq.rows();


    Eigen::VectorXd x(NUMVAR), g(NUMCON);
    getForces(gravity, g, false);

    //boundary condition for linear constraint Aeq
    vector<MSKboundkeye>  bkc;
    vector<double>        blc;
    vector<double>        buc;


    for(int id = 0; id < NUMCON; id++){
        blc.push_back(g(id));
        buc.push_back(g(id));
        bkc.push_back(MSK_BK_FX);
    }

    //boundary condition for variables
    vector<MSKboundkeye>  bkx;
    vector<double> blx, bux;
    for(int id = 0; id < NUMVAR; id++){
        bkx.push_back(MSK_BK_LO);
        blx.push_back(0);
        bux.push_back(+MSK_INFINITY);
    }

    MSKint32t     i, j;
    vector<double>  xx; xx.resize(NUMVAR);
    vector<double>  forces_norm; forces_norm.resize(NUMVAR / 2);

    MSKenv_t      env = NULL;
    MSKtask_t     task = NULL;
    MSKrescodee   r;

    double infeasible_force = 0;
    double primalobj = +MSK_INFINITY;

    /* Create the mosek environment. */
    r = MSK_makeenv(&env, NULL);

    if (r == MSK_RES_OK) {
        /* Create the optimization task. */
        r = MSK_maketask(env, NUMCON, NUMVAR, &task);

        if (r == MSK_RES_OK) {
            //r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);

            /* Append 'NUMCON' empty constraints.
             The constraints will initially have no bounds. */
            if (r == MSK_RES_OK)
                r = MSK_appendcons(task, NUMCON);

            /* Append 'NUMVAR' variables.
             The variables will initially be fixed at zero (x=0). */
            if (r == MSK_RES_OK)
                r = MSK_appendvars(task, NUMVAR);

            /* Optionally add a constant term to the objective. */
            if (r == MSK_RES_OK) {
                r = MSK_putcfix(task, 0.0);
            }

            for (j = 0; j < NUMVAR && r == MSK_RES_OK; ++j) {

                /* Set the bounds on variable j.
                 blx[j] <= x_j <= bux[j] */
                if (r == MSK_RES_OK)
                    r = MSK_putvarbound(task,
                                        j,           /* Index of variable.*/
                                        bkx[j],      /* Bound key.*/
                                        blx[j],      /* Numerical value of lower bound.*/
                                        bux[j]);     /* Numerical value of upper bound.*/
            }


            for (i = 0; i < NUMCON && r == MSK_RES_OK; ++i) {

                for (j = 0; j < NUMVAR && r == MSK_RES_OK; ++j) {
                    r = MSK_putaij(task, i, j, Aeq(i, j));
                }

                /* Set the bounds on constraints.
                for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
                r = MSK_putconbound(task,
                                    i,           /* Index of constraint.*/
                                    bkc[i],      /* Bound key.*/
                                    blc[i],      /* Numerical value of lower bound.*/
                                    buc[i]);     /* Numerical value of upper bound.*/
            }

            if (r == MSK_RES_OK) {
                //set objectives
                for (int i = 0; i < NUMVAR / 2; i++) {
                    MSK_putqobjij(task, 2 * i + 1, 2 * i + 1, 2);
                }
            }

            if (r == MSK_RES_OK) {
                MSKrescodee trmcode;

                /* Run optimizer */
                r = MSK_optimizetrm(task, &trmcode);

//                /* Print a summary containing information
//                   about the solution for debugging purposes*/
                MSK_solutionsummary(task, MSK_STREAM_MSG);

                if (r == MSK_RES_OK) {
                    MSKsolstae solsta;
                    int j;

                    MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

                    switch (solsta) {

                        case MSK_SOL_STA_OPTIMAL:
                            MSK_getxx(task,
                                      MSK_SOL_ITR,    /* Request the interior solution. */
                                      xx.data());

                            for (j = 0; j < forces_norm.size(); ++j){
                                forces_norm[j] = xx[2 * j];
                            }

                            for(j = 0; j < xx.size(); ++j){
                                x(j) = xx[j];
                            }

                            unpackStaticData(forces_norm, data);

                            if(silence) std::cout << "Equlibrium Constraints:\t" <<(Aeq * x - g).norm() << std::endl;

                            MSK_getprimalobj(task, MSK_SOL_ITR, &primalobj);

                            if(silence) std::cout << "Infeasible Force:\t" << primalobj << std::endl;

                            data->energy = primalobj;

                            break;

                        case MSK_SOL_STA_DUAL_INFEAS_CER:
                        case MSK_SOL_STA_PRIM_INFEAS_CER:
                            printf("Primal or dual infeasibility certificate found.\n");
                            break;

                        case MSK_SOL_STA_UNKNOWN:
                            printf("The status of the solution could not be determined. Termination code: %d.\n",
                                   trmcode);
                            break;

                        default:
                            printf("Other solution status.");
                            break;
                    }
                } else {
                    printf("Error while optimizing.\n");
                }
            }

            if (r != MSK_RES_OK) {
                /* In case of an error print error code and description. */
                char symname[MSK_MAX_STR_LEN];
                char desc[MSK_MAX_STR_LEN];

                printf("An error occurred while optimizing.\n");
                MSK_getcodedesc(r,
                                symname,
                                desc);
                printf("Error %s - '%s'\n", symname, desc);
            }
        }
        MSK_deletetask(&task);
    }
    MSK_deleteenv(&env);

    if(primalobj < 1E-5){
        return true;
    }
    else{
        return false;
    }
}

template<int dim>
bool EquilibriumFrictionLessSolver_Mosek<dim>::isEquilibrium_Constraints(Vector3 gravity, shared_ptr<StaticData> &data){

    Eigen::MatrixXd A;

    computeEquilibriumMatrix(A, false, true);

    int NUMVAR = A.cols();
    int NUMCON = A.rows();

    NUMVAR += 2 * NUMCON;

    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(NUMCON, NUMVAR);
    Aeq.block(0, 0, A.rows(), A.cols()) = A;
    for(int id = 0; id < NUMCON; id++){
        Aeq(id, A.cols() + 2 * id) = 1;
        Aeq(id, A.cols() + 2 * id + 1) = -1;
    }

    std::cout << "NUMVAR:\t" << NUMVAR << std::endl;

    Eigen::VectorXd x(NUMVAR), g(NUMCON), f(A.cols());

    //boundary condition for linear constraint Aeq
    vector<MSKboundkeye>  bkc;
    vector<double>        blc;
    vector<double>        buc;

    getForces(gravity, g, true);

    std::cout << "Aeq:\n" << Aeq << std::endl;
    std::cout << "g:\n" << g << std::endl;


    for(int id = 0; id < NUMCON; id++){
        blc.push_back(g(id));
        buc.push_back(g(id));
        bkc.push_back(MSK_BK_FX);
    }

    //boundary condition for variables
    vector<MSKboundkeye>  bkx;
    vector<double> blx, bux;
    for(int id = 0; id < NUMVAR; id++){
        bkx.push_back(MSK_BK_LO);
        blx.push_back(0);
        bux.push_back(+MSK_INFINITY);
    }

    MSKint32t     i, j;
    vector<double>  xx; xx.resize(NUMVAR);
    vector<double> forces_norm; forces_norm.resize(A.cols());

    MSKenv_t      env = NULL;
    MSKtask_t     task = NULL;
    MSKrescodee   r;

    double infeasible_force = 0;
    double primalobj = +MSK_INFINITY;

    /* Create the mosek environment. */
    r = MSK_makeenv(&env, NULL);

    if (r == MSK_RES_OK) {
        /* Create the optimization task. */
        r = MSK_maketask(env, NUMCON, NUMVAR, &task);

        if (r == MSK_RES_OK) {
            //r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);

            /* Append 'NUMCON' empty constraints.
             The constraints will initially have no bounds. */
            if (r == MSK_RES_OK)
                r = MSK_appendcons(task, NUMCON);

            /* Append 'NUMVAR' variables.
             The variables will initially be fixed at zero (x=0). */
            if (r == MSK_RES_OK)
                r = MSK_appendvars(task, NUMVAR);

            for (j = 0; j < NUMVAR && r == MSK_RES_OK; ++j) {
                /* Set the bounds on variable j.
                 blx[j] <= x_j <= bux[j] */
                if (r == MSK_RES_OK)
                    r = MSK_putvarbound(task,
                                        j,           /* Index of variable.*/
                                        bkx[j],      /* Bound key.*/
                                        blx[j],      /* Numerical value of lower bound.*/
                                        bux[j]);     /* Numerical value of upper bound.*/
            }


            for (i = 0; i < NUMCON && r == MSK_RES_OK; ++i) {

                for (j = 0; j < NUMVAR && r == MSK_RES_OK; ++j) {
                    r = MSK_putaij(task, i, j, Aeq(i, j));
                }

                /* Set the bounds on constraints.
                for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
                r = MSK_putconbound(task,
                                    i,           /* Index of constraint.*/
                                    bkc[i],      /* Bound key.*/
                                    blc[i],      /* Numerical value of lower bound.*/
                                    buc[i]);     /* Numerical value of upper bound.*/
            }

            if (r == MSK_RES_OK) {
                //set objectives
                for (int i = 0; i < NUMVAR && r == MSK_RES_OK; i++) {
                    if(i < A.cols()){
                        r = MSK_putcj(task, i, 0);
                    }
                    else{
                        r = MSK_putcj(task, i, 1);
                    }
                }
            }

            if (r == MSK_RES_OK) {
                MSKrescodee trmcode;

                /* Run optimizer */
                r = MSK_optimizetrm(task, &trmcode);

//                /* Print a summary containing information
//                   about the solution for debugging purposes*/
//                MSK_solutionsummary(task, MSK_STREAM_MSG);

                if (r == MSK_RES_OK) {
                    MSKsolstae solsta;
                    int j;

                    MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

                    switch (solsta) {

                        case MSK_SOL_STA_OPTIMAL:
                            MSK_getxx(task,
                                      MSK_SOL_ITR,    /* Request the interior solution. */
                                      xx.data());

                            std::cout << "x:\n";
                            for (j = 0; j < NUMVAR; ++j){
                                x(j) = xx[j];
                                std::cout << xx[j] << std::endl;
                            }

                            for(j = 0; j < A.cols(); ++j){
                                forces_norm[j] = xx[j];
                                f(j) = xx[j];
                            }

                            unpackStaticData(forces_norm, data);

                            if(!silence) std::cout << "Equlibrium Constraints:\t" <<(A * f - g).norm() << std::endl;

                            MSK_getprimalobj(task, MSK_SOL_ITR, &primalobj);

                            if(!silence) std::cout << "Infeasible Force:\t" << primalobj << std::endl;

                            data->energy = primalobj;

                            break;

                        case MSK_SOL_STA_DUAL_INFEAS_CER:
                        case MSK_SOL_STA_PRIM_INFEAS_CER:
                            printf("Primal or dual infeasibility certificate found.\n");
                            break;

                        case MSK_SOL_STA_UNKNOWN:
                            printf("The status of the solution could not be determined. Termination code: %d.\n",
                                   trmcode);
                            break;

                        default:
                            printf("Other solution status.");
                            break;
                    }
                } else {
                    printf("Error while optimizing.\n");
                }
            }

            if (r != MSK_RES_OK) {
                /* In case of an error print error code and description. */
                char symname[MSK_MAX_STR_LEN];
                char desc[MSK_MAX_STR_LEN];

                printf("An error occurred while optimizing.\n");
                MSK_getcodedesc(r,
                                symname,
                                desc);
                printf("Error %s - '%s'\n", symname, desc);
            }
        }
        MSK_deletetask(&task);
    }
    MSK_deleteenv(&env);

    if(primalobj < 1E-5){
        return true;
    }
    else{
        return false;
    }
}

template<int dim>
bool EquilibriumFrictionLessSolver_Mosek<dim>::isEquilibrium_Dual(Vector3 gravity, shared_ptr<KinematicData>  &data){
    Eigen::MatrixXd A, Bint;

    computeEquilibriumMatrix(A, false, true);
    Bint = A.transpose();

    int NUMVAR = Bint.cols();
    int NUMCON = Bint.rows();

    std::cout << NUMCON << std::endl;

    Eigen::VectorXd x(NUMVAR), g(NUMVAR);

    //boundary condition for linear constraint Aeq
    vector<MSKboundkeye>  bkc;
    vector<double>        blc;
    vector<double>        buc;

    for(int id = 0; id < NUMCON; id++){
        blc.push_back(0);
        buc.push_back(+MSK_INFINITY);
        bkc.push_back(MSK_BK_LO);
    }

    //boundary condition for variables
    vector<MSKboundkeye>  bkx;
    vector<double> blx, bux;
    for(int id = 0; id < NUMVAR; id++){

        bkx.push_back(MSK_BK_RA);
        blx.push_back(-1);
        bux.push_back(1);
    }

    getForces(gravity, g, true);

    MSKint32t     i, j;
    vector<double>  xx; xx.resize(NUMVAR);

    MSKenv_t      env = NULL;
    MSKtask_t     task = NULL;
    MSKrescodee   r;

    double primalobj = 0;

    /* Create the mosek environment. */
    r = MSK_makeenv(&env, NULL);

    if (r == MSK_RES_OK) {
        /* Create the optimization task. */
        r = MSK_maketask(env, NUMCON, NUMVAR, &task);

        if (r == MSK_RES_OK) {
            //r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);

            /* Append 'NUMCON' empty constraints.
             The constraints will initially have no bounds. */
            if (r == MSK_RES_OK)
                r = MSK_appendcons(task, NUMCON);

            /* Append 'NUMVAR' variables.
             The variables will initially be fixed at zero (x=0). */
            if (r == MSK_RES_OK)
                r = MSK_appendvars(task, NUMVAR);

            /* Optionally add a constant term to the objective. */
            if (r == MSK_RES_OK) {
                r = MSK_putcfix(task, 0.0);
            }

            for (j = 0; j < NUMVAR && r == MSK_RES_OK; ++j) {

                /* Set the bounds on variable j.
                 blx[j] <= x_j <= bux[j] */
                if (r == MSK_RES_OK)
                    r = MSK_putvarbound(task,
                                        j,           /* Index of variable.*/
                                        bkx[j],      /* Bound key.*/
                                        blx[j],      /* Numerical value of lower bound.*/
                                        bux[j]);     /* Numerical value of upper bound.*/
            }


            for (i = 0; i < NUMCON && r == MSK_RES_OK; ++i) {

                for (j = 0; j < NUMVAR && r == MSK_RES_OK; ++j) {
                    r = MSK_putaij(task, i, j, Bint(i, j));
                }

                /* Set the bounds on constraints.
                for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
                r = MSK_putconbound(task,
                                    i,           /* Index of constraint.*/
                                    bkc[i],      /* Bound key.*/
                                    blc[i],      /* Numerical value of lower bound.*/
                                    buc[i]);     /* Numerical value of upper bound.*/
            }

            if (r == MSK_RES_OK) {
                //set objectives
                for (int i = 0; i < NUMVAR; i++) {
                    MSK_putcj(task, i, g(i));
                }
            }

            if (r == MSK_RES_OK) {
                MSKrescodee trmcode;

                /* Run optimizer */
                r = MSK_optimizetrm(task, &trmcode);

//                /* Print a summary containing information
//                   about the solution for debugging purposes*/
                MSK_solutionsummary(task, MSK_STREAM_MSG);

                if (r == MSK_RES_OK) {
                    MSKsolstae solsta;
                    int j;

                    MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

                    switch (solsta) {

                        case MSK_SOL_STA_OPTIMAL:
                            MSK_getxx(task,
                                      MSK_SOL_ITR,    /* Request the interior solution. */
                                      xx.data());

                            for(j = 0; j < xx.size(); ++j){
                                x(j) = xx[j];
                            }

                            unpackKinematicData(xx, data);

                            if(!silence) std::cout << "Kinematic Constraints:\t" << (Bint * x).colwise().minCoeff() << std::endl;

                            MSK_getprimalobj(task, MSK_SOL_ITR, &primalobj);

                            if(!silence) std::cout << "Infeasible Force:\t" << -primalobj << std::endl;

                            data->energy = -primalobj;

                            break;

                        case MSK_SOL_STA_DUAL_INFEAS_CER:
                        case MSK_SOL_STA_PRIM_INFEAS_CER:
                            printf("Primal or dual infeasibility certificate found.\n");
                            break;

                        case MSK_SOL_STA_UNKNOWN:
                            printf("The status of the solution could not be determined. Termination code: %d.\n",
                                   trmcode);
                            break;

                        default:
                            printf("Other solution status.");
                            break;
                    }
                } else {
                    printf("Error while optimizing.\n");
                }
            }

            if (r != MSK_RES_OK) {
                /* In case of an error print error code and description. */
                char symname[MSK_MAX_STR_LEN];
                char desc[MSK_MAX_STR_LEN];

                printf("An error occurred while optimizing.\n");
                MSK_getcodedesc(r,
                                symname,
                                desc);
                printf("Error %s - '%s'\n", symname, desc);
            }
        }
        MSK_deletetask(&task);
    }
    MSK_deleteenv(&env);

    if(data->energy < 1E-5){
        return true;
    }
    else{
        return false;
    }
}

template<int dim>
bool EquilibriumFrictionLessSolver_Mosek<dim>::isEquilibrium(Vector3 gravity, shared_ptr<StaticData> &data)
{
    //return isEquilibrium_Constraints(gravity, data);
    shared_ptr<KinematicData> kdata;
    return isEquilibrium_Dual(gravity, kdata);
}

template class EquilibriumFrictionLessSolver_Mosek<3>;
template class EquilibriumFrictionLessSolver_Mosek<2>;

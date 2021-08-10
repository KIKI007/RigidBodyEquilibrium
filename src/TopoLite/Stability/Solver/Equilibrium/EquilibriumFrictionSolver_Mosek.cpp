//
// Created by ziqwang on 09.08.21.
//

#include "EquilibriumFrictionSolver_Mosek.h"

void EquilibriumFrictionSolver_Mosek::unpackStaticData(const vector<double> &force_value,
                                                       shared_ptr<StaticData> &data){

    if(data) data.reset();
    int contact_index = 0;
    data = make_shared<StaticData>();
    for(int id = 0; id < graph->edges.size(); id++)
    {
        auto edge = graph->edges[id];
        for(int jd = 0; jd < edge->size(); jd++)
        {
            Vector3 normal, u_fric, v_fric;
            edge->get_norm_fric_for_block(edge->partIDB, jd, normal, u_fric, v_fric);

            data->partIJs.push_back({edge->partIDA, edge->partIDB});
            data->contact_points.push_back(edge->points[jd]);

            data->compression_forces.push_back(normal * force_value[4 * contact_index]);
            data->tension_forces.push_back(-normal * force_value[4 * contact_index + 1]);
            data->friction_forces.push_back(u_fric * force_value[4 * contact_index + 2] + v_fric * force_value[4 * contact_index + 3]);

            contact_index ++;
        }
    }
}

bool EquilibriumFrictionSolver_Mosek::isEquilibrium(EquilibriumFrictionSolver::Vector3 gravity,
                                                    shared_ptr<StaticData> &data){

    //equilibrium matrix
    vector<EigenTriple> eq_tripleList;
    int n_eq_row, n_eq_col;
    computeEquilibriumMatrix(eq_tripleList, n_eq_row, n_eq_col);
    mosek::fusion::Matrix::t eq_mat;
    toMosekMatrix(n_eq_row, n_eq_col, eq_tripleList, eq_mat);

    //friction matrix
    vector<EigenTriple> fr_tripleList;
    int n_fr_row, n_fr_col;
    computeFrictionMatrix(fr_tripleList, n_fr_row, n_fr_col);
    mosek::fusion::Matrix::t fr_mat;
    toMosekMatrix(n_fr_row, n_fr_col, fr_tripleList, fr_mat);

    //gravity vector
    vector<double> external_force_vec;
    computeGravityForce(gravity, external_force_vec);
    auto external_force_array= monty::new_array_ptr(external_force_vec);

    mosek::fusion::Model::t M = new mosek::fusion::Model("rbe_quadratic");
    auto _M = monty::finally([&]() { M->dispose(); });

    int num_var = n_eq_col;
    mosek::fusion::Variable::t var_f  = M->variable("f", num_var);

    // equilibrium constraints
    M->constraint("equilibrium",
                  mosek::fusion::Expr::add(
                          mosek::fusion::Expr::mul(eq_mat, var_f),
                          external_force_array),
                       mosek::fusion::Domain::equalsTo(0.0)
                  );

    // friction constraints
    M->constraint("friction", mosek::fusion::Expr::mul(fr_mat, var_f), mosek::fusion::Domain::lessThan(0.0));

    //pick tension/compression force
    vector<int> compression_indices;
    vector<int> tension_indices;
    for(int id = 0; id < num_var / 4; id++){
        compression_indices.push_back(id * 4);
        tension_indices.push_back(id * 4 + 1);
    }
    auto compression_f_var = var_f->pick(monty::new_array_ptr(compression_indices));
    auto tension_f_var = var_f->pick(monty::new_array_ptr(tension_indices));

    // non-negative constraints
    M->constraint(compression_f_var, mosek::fusion::Domain::greaterThan(0.0));
    M->constraint(tension_f_var, mosek::fusion::Domain::greaterThan(0.0));

    //objective
    mosek::fusion::Variable::t quad_obj = M->variable("quad_obj", 1);
    auto stack_var = mosek::fusion::Expr::vstack(1, quad_obj, tension_f_var);
    M->constraint("obj", stack_var, mosek::fusion::Domain::inRotatedQCone());
    M->objective("obj", mosek::fusion::ObjectiveSense::Minimize, quad_obj);
    M->setSolverParam("intpntTolRelGap", getVarList()->getFloat("intpntTolRelGap"));

    M->solve();
    if(M->getProblemStatus() != mosek::fusion::ProblemStatus::PrimalAndDualFeasible){
        data.reset();
        return false;
    }

    vector<double> force_value;
    for(int id = 0; id < var_f->level()->size(); id++){
        force_value.push_back((*var_f->level())[id]);
    }
    unpackStaticData(force_value, data);
    data->energy = M->primalObjValue();
    if(data->energy < getVarList()->getFloat("equilibrium_infeasibility_tol")){
        return true;
    }
    else{
        return false;
    }
}
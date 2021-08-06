//
// Created by ziqwang on 14.08.20.
//

#ifndef TOPOLITE_EQUILIBRIUMFRICTIONLESSSOLVER_CONICMOSEK_H
#define TOPOLITE_EQUILIBRIUMFRICTIONLESSSOLVER_CONICMOSEK_H

#include "mosek.h"
#include "EquilibriumFrictionLessSolver.h"

template<int dim>
class EquilibriumFrictionLessSolver_ConicMosek: public EquilibriumFrictionLessSolver<dim> {
public:
    typedef typename EquilibriumFrictionLessSolver<dim>::Vector3 Vector3;
    typedef typename EquilibriumFrictionLessSolver<dim>::StaticData StaticData;
    typedef typename EquilibriumFrictionLessSolver<dim>::KinematicData KinematicData;

public:

    using EquilibriumFrictionLessSolver<dim>::computeEquilibriumMatrix;
    using EquilibriumFrictionLessSolver<dim>::getForces;
    using EquilibriumFrictionLessSolver<dim>::getEquilibriumData;
    using EquilibriumFrictionLessSolver<dim>::getKinetamticData;
    using EquilibriumFrictionLessSolver<dim>::silence;

public:
    typedef shared_ptr<ContactGraphBase<double>> pContactGraph;

public:
    EquilibriumFrictionLessSolver_ConicMosek(pContactGraph _graph) :
            EquilibriumFrictionLessSolver<dim>::EquilibriumFrictionLessSolver(_graph) {}

public:

    bool isEquilibrium(Vector3 gravity, shared_ptr<StaticData> &data) override;

    bool isEquilibrium_Constraints(Vector3 gravity, shared_ptr<StaticData> &data);

    bool isEquilibrium_Dual(Vector3 gravity, shared_ptr<KinematicData>  &data);

    bool isInterlocking(Eigen::Matrix<double, Eigen::Dynamic, 1> &force, shared_ptr<KinematicData>  &data);

    void static MSKAPI printstr(void *handle, const char str[]) { printf("%s", str);} /* printstr */

    void unpackStaticData(const vector<double> &forces_norm, shared_ptr<StaticData> &data);

    void unpackKinematicData(const vector<double> &velocities, shared_ptr<KinematicData> &data);
};

#endif
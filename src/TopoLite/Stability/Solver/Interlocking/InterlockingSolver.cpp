//
// Created by ziqwang on 2019-12-10.
//

#include "InterlockingSolver.h"



/*************************************************
*
*           Compute Interlocking Matrix
*
*************************************************/

template<typename Scalar>
void InterlockingSolver<Scalar>::computeTranslationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int rowID = 0;
    tri.clear();
    for(shared_ptr<ContactGraphEdgeBase<Scalar>> edge: graph->edges)
    {

        int iA = graph->nodes[edge->partIDA]->dynamicID;

        int iB = graph->nodes[edge->partIDB]->dynamicID;

        for (size_t id = 0; id < edge->size(); id++)
        {
            Vector3 nrm = (edge->normals[id]).template cast<double>();
            if (iA != -1) {
                tri.push_back(EigenTriple(rowID, 3 * iA, -nrm[0]));
                tri.push_back(EigenTriple(rowID, 3 * iA + 1, -nrm[1]));
                tri.push_back(EigenTriple(rowID, 3 * iA + 2, -nrm[2]));
            }

            if (iB != -1) {
                tri.push_back(EigenTriple(rowID, 3 * iB, nrm[0]));
                tri.push_back(EigenTriple(rowID, 3 * iB + 1, nrm[1]));
                tri.push_back(EigenTriple(rowID, 3 * iB + 2, nrm[2]));
            }
            rowID++;
        }
    }
    size = Eigen::Vector2i(rowID, 3 * graph->dynamic_nodes.size());
}

template<typename Scalar>
void InterlockingSolver<Scalar>::computeRotationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int rowID = 0;
    tri.clear();
    for(shared_ptr<ContactGraphEdgeBase<Scalar>> edge: graph->edges)
    {
        shared_ptr<ContactGraphNode<Scalar>> nodeA = graph->nodes[edge->partIDA];
        shared_ptr<ContactGraphNode<Scalar>> nodeB = graph->nodes[edge->partIDB];

        int iA = nodeA->dynamicID;
        int iB = nodeB->dynamicID;

        Vector3 ctA = (nodeA->centroid).template cast<double>();
        Vector3 ctB = (nodeB->centroid).template cast<double>();

        for(size_t id = 0; id < edge->points.size(); id++)
        {
            Vector3 nrm = (edge->normals[id]).template cast<double>();
            Vector3 pt = (edge->points[id]).template cast<double>();

            if (iA != -1)
            {
                Vector3 mt = (pt - ctA).cross(nrm);


                tri.push_back(EigenTriple(rowID, 6 * iA, -nrm[0]));
                tri.push_back(EigenTriple(rowID, 6 * iA + 1, -nrm[1]));
                tri.push_back(EigenTriple(rowID, 6 * iA + 2, -nrm[2]));
                tri.push_back(EigenTriple(rowID, 6 * iA + 3, -mt[0]));
                tri.push_back(EigenTriple(rowID, 6 * iA + 4, -mt[1]));
                tri.push_back(EigenTriple(rowID, 6 * iA + 5, -mt[2]));
            }

            if (iB != -1) {
                Vector3 mt = (pt - ctB).cross(nrm);
                tri.push_back(EigenTriple(rowID, 6 * iB, nrm[0]));
                tri.push_back(EigenTriple(rowID, 6 * iB + 1, nrm[1]));
                tri.push_back(EigenTriple(rowID, 6 * iB + 2, nrm[2]));
                tri.push_back(EigenTriple(rowID, 6 * iB + 3, mt[0]));
                tri.push_back(EigenTriple(rowID, 6 * iB + 4, mt[1]));
                tri.push_back(EigenTriple(rowID, 6 * iB + 5, mt[2]));
            }
            rowID++;
        }
    }
    size = Eigen::Vector2i(rowID, 6 * graph->dynamic_nodes.size());
}

template<typename Scalar>
void InterlockingSolver<Scalar>::computeTranslationalInterlockingMatrixDense(Eigen::MatrixXd &mat){
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    computeTranslationalInterlockingMatrix(tris, size);

    mat = Eigen::MatrixXd::Zero(size[0], size[1]);

    for(EigenTriple tri: tris){
        mat(tri.row(), tri.col()) = tri.value();
    }

    return;
}

template<typename Scalar>
void InterlockingSolver<Scalar>::computeRotationalInterlockingMatrixDense(Eigen::MatrixXd &mat)
{
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    computeRotationalInterlockingMatrix(tris, size);

    mat = Eigen::MatrixXd::Zero(size[0], size[1]);

    for(EigenTriple tri: tris){
        mat(tri.row(), tri.col()) = tri.value();
    }

    return;
}


template<typename Scalar>
void InterlockingSolver<Scalar>::computeRotationalInterlockingMatrixSparse(EigenSpMat &spatMat)
{
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    computeRotationalInterlockingMatrix(tris, size);
    spatMat = EigenSpMat(size[0], size[1]);
    spatMat.setFromTriplets(tris.begin(), tris.end());
}


template<typename Scalar>
void InterlockingSolver<Scalar>::appendAuxiliaryVariables(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int num_row = size[0];
    int num_col = size[1];

    for(size_t id = 0; id < num_row; id++){
        tri.push_back(EigenTriple(id, num_col + id, -1));
    }
    size[1] += num_row;
    return;
}

template<typename Scalar>
void InterlockingSolver<Scalar>::appendMergeConstraints(vector<EigenTriple> &tri, Eigen::Vector2i &size, bool isRotation)
{
    int dimension = (isRotation ? 6 : 3);
    for(int id = 0;id < graph->merged_nodes.size(); id++)
    {
        wpContactGraphNode A = graph->merged_nodes[id].first;
        wpContactGraphNode B = graph->merged_nodes[id].second;
        int iA = A.lock()->dynamicID;
        int iB = B.lock()->dynamicID;
        if(iA != -1 && iB != -1)
        {
            for(int index = 0; index < dimension; index++)
            {
                tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index, dimension * iA + index, 1));
                tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index, dimension * iB + index, -1));

                tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index + 1, dimension * iA + index, -1));
                tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index + 1, dimension * iB + index, 1));
            }
        }
        else{
            for(int index = 0; index < dimension; index++){
                if(iA != -1) {
                    tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index, dimension * iA + index, 1));
                    tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index + 1, dimension * iA + index, -1));
                }
                if(iB != -1){
                    tri.push_back(EigenTriple(size[0] + id * dimension + index, dimension * iB + index, 1));
                    tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index + 1, dimension * iB + index, -1));

                }
            }
        }
    }
    size[0] += graph->merged_nodes.size() * dimension * 2;
    return;
}
// No need to call this TemporaryFunction() function,
// it's just to avoid link error.
void TemporaryFunction_InterlockingSolver ()
{
    InterlockingSolver<double> solver(nullptr);
    vector<InterlockingSolver<double>::EigenTriple> tri;
    Eigen::Vector2i size;
    bool isRotation = true;
    solver.appendMergeConstraints(tri, size, isRotation);
    solver.appendAuxiliaryVariables(tri, size);
    solver.computeRotationalInterlockingMatrix(tri, size);
    solver.computeTranslationalInterlockingMatrix(tri, size);
}
template class InterlockingSolver<double>;
template class InterlockingSolver<float>;

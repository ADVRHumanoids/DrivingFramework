#ifndef DYNAMIC_MODELS_QR_DECOMPOSITION_H
#define DYNAMIC_MODELS_QR_DECOMPOSITION_H

#include <rbdl/rbdl.h>
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/dynamic_models/dynamic_models.h"
#include "mwoibn/dynamic_models/basic_model.h"

namespace mwoibn
{

namespace dynamic_models
{

/** @brief This class provides QR decomposition for a robot
**/
class QrDecomposition : public BasicModel
{

public:
QrDecomposition(mwoibn::robot_class::Robot& robot) : BasicModel(robot)
{
        _qr_ptr.reset(new Eigen::ColPivHouseholderQR<Eigen::MatrixXd>(
                              _robot.contacts().jacobianCols(), _robot.contacts().jacobianRows()));

        _resize(std::min(_robot.contacts().jacobianCols(),
                         _robot.contacts().jacobianRows()));
        _contacts_transpose = _robot.contacts().getJacobian().transpose();
        _qr_ptr->compute(_contacts_transpose);
}
virtual ~QrDecomposition() {
}

virtual const mwoibn::VectorLimited& getGravityUnconstrained()
{

        _qr_gravity.noalias() =  _independent * BasicModel::getGravity();

        return _qr_gravity;
}
/** @brief returns all modeled nonlinear effects including gravity in robots
 * dynamics, computed for a non-constrained directions **/
virtual const mwoibn::VectorLimited& getNonlinearEffectsUnconstrained()
{
        _qr_non_linear.noalias() =  _independent * BasicModel::getNonlinearEffects();

        return _qr_non_linear;
}

/** @brief returns inertia, computed for a non-constrained directions **/
virtual const mwoibn::MatrixLimited& getInertiaUnconstrained()
{
        _qr_inertia.noalias() = _independent * BasicModel::getInertia();
        return _qr_inertia;
}

const mwoibn::Matrix& getQMatrix(){
        _q = _qr_ptr->matrixQ();
        return _q;
}

mwoibn::Matrix getRMatrix() const
{
        return _qr_ptr->matrixQR().triangularView<Eigen::Upper>();
}

/** @brief returns rank of a jacobian matrix, reveals number of unconstrained
 * dofs  **/
unsigned int getRank() const {
        return _qr_ptr->rank();
}

void updateDecomposition()
{


        _contacts_transpose.noalias() = _robot.contacts().getJacobian().transpose();

        _qr_ptr->compute(_contacts_transpose);

        getQMatrix();

        if (getRank() != _rank)
                resize();  // NRT
        else
                _changed = false;

        _q_cut.noalias() = _q.rightCols(_robot.getDofs() - _rank);
        _independent.noalias() = _q_cut.transpose();

}

void resize()
{
        _resize(_qr_ptr->rank());
}

bool changed(){
        return _changed;
}

virtual void update() {
        updateDecomposition();
}

/** @brief performs transformation of a user defined vector/matrix to a
   *non-constrainted system
 *
 * @warning this method doesn't check if the size of a state send to the
 **matrix is correct, it has to be ensured by a user
 *
 */
template <typename Type> Type decompose(Type state) const
{
        return _independent * state;
}

/** @brief retrunes transformation matrix to get the non-constrained state */
const mwoibn::MatrixLimited& getTransformationMatrix() const {
        return _independent;
}

protected:
//  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>_permutation;
std::unique_ptr<Eigen::ColPivHouseholderQR<Eigen::MatrixXd> > _qr_ptr;
mwoibn::MatrixLimited _independent, _qr_inertia;   //< matrix to extract independent rows
int _rank;
bool _changed;
mwoibn::Matrix _contacts_transpose, _q_cut, _q;
mwoibn::VectorLimited _qr_gravity, _qr_non_linear;
void _resize(int rank)
{
        _rank = rank;
        _independent.setZero(_robot.getDofs()-_rank, _robot.getDofs());
        _qr_gravity.setZero(_robot.getDofs()-_rank);
        _qr_non_linear.setZero(_robot.getDofs()-_rank);
        _qr_inertia.setZero(_robot.getDofs()-rank, _robot.getDofs());
        _q.setZero(_robot.getDofs(), _robot.getDofs());
        _q_cut.setZero(_robot.getDofs()-_rank, _robot.getDofs());

        _changed = true;
//    _qr_non_linear.setZero(_robot.getDofs()-_rank);
//    _qr_inertia.setZero(_robot.getDofs()-rank, _robot.getDofs());
}
};
} // namespace package
} // namespace library

#endif // DYANMIC_MODELS_QR_DECOMPOSITION_H

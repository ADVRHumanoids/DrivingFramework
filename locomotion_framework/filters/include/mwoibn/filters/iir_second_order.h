#ifndef __MWOIBN_FILTERS_IIR_SECOND_ORDER_H
#define __MWOIBN_FILTERS_IIR_SECOND_ORDER_H

#include "mwoibn/common/types.h"

namespace mwoibn
{

namespace filters
{

class IirSecondOrder
{
public:
IirSecondOrder(int size, double omega, double damp);

template <typename Vector1> void init(double dt, const Vector1& u)
{

        computeCoeffs(dt);
        reset(u);
}


void computeCoeffs(double dt);

void resize(int size);

template <typename Vector1> void reset(const Vector1& u)
{
        for(int i = 0; i < _u.cols(); i++)
                _u.row(0)[i] = u[i];
        _u.row(1) = _u.row(0);
        _u.row(2) = _u.row(1);

        _y.noalias() = _u;
}

template <typename Vector1, typename Vector2>
void update(const Vector1& u, Vector2& y)
{

        _u.row(2) = _u.row(1);
        _u.row(1) = _u.row(0);

        for(int i = 0; i < _u.cols(); i++)
                _u.row(0)[i] = u[i];

        _y.row(2) = _y.row(1);
        _y.row(1) = _y.row(0);

        _y.row(0).noalias() = _b * _u;
        _y.row(0).noalias() -= _a.tail<2>() * _y.bottomRows<2>();
        _y.row(0).noalias() = _y.row(0) / _a[0];

        for(int i = 0; i < _y.cols(); i++)
                y[i] = _y.row(0)[i];
}

template <typename Vector1> void update(Vector1& u) {
        update(u, u);
}

void setStep() {
}
virtual ~IirSecondOrder() {
}

protected:
mwoibn::Matrix _y, _u;
mwoibn::VectorNT _a, _b;
double _omega, _damp;
};
}
}
#endif

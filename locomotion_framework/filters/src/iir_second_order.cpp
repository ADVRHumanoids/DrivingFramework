#include "mwoibn/filters/iir_second_order.h"

mwoibn::filters::IirSecondOrder::IirSecondOrder(int size, double omega,
                                                double damp)
    : _damp(damp), _omega(omega)
{

  resize(size);

  _a.setZero(3);
  _b.setZero(3);

  _b << 1, 2, 1;
}

void mwoibn::filters::IirSecondOrder::resize(int size)
{
  _u.setZero(3, size);
  _y.setZero(3, size);
}

void mwoibn::filters::IirSecondOrder::computeCoeffs(double dt)
{
  //std::cout << dt << "\t" << _omega << "\t" << _damp << std::endl;
  double t_omega = 1 / _omega / dt;

  _a[0] = 1 + 4 * _damp * t_omega + 4 * t_omega * t_omega;
  _a[1] = 2 - 8 * t_omega;
  _a[2] = 1 - 4 * _damp * t_omega + 4 * t_omega * t_omega;
}

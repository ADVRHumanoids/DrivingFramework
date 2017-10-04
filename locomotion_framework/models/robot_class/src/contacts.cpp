#include "mwoibn/robot_class/contacts.h"

namespace mwoibn
{

namespace robot_class
{

bool mwoibn::robot_class::Contacts::remove(int i)
{
  if (i < 0 || i >= _contacts.size())
    return false;

  _contacts.erase(_contacts.begin() + i);
  resize();
  return true;
}

int mwoibn::robot_class::Contacts::getId(std::string name) const
{
  auto foundItem = std::find_if(_contacts.begin(), _contacts.end(),
                                [&name](std::unique_ptr<ContactV2> const& contact)
                                {
                                  return contact->getPointName(0) == name;
                                });

  if (foundItem == _contacts.end())
    return -1;
  else
    return std::distance(_contacts.begin(), foundItem);
}

mwoibn::robot_class::ContactV2&
mwoibn::robot_class::Contacts::contact(unsigned int id)
{
  if (id < _contacts.size())
    return *_contacts.at(id);
  else
    throw std::out_of_range("Given ID is beyond a vector scope");
}

std::vector<unsigned int>
mwoibn::robot_class::Contacts::getActive(std::vector<std::string>* names) const
{

  std::vector<unsigned int> ids;
  for (int i = 0; i < _contacts.size(); i++)
  {
    if (_contacts[i]->isActive())
      ids.push_back(i);
  }
  if (names)
  {
    for (auto& id : ids)
      names->push_back(_contacts[id]->getPointName(0));
  }

  return ids;
}

std::vector<unsigned int> mwoibn::robot_class::Contacts::getInactive(
    std::vector<std::string>* names) const
{

  std::vector<unsigned int> ids;
  for (int i = 0; i < _contacts.size(); i++)
  {
    if (!_contacts[i]->isActive())
      ids.push_back(i);
  }
  if (names)
  {
    for (auto& id : ids)
      names->push_back(_contacts[id]->getPointName(0));
  }

  return ids;
}

unsigned int mwoibn::robot_class::Contacts::sizeActive() const
{
  int i = 0;
  for (auto& contact : _contacts)
    if (contact->isActive())
      ++i;

  return i;
}

unsigned int mwoibn::robot_class::Contacts::sizeInactive() const
{
  int i = 0;
  for (auto& contact : _contacts)
    if (!contact->isActive())
      ++i;

  return i;
}

const mwoibn::Matrix& mwoibn::robot_class::Contacts::getJacobian()
{
  _jacobian.setZero();

  unsigned int i = 0;

  for (auto& contact : _contacts)
  {
    _jacobian.block(i, 0, contact->jacobianSize(), _dofs) = contact->getPointJacobian();
    i += contact->jacobianSize();
  }

  return _jacobian;
}

mwoibn::Matrix
mwoibn::robot_class::Contacts::getMinimumJacobian()
{

  _jacobian.setZero();
  unsigned int i = 0;
  int j = 0;
  for (auto& contact : _contacts)
  {
    if (contact->isActive())
    {
      _jacobian.block(i, 0, contact->jacobianSize(), _dofs) = contact->getPointJacobian();
      i += contact->jacobianSize();
      j++;
    }
  }

  return _jacobian.topRows(i);
}

std::vector<mwoibn::Matrix>
mwoibn::robot_class::Contacts::getJacobians()
{

  std::vector<mwoibn::Matrix> jacobians;
  for (auto& contact : _contacts)
  {
    jacobians.push_back(contact->getPointJacobian());
  }

  return jacobians;
}

std::vector<mwoibn::Matrix>
mwoibn::robot_class::Contacts::getMinimumJacobians()
{

  std::vector<mwoibn::Matrix> jacobians;
  for (auto& contact : _contacts)
  {
    if (contact->isActive())
      jacobians.push_back(contact->getPointJacobian());
  }

  return jacobians;
}

const mwoibn::VectorN& mwoibn::robot_class::Contacts::getPosition()
{

  _positions.setZero(_contacts.size() * 7);

  unsigned int i = 0;

  for (auto& contact : _contacts)
  {
    _positions.segment<7>(i) = contact->getPosition();
    i += 7;
  }

  return _positions;
}

std::vector<mwoibn::VectorN>
mwoibn::robot_class::Contacts::getPositions()
{
  std::vector<mwoibn::VectorN> positions;

  for (auto& contact : _contacts)
  {
    positions.push_back(contact->getPosition());
  }
  return positions;
}

mwoibn::VectorBool
mwoibn::robot_class::Contacts::getTypes(std::vector<CONTACT_TYPE> types)
{

  mwoibn::VectorBool state(size());

  for (int i = 0; i < size(); i++)
  {
    if (std::find(types.begin(), types.end(), _contacts.at(i)->type()) !=
        types.end())
      state[i] = true;
    else
      state[i] = false;
  }
  return state;
}

const mwoibn::VectorBool& mwoibn::robot_class::Contacts::getDofs()
{

  _boolean_checks.setConstant(false);
  for (auto& contact : _contacts)
  {
    for (auto& dof : contact->getChain())
      _boolean_checks[dof] = true;
  }

  return _boolean_checks;
}

const mwoibn::VectorBool& mwoibn::robot_class::Contacts::getActiveDofs()
{
  _boolean_checks.setConstant(false);

  int i = 0;
  for (auto& contact : _contacts)
  {
    if (!contact->isActive())
      continue;

    for (auto& dof : contact->getChain())
      _boolean_checks[dof] = true;
  }

  return _boolean_checks;
}

const mwoibn::VectorBool& mwoibn::robot_class::Contacts::getInactiveDofs()
{
  _boolean_checks.noalias() = mwoibn::eigen_utils::flip(getActiveDofs());
  return _boolean_checks;
}

} // namespace package
} // namespace library

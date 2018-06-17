#include "mgnss/communication/events_handler.h"

events::EventHandler::EventHandler(
        mwoibn::hierarchical_control::tasks::CartesianSelective& tasks,
        mwoibn::hierarchical_control::tasks::OrientationSelective& orientation,

        mwoibn::hierarchical_control::tasks::Constraints& constraints,
        mwoibn::hierarchical_control::tasks::CenterOfMass& com,
        mwoibn::point_handling::PositionsHandler& phs,
        mwoibn::hierarchical_control::controllers::Basic& controller,
        mwoibn::robot_class::Robot& robot)
        : _tasks(tasks), _phs(phs), _controller(controller), _robot(robot),
        _constraints(constraints), _com(com), _orientation(orientation)
{
        for (int i = 0; i < robot.contacts().size(); i++)
        {
                _chains.push_back(Chain(robot.contacts().contact(i)));
        }

        reset();
}

bool events::EventHandler::claim(int i)
{
        if (!_chains[i].setAction(ACTION::CLAIM))
                return false;
        if (!_chains[i].setReferenceState(STATE::SWING))
                return false;

        reset();
        return true;
}

// bool events::EventHandler::circle(int i){

//}

bool events::EventHandler::circle(int i)
{
        // create com reference
        _robot.centerOfMass().update();

        Eigen::VectorXd original(3);
        original << _robot.centerOfMass().get().head(2), 0;

        Eigen::Vector3d one, two, three;
        double radious = i / 100.0;
        one << -radious, 0, 0;
        two << radious, 0, 0;
        three << radious, radious, 0;

        _line_ptr.reset(new mwoibn::reference_generation::Line(
                                _robot.centerOfMass().get().head(2), 0.0005,
                                original.head(2) + one.head(2)));

        _circle_ptr.reset(new mwoibn::reference_generation::Local_Circle(
                                  original + one, original + two, original + three, 0.0005));

        setAction(ACTION::CIRCLE);
}

void events::EventHandler::_circle_1()
{
        if (_line_ptr->isDone())
                advance();
}

void events::EventHandler::_claim_2(int i)
{

        if (!_chains[i].setAction(ACTION::NONE))
                return;
        if (!_chains[i].setState(STATE::SWING))
                return;
        mwoibn::Vector3 point =
                _tasks.points().getPointStateWorld(i);

        point[2] += 0.005;
        //      reference.segment(i * 3, 3) = point;
        _tasks.setReference(i, point);
//  _tasks.updateSelection(i * 3, 1);
//  _tasks.updateSelection(i * 3 + 1, 1);
        _tasks.updateSelection(i * 3 + 2, 1);
        _constraints.claimContact(i);
        _com.claimContact(i);
        stop();

        std::cout << "claimed resource: " << i << std::endl;
}

bool events::EventHandler::release(int i)
{

        bool run = _chains[i].setAction(ACTION::RELEASE);
        run = run && _chains[i].setState(STATE::STATIC);
        run = run && _chains[i].setReferenceState(STATE::STATIC);

        if (!run)
                return false;
//  _tasks.updateSelection(i * 3, 0);
//  _tasks.updateSelection(i * 3 + 1, 0);
        _tasks.updateSelection(i * 3 + 2, 0);
        _constraints.releaseContact(i);
        _com.releaseContact(i);

        std::cout << "released " << i << std::endl;

        return _chains[i].setAction(ACTION::NONE);
}

bool events::EventHandler::put(int i)
{
        if (!_chains[i].setAction(ACTION::PUT))
                return false;
        mwoibn::Vector3 point =
                _tasks.points().getPointStateWorld(i);
        point.head(2) = _chains[i].getReference().head(2);
        _tasks.setReference(i, point);


        Eigen::Vector3d axis;
        axis << 0, 1, 0;
        mwoibn::Quaternion desired_state =
                _orientation.getReference(i)*mwoibn::Quaternion::fromAxisAngle(
                        axis, -0.20);


        _orientation.setReference(i, desired_state);


        return true;
}

bool events::EventHandler::raise(int i)
{
        if (!_chains[i].setAction(ACTION::RAISE))
                return false;
        mwoibn::Vector3 point = _tasks.getReference(i);

        point[2] = 0.15+0.075;
        _tasks.setReference(i, point);

        Eigen::Vector3d axis;
        axis << 0, 1, 0;

        mwoibn::Quaternion desired_state =
                _orientation.getReference(i) * mwoibn::Quaternion::fromAxisAngle(
                        axis, 0.20);

        _orientation.setReference(i, desired_state);

        return true;
}

bool events::EventHandler::reset()
{
        _support_polygon.p.clear();
        for (auto chain : _chains)
        {
                if (chain.getReferenceState() == STATE::STATIC)
                        _support_polygon.p.push_back(
                                mwoibn::reference_generation::Utils::toFloat3(
                                        chain.getContactPosition()));
        }

        mwoibn::Vector3 msp_point =
                mwoibn::reference_generation::Utils::fromFloat3(
                        _support_polygon.CenterPoint());
        _line_ptr.reset(new mwoibn::reference_generation::Line(
                                _robot.centerOfMass().get().head(2), 0.0005, msp_point.head(2)));
        setAction(ACTION::LINE);
        return true;
}

bool events::EventHandler::stop()
{
        _line_ptr.reset(new mwoibn::reference_generation::Line(
                                _robot.centerOfMass().get().head(2), 0.0005,
                                _robot.centerOfMass().get().head(2)));
        setAction(ACTION::LINE);
        return true;
}

void events::EventHandler::update()
{
        int i = 0;
        for (auto& chain : _chains)
        {
                switch (chain.getAction())
                {
                case ACTION::CLAIM :
                        if (chain.getMode() == 0)
                                _claim_1(i);
                        if (chain.getMode() == 1)
                                _claim_2(i);

                        break;
                case ACTION::PUT:
                        if (chain.getMode() == 0)
                                _put_1(i);
                        if (chain.getMode() == 1)
                                _put_2(i);
                        if (chain.getMode() == 2)
                                _put_3(i);
                        break;
                case ACTION::RAISE:
                        if (chain.getMode() == 0)
                                _raise_1(i);
                        break;
                }
                ++i;
        }

        stepReference();
}

mwoibn::VectorBool events::EventHandler::getActiveContacts()
{
        mwoibn::VectorBool contacts(_chains.size());
        for (int i = 0; i < _chains.size(); i++)
        {
                contacts[i] = (_chains[i].getState() == STATE::STATIC);
        }
        return contacts;
}

std::vector<int> events::EventHandler::getReferenceContacts()
{
        std::vector<int> contacts;
        for (int i = 0; i < _chains.size(); i++)
        {
                if (_chains[i].getReferenceState() == STATE::STATIC)
                        contacts.push_back(i);
        }
        return contacts;
}

void events::EventHandler::_raise_1(int i)
{
        mwoibn::Vector3 error = _tasks.getError().segment(i * 3, 3);
        mwoibn::Vector3 error_2 =
                _orientation.getError().segment(i * 3, 3);

        if (error.norm() < eps && error_2.norm() < eps)
        {
                _chains[i].setAction(ACTION::NONE);
                std::cout << "Leg raised" << std::endl;
        }
        else{
                std::cout << "position\t" << error.norm() << ",\t orientation" << error_2.norm() << std::endl;
                std::cout << "error\n" << error_2 << std::endl;
        }
}

void events::EventHandler::_put_1(int i)
{
        mwoibn::Vector3 error = _tasks.getError().segment(i * 3, 3);
        mwoibn::Vector3 error_2 =
                _orientation.getError().segment(i * 3, 3);

        if (error.norm() < 3 * eps && error_2.norm() < eps)
        {
                _chains[i].advance();
                std::cout << "first step done" << std::endl;
        }
        else{
                std::cout << "position\t" << error.norm() << ",\t orientation" << error_2.norm() << std::endl;
                std::cout << "error\n" << error_2 << std::endl;
        }
}

void events::EventHandler::_put_2(int i)
{
        if (!_chains[i].setAction(ACTION::NONE))
                return;

        mwoibn::Vector3 point = _tasks.getReference(i);
        point[2] = -0.0001+0.075;
        _tasks.setReference(i, point);

        _chains[i].advance();
}

void events::EventHandler::_put_3(int i)
{
        mwoibn::Vector3 error = _tasks.getError().segment(i * 3, 3);
        if (error.norm() < eps)
        {
                _chains[i].setAction(ACTION::NONE);
                std::cout << "Leg " << i << " is on the ground" << std::endl;
        }
        else
                std::cout << error.norm() << std::endl;
}

void events::EventHandler::_claim_1(int i)
{

        Polygon com;

        float3 point = mwoibn::reference_generation::Utils::toFloat3(
                _robot.centerOfMass().get());
        point[2] = 0;

        com.p.push_back(point + float3(_safety_boundry, _safety_boundry, 0));
        com.p.push_back(point + float3(_safety_boundry, -_safety_boundry, 0));
        com.p.push_back(point + float3(-_safety_boundry, -_safety_boundry, 0));
        com.p.push_back(point + float3(-_safety_boundry, _safety_boundry, 0));

        if (_support_polygon.Contains(com))
                _chains[i].advance();
}

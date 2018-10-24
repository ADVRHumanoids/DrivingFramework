#include <config.h>

#include <mwoibn/loaders/robot.h>

#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <mwoibn/robot_class/robot_xbot_nrt.h>
#include <mwoibn/visualization_tools/rviz_track_point.h>
#include <mwoibn/point_handling/robot_points_handler.h>

// std::vector<mwoibn::Vector3>
// contactPoints(mwoibn::robot_class::Robot& robot,
//              mwoibn::point_handling::PositionsHandler& wheels)
//{
//  mwoibn::Vector3 ground, wheel, x, point;
//  std::vector<mwoibn::Vector3> points;
//  ground << 0, 0, 1; // flat ground
//  double R = 0.010, r = 0.068;

//  for (int i = 0; i < wheels.size(); i++)
//  {

//    points.push_back(point);
//  }

//  // std::cout << std::endl;
//  return points;
//}

class Flow
{
public:
Flow(mwoibn::robot_class::Robot& robot,
     mwoibn::point_handling::PositionsHandler& wheels)
        : _robot(robot), _wheels(wheels), _contacts("ROOT", _robot)
{
        _ground << 0, 0, 1;

        for (auto link : _robot.getLinks("wheels"))
        {
                _contacts.addPoint(link);
                _current.push_back(mwoibn::Vector3::Zero());
                _spherical.push_back(mwoibn::Vector3::Zero());

                _modelDiff.push_back(mwoibn::Vector3::Zero());
                _equationDiff.push_back(mwoibn::Vector3::Zero());
                _flowDiff.push_back(mwoibn::Vector3::Zero());
        }

        _contacts.computeChain();

        _contactPoints();

        // init estimation
        _estimated = _current;
        _fromPosDiff = _current;

        _contacts.setFullStatesWorld(
                _current, robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));
}
virtual ~Flow() {
}

void update()
{

        std::vector<mwoibn::Vector3> _previous = _current;
        _contactPoints(); // update contact position

        _contacts.setFullStatesWorld(
                _current,
                _robot.state.get(
                        mwoibn::robot_class::INTERFACE::POSITION)); // update handler

        double test = ((_current[0] - _previous[0]).norm() +
                       (_current[1] - _previous[1]).norm() +
                       (_current[2] - _previous[2]).norm() +
                       (_current[3] - _previous[3]).norm());

        for (int i = 0; i < _wheels.size(); i++)
                _fromPosDiff[i] += (_current[i] - _previous[i]);
        _flowDerivatives(); // update flow derivative
        _flowEstimates();   // estimate current contact
        _sphericalPoints(); // compute spherical estimation
        _modelDerivatives(); // compute contact point derivative
        _equationDerivatives(); // compute from my equation

        nr++;
        if (test > mwoibn::EPS)
        {
                std::cout << "nr\t" << nr << "\n";
                std::cout << "from position"
                          << "\t";
                for (int i = 0; i < _wheels.size(); i++)
                        std::cout << (_current[i][0] - _previous[i][0]) / _robot.rate() / nr
                                  << "\t"
                                  << (_current[i][1] - _previous[i][1]) / _robot.rate() / nr
                                  << "\t"
                                  << (_current[i][2] - _previous[i][2]) / _robot.rate() / nr
                                  << "\t";
                std::cout << std::endl;
                std::cout << "estimated"
                          << "\t";
                for (int i = 0; i < _wheels.size(); i++)
                        std::cout << _estimated[i][0] << "\t" << _estimated[i][1] << "\t"
                                  << _estimated[i][2] << "\t";
                std::cout << std::endl;
                std::cout << "point\t"
                          << "\t";
                for (int i = 0; i < _wheels.size(); i++)
                        std::cout << _equationDiff[i][0] << "\t" << _equationDiff[i][1] << "\t"
                                  << _equationDiff[i][2] << "\t";
                std::cout << std::endl;
                std::cout << "from positions"
                          << "\t";
                for (int i = 0; i < _wheels.size(); i++)
                        std::cout << _current[i][0] << "\t" << _current[i][1] << "\t"
                                  << _current[i][2] << "\t";
                std::cout << std::endl;
                std::cout << "models\t"
                          << "\t";
                for (int i = 0; i < _wheels.size(); i++)
                        std::cout << _modelDiff[i][0] << "\t" << _modelDiff[i][1] << "\t"
                                  << _modelDiff[i][2] << "\t";
                std::cout << std::endl;
                nr = 0;
        }
}

mwoibn::Vector3& getContact(int i) {
        return _current[i];
}

mwoibn::Vector3& getEstimate(int i) {
        return _estimated[i];
}
mwoibn::Vector3& getSpherical(int i) {
        return _spherical[i];
}

mwoibn::Vector3& getModelDiff(int i) {
        return _modelDiff[i];
}
mwoibn::Vector3& getEquationDiff(int i) {
        return _equationDiff[i];
}

protected:
double R = 0.010, r = 0.068;
int nr = 0;

mwoibn::robot_class::Robot& _robot;
mwoibn::point_handling::PositionsHandler& _wheels;    // wheels centers
mwoibn::point_handling::PositionsHandler _contacts;   // contact points

std::vector<mwoibn::Vector3> _current, _estimated, _modelDiff, _equationDiff,
                             _fromPosDiff, _flowDiff, _spherical;
mwoibn::Vector3 _ground, _wheel_axis, _x;

mwoibn::Matrix3 _skew(mwoibn::Vector3 vec)
{

        mwoibn::Matrix3 skew;
        skew << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;

        // std::cout << skew << std::endl;
        return skew;
}

void _sphericalPoints()
{
        for (int i = 0; i < _wheels.size(); i++)
                _sphericalPoint(i);
}

void _sphericalPoint(int i)
{
        _spherical[i] = _robot.contacts().contact(i).getPosition().head(3);
}

void _contactPoints()
{
        for (int i = 0; i < _wheels.size(); i++)
                _contactPoint(i);
}

void _modelDerivatives()
{
        for (int i = 0; i < _wheels.size(); i++)
                _modelDerivative(i);
}

void _modelDerivative(int i)
{
        mwoibn::Matrix jacobian = _contacts.point(i).getPositionJacobian();

        _modelDiff[i] =
                jacobian *
                _robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY); // *
        //_robot.rate();
}

void _contactPoint(int i)
{
        _wheel_axis = _wheels.point(i).getRotationWorld().col(2); // z axis

        _x = _wheel_axis * _ground.transpose() * _wheel_axis;

        _x.normalize();

        std::cout << "norm: " << _x.norm() << std::endl;
        _current[i] = _x * R - _ground * (r + R);

        _current[i] += _wheels.getPointStateWorld(i);
}

void _equationDerivatives()
{
        for (int i = 0; i < _wheels.size(); i++)
                _equationDerivative(i);
}

void _equationDerivative(int i)
{

        _wheel_axis = _wheels.point(i).getRotationWorld().col(2); // z axis

        mwoibn::Matrix jacobian = _wheels.point(i).getOrientationJacobian();

        mwoibn::Matrix jacobian_pos = _wheels.point(i).getPositionJacobian();

        mwoibn::Matrix3 temp =
                -_skew(_current[i] - _wheels.getPointStateWorld(i));
//    temp = temp * R;

//    temp += _skew(_ground) * r;

        _equationDiff[i] =
                jacobian * _robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY);

        _equationDiff[i] = temp * _equationDiff[i];

        _equationDiff[i] +=
                jacobian_pos *
                _robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY);

        //_equationDiff[i] = _equationDiff[i] * _robot.rate();
}

void _flowEstimates()
{
        for (int i = 0; i < _wheels.size(); i++)
                _flowEstimate(i);
}

void _flowEstimate(int i) {
        _estimated[i] += _flowDiff[i] * _robot.rate();
}

void _flowDerivatives()
{
        for (int i = 0; i < _wheels.size(); i++)
                _flowDerivative(i);
}
void _flowDerivative(int i)
{
        _wheel_axis = _wheels.point(i).getRotationWorld().col(2); // z axis

        mwoibn::Matrix jacobian = _wheels.point(i).getOrientationJacobian();

        mwoibn::Matrix jacobian_pos = _wheels.point(i).getPositionJacobian();

        mwoibn::Matrix3 temp =
                -_skew(_wheel_axis * _ground.transpose() * _wheel_axis);

        temp -= _wheel_axis * _ground.transpose() * _skew(_wheel_axis);

        //    if (i == 0)
        //    {

        //      mwoibn::Matrix3 temp1 =
        //          -_skew(_wheel_axis * _ground.transpose() * _wheel_axis);
        //      mwoibn::Vector3 vec1 =
        //          temp1 * jacobian *
        //          _robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY);
        //      std::cout << "1\t" << vec1[0] << "\t" << vec1[1] << "\t" << vec1[2]
        //                << "\n";
        //      temp1 = -_wheel_axis * _ground.transpose() * _skew(_wheel_axis);
        //      vec1 = temp1 * jacobian *
        //             _robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY);
        //      std::cout << "2\t" << vec1[0] << "\t" << vec1[1] << "\t" << vec1[2]
        //                << "\n";
        //    }
        _flowDiff[i] =
                jacobian * _robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY);

        _flowDiff[i] = temp * _flowDiff[i];

        _flowDiff[i] = _flowDiff[i] * R;

        _flowDiff[i] += jacobian_pos *
                        _robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY);
}
};

int main(int argc, char** argv)
{

        static const mwoibn::visualization_tools::Utils::TYPE tracker_type =
                mwoibn::visualization_tools::Utils::TYPE::POINT;

        ros::init(argc, argv,
                  "contacts_test"); // initalize node needed for the service

        ros::NodeHandle n;
        ros::Publisher pub;

        std::string path = std::string(DRIVING_FRAMEWORK_WORKSPACE);
        mwoibn::loaders::Robot loader;
        mwoibn::robot_class::Robot& robot = loader.init(
                path + "DrivingFramework/locomotion_framework/configs/mwoibn_v2.yaml",
                "simulated");

        mwoibn::point_handling::PositionsHandler wheels_ph("ROOT", robot,
                                                           robot.getLinks("wheels"));

        mwoibn::visualization_tools::RvizTrackPoint track_current("rviz/current");
        mwoibn::visualization_tools::RvizTrackPoint track_new("rviz/new");
        mwoibn::visualization_tools::RvizTrackPoint track_estimated("rviz/estimated");

        Flow flow(robot, wheels_ph);

        double edge = 0.001;
        for (int i = 0; i < robot.contacts().size(); i++)
        {
                // RED: current equation
                track_current.initMarker(tracker_type, "world", edge, edge, edge);

                // GREEN: new equation
                track_new.initMarker(tracker_type, "world", edge, edge, edge);

                // BLUE: estimated new
                track_estimated.initMarker(tracker_type, "world", edge, edge, edge);
        }
        for (int i = 0; i < robot.contacts().size(); i++)
        {
                track_current.setColor(i, 1, 0, 0);
                track_new.setColor(i, 0, 1, 0);
                track_estimated.setColor(i, 1, 1, 0);
        }

        std::cout.precision(10);
        std::cout << std::fixed;
        //  std::cout
        //      << "1_new[x]\t1_new[y]\t1_new[z]\t1_simp[x]\t1_simp[y]\t1_simp[z]\t";
        //  std::cout
        //      << "1_mdt[x]\t1_mdt[y]\t1_mdt[z]\t1_edt[x]\t1_edt[y]\t1_edt[z]\t";
        //  std::cout
        //      << "2_new[x]\t2_new[y]\t2_new[z]\t2_simp[x]\t2_simp[y]\t2_simp[z]\t";
        //  std::cout
        //      << "2_mdt[x]\t2_mdt[y]\t2_mdt[z]\t2_edt[x]\t2_edt[y]\t2_edt[z]\t";
        //  std::cout
        //      << "3_new[x]\t3_new[y]\t3_new[z]\t3_simp[x]\t3_simp[y]\t3_simp[z]\t";
        //  std::cout
        //      << "3_mdt[x]\t3_mdt[y]\t3_mdt[z]\t3_edt[x]\t3_edt[y]\t3_edt[z]\t";
        //  std::cout
        //      << "4_new[x]\t4_new[y]\t4_new[z]\t4_simp[x]\t4_simp[y]\t4_simp[z]\t";
        //  std::cout
        //      << "4_mdt[x]\t4_mdt[y]\t4_mdt[z]\t4_edt[x]\t4_edt[y]\t4_edt[z]\n";

        int i = 0;
        while (ros::ok())
        {
                // std::vector<mwoibn::Vector3> new_contacts = contactPoints(robot,
                // wheels_ph);

                for (int i = 0; i < wheels_ph.size(); i++)
                {

                        track_current.updateMarker(i, flow.getSpherical(i));
                        track_new.updateMarker(i, flow.getContact(i));

                        track_estimated.updateMarker(i, flow.getEstimate(i));
                        //      std::cout << flow.getContact(i)[0] << "\t" <<
                        //      flow.getContact(i)[1]
                        //                << "\t" << flow.getContact(i)[2] << "\t";
                        //      std::cout << flow.getEstimate(i)[0] << "\t" <<
                        //      flow.getEstimate(i)[1]
                        //                << "\t" << flow.getEstimate(i)[2] << "\t";
                        //      std::cout << flow.getModelDiff(i)[0] << "\t" <<
                        //      flow.getModelDiff(i)[1]
                        //                << "\t" << flow.getModelDiff(i)[2] << "\t";
                        //      std::cout << flow.getEquationDiff(i)[0] << "\t" <<
                        //      flow.getEquationDiff(i)[1]
                        //                << "\t" << flow.getEquationDiff(i)[2] << "\t";
                }

                std::cout << std::endl;
                flow.update();

                track_estimated.publish();
                track_current.publish();
                track_new.publish();

                robot.update();
        }
}

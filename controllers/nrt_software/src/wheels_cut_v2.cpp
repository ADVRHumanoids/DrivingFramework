#include <config.h>

#include <mwoibn/loaders/robot.h>
#include "mgnss/controllers/wheeled_motion_full.h"

#include "mgnss/controllers/wheeled_motion_com.h"
#include "mgnss/controllers/wheeled_references.h"
#include <custom_services/updatePDGains.h>

bool evenstHandler ( custom_services::updatePDGains::Request& req,
                     custom_services::updatePDGains::Response& res,
                     mwoibn::SupportPolygon* support, mwoibn::WheeledMotionCom* controller );

int main ( int argc, char** argv )
{
        ros::init ( argc, argv, "wheels_reference" ); // initalize node

        ros::NodeHandle n;

        // init wheels_controller
        mwoibn::loaders::Robot loader;

        mwoibn::robot_class::Robot& robot = loader.init ( std::string(DRIVING_FRAMEWORK_WORKSPACE) + "DrivingFramework/locomotion_framework/configs/mwoibn_v2.yaml", "default", std::string(DRIVING_FRAMEWORK_WORKSPACE) + "DrivingFramework/"
                                                          "locomotion_framework/configs/lower_body.yaml" );

        mwoibn::WheeledMotionCom wheeld_controller ( robot );

        mwoibn::SupportPolygon support ( 0.45, 0.22 );

        support.setBase ( 0.25, 0.125 );          // HARDCODED

        support.setUpperLimit ( -10 * 3.1416 / 180 );

        support.setLowerLimit ( -80 * 3.1416 / 180 );
        support.setRadious ( 0.38 );
        support.setStep ( 0.0005 );


//        mwoibn::Base base;

//  base.heading.setUpperLimit(2 * 3.1416 / 180);
//  base.heading.setLowerLimit(-2 * 3.1416 / 180);
        // ros topics/service support
        ros::ServiceServer service =
                n.advertiseService<custom_services::updatePDGains::Request, custom_services::updatePDGains::Response> (
                        "wheels_command",
                        boost::bind ( &evenstHandler, _1, _2, &support, &wheeld_controller ) );

// starting

        support.setCurrent ( wheeld_controller.getSupportReference() );
        support.setDesired ( wheeld_controller.getSupportReference() );

        while ( ros::ok() ) {
                support.update();
                wheeld_controller.fullUpdate ( support.get() );
        }

}

bool evenstHandler (custom_services::updatePDGains::Request& req,
                    custom_services::updatePDGains::Response& res,
                    mwoibn::SupportPolygon* support, mwoibn::WheeledMotionCom *controller )
{
//  std::cout << "req\t" << req.p << "\t" << req.d << "\t" << req.nr << std::endl;

        if ( req.p == 1 ) { // base
                if ( req.d == 1 )
                        controller->setBaseDotX ( req.nr/100.0 );
                else if ( req.d == 2 )
                        controller->setBaseDotY ( req.nr/100.0 );
                else if ( req.d == 3 )
                        controller->setBaseDotZ ( req.nr/100.0 );
                else if ( req.d == 4 ) {
                        controller->setBaseDotHeading ( req.nr/100.0 );
                }
                else if ( req.d == 5)
                        controller->setBaseRotVelX ( req.nr/100.0 );
                else if ( req.d == 6)
                        controller->setBaseRotVelY ( req.nr/100.0 );
                else{
                        res.success = false;
                        return false;
                }
                res.success = true;
                return true;
        }
        if ( req.p == 2 ) { // base
                if ( req.d == 1 )
                        controller->setBaseX ( req.nr/100.0 );
                else if ( req.d == 2 )
                        controller->setBaseY ( req.nr/100.0 );
                else if ( req.d == 3 )
                        controller->setBaseZ ( req.nr/100.0 );
                else if ( req.d == 4 )
                        controller->setBaseHeading ( req.nr/100.0 );
                else if ( req.d == 5)
                        controller->rotateBaseX ( req.nr/100.0 );
                else if ( req.d == 6)
                        controller->rotateBaseY ( req.nr/100.0 );
                else{
                        res.success = false;
                        return false;
                }
                res.success = true;
                return true;
        }
        if ( req.p == 3 ) { // support
                mwoibn::SUPPORT_MOTION motion;
                mwoibn::SUPPORT_STATE state;

                if ( req.d > 0 && req.d < 3 ) {
                        motion = static_cast<mwoibn::SUPPORT_MOTION> ( req.d );

                        if ( req.nr > 3 || req.nr < 0 ) {
                                res.success = false;
                                return false;
                        }
                        state = static_cast<mwoibn::SUPPORT_STATE> ( req.nr );
                        support->initMotion ( motion, state );
                } else if ( req.d == 3 ) {
                        support->setStep ( req.nr/10000.0 );
                } else if ( req.d == 4 ) {
                        support->setUpperLimit ( req.nr/100.0 );
                } else if ( req.d == 5 ) {
                        support->setLowerLimit ( req.nr/100.0 );
                } else if ( req.d == 6 ) {
                        support->setRadious ( req.nr/100.0 );
                } else{
                        res.success = false;
                        return false;
                }
                res.success = true;
                return true;
        } else if ( req.p == 4 ) { // base
                if(req.d > 3 || req.d < 0) {
                        res.success = false;
                        return false;
                }
                controller->setSteering ( req.d, req.nr*3.14/180 );
        } else if ( req.p == 5 ) { // base
                if(req.d > 3 || req.d < 0) {
                        res.success = false;
                        return false;
                }
                controller->setCamber ( req.d, req.nr*3.14/180/10 );
        } else if ( req.p == 6 ) { // base
                if(req.d > 3 || req.d < 0) {
                        res.success = false;
                        return false;
                }
                controller->setCastor ( req.d, req.nr*3.14/180/10 );
        }
        else{
                res.success = false;
                return false;
        }

        res.success = true;
        return true;
}

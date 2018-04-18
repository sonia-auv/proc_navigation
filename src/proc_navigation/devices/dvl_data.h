//
// Created by jeremie on 9/14/16.
//

#ifndef DVLDATA_H
#define DVLDATA_H

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <eigen3/Eigen/Geometry>
#include "navigation_device.h"

namespace proc_navigation
{
    class DvlData: public NavigationDevice 
    {
        typedef void (DvlData::*IntegrationMethodT) (const double &);
    public:
        //==========================================================================
        // C O N S T  ,  T Y P E D E F   A N D   E N U M

        const double BAR_TO_METER_OF_WATER = 10.1972;

        enum IntegrationMethodType
        {
            StdMethod  = 0,
            RKMethod,
            DefaultMethod
        };

        //==========================================================================
        // P U B L I C   C / D T O R S

        DvlData(IntegrationMethodType integrationMethodType = RKMethod);
        ~DvlData();

        //==========================================================================
        // P U B L I C   M E T H O D S

        void DvlTwistCallback(geometry_msgs::TwistStamped msg);
        void DvlPressureCallback(sensor_msgs::FluidPressure msg);


        Eigen::Vector3d GetPositionXYZ();
        Eigen::Vector3d GetVelocityXYZ();
        double GetPositionZFromPressure();
        sensor_msgs::FluidPressure GetPressure();

    private:
        //==========================================================================
        // P R I V A T E   M E T H O D S
        void StdIntegrationMethod(const double &dt_sec);
        void RKIntegrationMethod(const double &dt_sec);

        //==========================================================================
        // P R I V A T E   M E M B E R S
        ros::Time last_timestamp_;

        Eigen::Vector3d positionIncrement_;
        Eigen::MatrixXd historyPositionIncrement_;

        geometry_msgs::TwistStamped dvl_twist_;
        sensor_msgs::FluidPressure dvl_pressure_;

        IntegrationMethodT integrationMethod_;
    };

}
#endif // DVLDATA_H

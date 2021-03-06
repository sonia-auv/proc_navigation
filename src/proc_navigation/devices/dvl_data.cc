/**
 * \file	dvl_data.cc
 * \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	9/14/2016
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include "dvl_data.h"

namespace proc_navigation {

    //==============================================================================
    // C / D T O R S   S E C T I O N

    //------------------------------------------------------------------------------
    //
    DvlData::DvlData(IntegrationMethodType integrationMethodType) :
            last_timestamp_(ros::Time::now())
    {
        positionIncrement_        = Eigen::Vector3d::Zero();
        historyPositionIncrement_ = Eigen::MatrixXd::Zero(3, 4);

        switch (integrationMethodType)
        {
            case StdMethod :
                integrationMethod_ = &DvlData::StdIntegrationMethod;
                break;
            case RKMethod  :
                integrationMethod_ = &DvlData::RKIntegrationMethod;
                break;
            default :
                integrationMethod_ = &DvlData::StdIntegrationMethod;
                break;
        }
    }

    //------------------------------------------------------------------------------
    //
    DvlData::~DvlData() { }

    //==============================================================================
    // M E T H O D   S E C T I O N

    //------------------------------------------------------------------------------
    //
    void DvlData::DvlTwistCallback(geometry_msgs::TwistStamped msg)
    {
        dvl_twist_ = msg;
        SetNewDataReady();
    }

    //------------------------------------------------------------------------------
    //
    void DvlData::DvlPressureCallback(sensor_msgs::FluidPressure msg)
    {
        dvl_pressure_ = msg;
        SetNewDataReady();
    }

    //------------------------------------------------------------------------------
    //
    Eigen::Vector3d DvlData::GetPositionXYZ()
    {
        ros::Duration dt = ros::Time::now() - last_timestamp_;
        double dt_sec = dt.toSec();

        (this->*integrationMethod_)(dt_sec);

        last_timestamp_ = ros::Time::now();

        return positionIncrement_;
    }

    //------------------------------------------------------------------------------
    //
    void DvlData::StdIntegrationMethod(const double &dt_sec)
    {
        positionIncrement_ << dvl_twist_.twist.linear.x * dt_sec, dvl_twist_.twist.linear.y * dt_sec, dvl_twist_.twist.linear.z * dt_sec;
    }

    //------------------------------------------------------------------------------
    //
    void DvlData::RKIntegrationMethod(const double &dt_sec)
    {
        positionIncrement_ << dvl_twist_.twist.linear.x * dt_sec, dvl_twist_.twist.linear.y * dt_sec, dvl_twist_.twist.linear.z * dt_sec;
        historyPositionIncrement_.block<3,3>(0, 1) = historyPositionIncrement_.block<3,3>(0, 0);
        historyPositionIncrement_.col(0) = positionIncrement_;
        positionIncrement_ = (1.0 / 6.0) * (historyPositionIncrement_.col(0) + 2 * historyPositionIncrement_.col(1) + 2 * historyPositionIncrement_.col(2) + historyPositionIncrement_.col(3));
    }

    //------------------------------------------------------------------------------
    //
    Eigen::Vector3d DvlData::GetVelocityXYZ()
    {
        Eigen::Vector3d twist;
        twist << dvl_twist_.twist.linear.x, dvl_twist_.twist.linear.y, dvl_twist_.twist.linear.z;
        return twist;
    }

    //------------------------------------------------------------------------------
    //
    sensor_msgs::FluidPressure DvlData::GetPressure()
    {
        return dvl_pressure_;
    }

    //------------------------------------------------------------------------------
    //
    double DvlData::GetPositionZFromPressure()
    {
        return dvl_pressure_.fluid_pressure * BAR_TO_METER_OF_WATER;
    }

}

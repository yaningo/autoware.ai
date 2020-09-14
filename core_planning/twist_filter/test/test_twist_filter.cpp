/*
 * Copyright (C) 2018-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "velocity_limit.hpp"

namespace twist_filter
{
    TEST(TwistFilterTest, test_longitudinal_twist_filter) {
        geometry_msgs::TwistStamped twist;
        twist.twist.linear.x = 7.0;

        geometry_msgs::TwistStamped out = twist_filter::longitudinalLimitTwist(twist, 5.0);

        ASSERT_DOUBLE_EQ(5.0, out.twist.linear.x);

        geometry_msgs::TwistStamped twist2;
        twist2.twist.linear.x = 50.0;

        geometry_msgs::TwistStamped out2 = twist_filter::longitudinalLimitTwist(twist2, 100.0);

        ASSERT_LT(out2.twist.linear.x, 36.0);
    }

    TEST(TwistFilterTest, test_longitudinal_ctrl_filter) {
        autoware_msgs::ControlCommandStamped ccs;
        ccs.cmd.linear_velocity = 7.0;

        autoware_msgs::ControlCommandStamped out = twist_filter::longitudinalLimitCtrl(ccs, 5.0);

        ASSERT_DOUBLE_EQ(5.0, out.cmd.linear_velocity);

        autoware_msgs::ControlCommandStamped ccs2;
        ccs2.cmd.linear_velocity = 50.0;

        autoware_msgs::ControlCommandStamped out2 = twist_filter::longitudinalLimitCtrl(ccs2, 100.0);

        ASSERT_LT(out2.cmd.linear_velocity, 36.0);
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

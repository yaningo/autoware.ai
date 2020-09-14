// TODO: Add CARMA copyright notice

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "twist_filter.hpp"

namespace twist_filter
{
    TEST(TwistFilterTest, test_longitudinal_twist_filter) {
        twist_filter::TwistFilter tf(nullptr, nullptr);

        tf.longitudinal_velocity_limit_ = 5.0;

        geometry_msgs::TwistStamped twist;
        twist.twist.linear.x = 7.0;

        geometry_msgs::TwistStamped out = tf.longitudinalLimitTwist(twist);

        ASSERT_DOUBLE_EQ(5.0, out.twist.linear.x);

        twist_filter::TwistFilter tf2(nullptr, nullptr);

        tf2.longitudinal_velocity_limit_ = 100.0;

        geometry_msgs::TwistStamped twist2;
        twist2.twist.linear.x = 50.0;

        geometry_msgs::TwistStamped out2 = tf2.longitudinalLimitTwist(twist2);

        ASSERT_LT(36.0, out2.twist.linear.x);
    }

    TEST(TwistFilterTest, test_longitudinal_ctrl_filter) {
        twist_filter::TwistFilter tf(nullptr, nullptr);

        tf.longitudinal_velocity_limit_ = 5.0;

        autoware_msgs::ControlCommandStamped ccs;
        ccs.cmd.linear_velocity = 7.0;

        autoware_msgs::ControlCommandStamped out = tf.longitudinalLimitCtrl(ccs);

        ASSERT_DOUBLE_EQ(5.0, out.cmd.linear_velocity);

        twist_filter::TwistFilter tf2(nullptr, nullptr);

        tf2.longitudinal_velocity_limit_ = 100.0;

        autoware_msgs::ControlCommandStamped ccs2;
        ccs2.cmd.linear_velocity = 7.0;

        autoware_msgs::ControlCommandStamped out2 = tf2.longitudinalLimitCtrl(ccs2);


        ASSERT_LT(36.0, out2.cmd.linear_velocity);
    }
}

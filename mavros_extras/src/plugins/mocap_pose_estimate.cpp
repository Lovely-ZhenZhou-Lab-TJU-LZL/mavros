/**
 * @brief MocapPoseEstimate plugin
 * @file mocap_pose_estimate.cpp
 * @author Tony Baltovski <tony.baltovski@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015,2016 Vladimir Ermakov, Tony Baltovski.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>


namespace mavros {
namespace extra_plugins{
/**
 * @brief MocapPoseEstimate plugin
 *
 * Sends motion capture data to FCU.
 */
class MocapPoseEstimatePlugin : public plugin::PluginBase
{
public:
	MocapPoseEstimatePlugin() : PluginBase(),
		mp_nh("~mocap")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		bool use_tf;
		bool use_pose;

		/** @note For VICON ROS package, subscribe to TransformStamped topic */
		mp_nh.param("use_tf", use_tf, false);

		/** @note For Optitrack ROS package, subscribe to PoseStamped topic */
		mp_nh.param("use_pose", use_pose, true);

		if (use_tf && !use_pose) {
			mocap_tf_sub = mp_nh.subscribe("tf", 1, &MocapPoseEstimatePlugin::mocap_tf_cb, this);
		}
		else if (use_pose && !use_tf) {
			mocap_pose_sub = mp_nh.subscribe("pose", 1, &MocapPoseEstimatePlugin::mocap_pose_cb, this);
		}
		else {
			ROS_ERROR_NAMED("mocap", "Use one motion capture source.");
		}
		last_send_timestamp = ros::WallTime::now();
		rs_odom_sub = mp_nh.subscribe("/ukf_predict/odometry", 1, &MocapPoseEstimatePlugin::odom_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle mp_nh;

	ros::Subscriber mocap_pose_sub;
	ros::Subscriber rs_odom_sub;
	ros::Subscriber mocap_tf_sub;

	ros::WallTime last_send_timestamp;

	/* -*- low-level send -*- */
	void mocap_pose_send
		(uint64_t usec,
			Eigen::Quaterniond &q,
			Eigen::Vector3d &v)
	{
		mavlink::common::msg::ATT_POS_MOCAP pos;

		pos.time_usec = usec;
		ftf::quaternion_to_mavlink(q, pos.q);
		pos.x = v.x();
		pos.y = v.y();
		pos.z = v.z();

		UAS_FCU(m_uas)->send_message_ignore_drop(pos);
	}

	void odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
		if (std::abs((ros::WallTime::now() - last_send_timestamp).toSec()) < 1.0f/20.0f) {
			return;
		}
		last_send_timestamp = ros::WallTime::now();
		Eigen::Quaterniond q;;
		q.w() = msg->pose.pose.orientation.w;
		q.x() = msg->pose.pose.orientation.x;
		q.y() = -msg->pose.pose.orientation.y;
		q.z() = -msg->pose.pose.orientation.z;
		auto position = Eigen::Vector3d(
			msg->pose.pose.position.x,
			-msg->pose.pose.position.y,
			-msg->pose.pose.position.z);
		mocap_pose_send(msg->header.stamp.toNSec()/1000,
			q,
			position);	
	}

	/* -*- mid-level helpers -*- */
	void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
	{
		Eigen::Quaterniond q_enu;

		tf::quaternionMsgToEigen(pose->pose.orientation, q_enu);
		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(q_enu));

		auto position = ftf::transform_frame_enu_ned(
				Eigen::Vector3d(
					pose->pose.position.x,
					pose->pose.position.y,
					pose->pose.position.z));

		mocap_pose_send(pose->header.stamp.toNSec() / 1000,
				q,
				position);
	}

	/* -*- callbacks -*- */
	void mocap_tf_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
	{
		Eigen::Quaterniond q_enu;

		tf::quaternionMsgToEigen(trans->transform.rotation, q_enu);
		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(q_enu));

		auto position = ftf::transform_frame_enu_ned(
				Eigen::Vector3d(
					trans->transform.translation.x,
					trans->transform.translation.y,
					trans->transform.translation.z));

		mocap_pose_send(trans->header.stamp.toNSec() / 1000,
				q,
				position);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MocapPoseEstimatePlugin, mavros::plugin::PluginBase)

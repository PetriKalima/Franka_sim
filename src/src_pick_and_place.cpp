/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 12/17/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#include <exercise2/functions.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle node("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    std_srvs::Empty srv_reset;
    ros::service::waitForService("/lumi_mujoco/reset"); 
    ros::service::call("/lumi_mujoco/reset", srv_reset);

    //Load MoveGroup interface and moveit visual tools
    moveit::planning_interface::MoveGroupInterface g_arm("lumi_arm");
    moveit::planning_interface::MoveGroupInterface g_hand("lumi_hand");
    moveit_visual_tools::MoveItVisualTools vis("base_link");
    vis.loadMarkerPub(true);
    vis.deleteAllMarkers();
    vis.trigger();

    //Get Start state
    const auto state_ptr = g_arm.getCurrentState(10.0);
    if (!state_ptr) {
        ROS_ERROR_STREAM("Cannot get current state");
        return -1;
    }
    auto state = *state_ptr;

    const auto jmg = state.getJointModelGroup("lumi_arm"); //joint model group used for IK computation
    const auto ee_link = "lumi_ee";
    const Eigen::Affine3d arm_to_ee = state.getGlobalLinkTransform(g_arm.getEndEffectorLink()).inverse() *
                                      state.getGlobalLinkTransform(ee_link);

    Eigen::Affine3d e1 = state.getGlobalLinkTransform(ee_link) * Eigen::Translation3d(0.1, 0.0, 0.0);
    Eigen::Affine3d e2 = state.getGlobalLinkTransform(ee_link) * Eigen::Translation3d(-0.1, 0.0, 0.0) *
                         Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());

    vis.publishAxis(e1);
    vis.publishAxis(e2);
    vis.trigger();

    std::vector<moveit_msgs::RobotTrajectory> trajectories;

    //planning to pose
    g_arm.setStartState(state);
    if (!state.setFromIK(jmg, e1, ee_link)) {
        ROS_ERROR_STREAM("Cannot set arm position with IK");
        return -1;
    }
    trajectories.push_back(planToState(g_arm, state));
    if (trajectories.back().joint_trajectory.points.empty()) {
        return -1;
    }
    state.setVariablePositions(trajectories.back().joint_trajectory.joint_names,
                               trajectories.back().joint_trajectory.points.back().positions);

    //cartesian pose
    g_arm.setStartState(state);
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(e2 * arm_to_ee.inverse(), pose);
    moveit_msgs::RobotTrajectory rtraj;
    const auto d = g_arm.computeCartesianPath({pose}, 0.01, 1.4, rtraj);
    if (d < 0.99) {
        ROS_ERROR_STREAM("Cannot interpolate to the grasping position");
        return -1;
    }
    trajectories.push_back(rtraj);

    //Visualise all trajectories
    for (const auto &t : trajectories) {
        vis.publishTrajectoryLine(t, state.getLinkModel(ee_link), jmg);
    }
    vis.trigger();

    if (askContinue()) {
// ...
    } else {
// ....
    }

    //open gripper
    g_hand.setStartStateToCurrentState();
    const auto traj = getGripperTrajectory(g_hand, true);
    if (!traj.joint_trajectory.points.empty()) {
        g_hand.execute(trajectoryToPlan(traj));
    }

    return 0;
}

/*
The door_demo demonstrates a proof of concept operation task of the door model
The code is loosely based on the 'robotic manipulation' -course's exercise 2: pick_and_place.cpp
*/

#include <functions.h>
#include <std_srvs/Empty.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "door_demo");
    ros::NodeHandle node("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    std_srvs::Empty srv_reset;
    ros::service::waitForService("/lumi_mujoco/reset"); 
    ros::service::call("/lumi_mujoco/reset", srv_reset);

    // Load MoveGroup interface and moveit visual tools
    moveit::planning_interface::MoveGroupInterface g_arm("lumi_arm");
    moveit::planning_interface::MoveGroupInterface g_hand("lumi_hand");
    moveit_visual_tools::MoveItVisualTools vis("base_link");
    vis.loadMarkerPub(true);
    vis.deleteAllMarkers();
    vis.trigger();

    // Get Start state
    const auto state_ptr = g_arm.getCurrentState(10.0);
    if (!state_ptr) {
        ROS_ERROR_STREAM("Cannot get current state");
        return -1;
    }
    auto state = *state_ptr;

    const auto jmg = state.getJointModelGroup("lumi_arm"); // joint model group used for IK computation
    const auto ee_link = "lumi_ee";
    const Eigen::Affine3d arm_to_ee = state.getGlobalLinkTransform(g_arm.getEndEffectorLink()).inverse() * state.getGlobalLinkTransform(ee_link);// * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

    // Geometry message which is populated with transformations from publish_frames.launch
    geometry_msgs::TransformStamped base_to_handle_core_pose;
    geometry_msgs::TransformStamped base_to_hinge_pose;
    geometry_msgs::TransformStamped hinge_to_handle_pose;

    // handle_core and door_hinge are defined in publish_frames.launch
    // Read the tf into variable
	base_to_handle_core_pose = tfBuffer.lookupTransform("base_link", "handle_core", ros::Time(0), ros::Duration(20.0));
    base_to_hinge_pose = tfBuffer.lookupTransform("base_link", "door_link", ros::Time(0), ros::Duration(20.0));
    hinge_to_handle_pose = tfBuffer.lookupTransform("door_link", "handle_core", ros::Time(0), ros::Duration(20.0));

    Eigen::Affine3d handle_core;
    Eigen::Affine3d hinge;
    Eigen::Affine3d hinge_handle;

    // Convert the geometric message into eigen
	tf::transformMsgToEigen(base_to_handle_core_pose.transform, handle_core);
	tf::transformMsgToEigen(base_to_hinge_pose.transform, hinge);
    tf::transformMsgToEigen(hinge_to_handle_pose.transform, hinge_handle);


    // Definitions of grasp locations
    // Radius of grasp location from handle core (in metres)
    float grasp_radius = .1; 

    //pregrasp frame
    Eigen::Affine3d pregrasp = handle_core * Eigen::Translation3d(grasp_radius, 0, -0.1); //pregrasp 10cm away from handle

    //grasp frame
    Eigen::Affine3d grasp = handle_core * Eigen::Translation3d(grasp_radius, 0, 0);

    //frames for opening the handle
    int n = 10; // number of points for move
    float handle_angle = 30; //rotation angle in degrees
    Eigen::Affine3d translation;
    Eigen::Affine3d point[n];
    for (int i=1; i<=n; i++){
        float a = handle_angle/n*i * M_PI/180; //step angle in degrees, convert to rad
        float dx = cos(a)*grasp_radius; //step in x axis
        float dy = sin(a)*grasp_radius; //step in y axis
        translation = Eigen::Translation3d( dx, dy, 0) * Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ());
        point[i-1] = handle_core * translation;
        vis.publishAxis(point[i-1]);
    }

    //frames for opening the door
    int m = 30; // number of points for move
    float door_angle = 30; //rotation angle in degrees
    Eigen::Affine3d points[m];
    for (int j=1; j<=m; j++){
        float a2 = door_angle/m*j * M_PI/180; //step angle in degrees, convert to rad
        points[j-1] = hinge * Eigen::AngleAxisd(a2, Eigen::Vector3d::UnitY()) * hinge_handle * translation;        
        vis.publishAxis(points[j-1]);
    }


    // save the poses
    vis.publishAxis(pregrasp);
    vis.publishAxis(grasp);
    //vis.publishAxis(handle_core);
    //vis.publishAxis(hinge);

    // push the poses to rviz
	vis.trigger();

    // execute the trajectories
    std::vector<moveit_msgs::RobotTrajectory> trajectories;


// plan to pregrasp
    g_arm.setStartState(state);
    if (!state.setFromIK(jmg, pregrasp, ee_link)) {
        ROS_ERROR_STREAM("Cannot set arm position with IK");
        return -1;
    }
	moveit_msgs::RobotTrajectory t1; //create trajectory
	t1 = planToState(g_arm, state); //calculate
    trajectories.push_back(t1); //something
    if (trajectories.back().joint_trajectory.points.empty()) {
        return -1;
    }
    state.setVariablePositions(trajectories.back().joint_trajectory.joint_names,
                               trajectories.back().joint_trajectory.points.back().positions);
	g_arm.execute(trajectoryToPlan(t1)); //execute trajectory

// open gripper
    g_hand.setStartStateToCurrentState();
    const auto traj1 = getGripperTrajectory(g_hand, true);
    if (!traj1.joint_trajectory.points.empty()) {
        g_hand.execute(trajectoryToPlan(traj1));
    }

// cartesian to grasp
	g_arm.setStartStateToCurrentState();
    geometry_msgs::Pose pose1;
    tf::poseEigenToMsg(grasp * arm_to_ee.inverse(), pose1);
    moveit_msgs::RobotTrajectory rtraj1;
    const auto d1 = g_arm.computeCartesianPath({pose1}, 0.01, 1.4, rtraj1);
    if (d1 < 0.99) {
        ROS_ERROR_STREAM("Cannot interpolate to the grasping position");
        return -1;
    }
    trajectories.push_back(rtraj1);
	g_arm.execute(trajectoryToPlan(rtraj1));

// close gripper
    g_hand.setStartStateToCurrentState();
    const auto traj2 = getGripperTrajectory(g_hand, false);
    if (!traj2.joint_trajectory.points.empty()) {
        g_hand.execute(trajectoryToPlan(traj2));
    }

// cartesian to open handle (point[i])
    geometry_msgs::Pose pose[n];
    moveit_msgs::RobotTrajectory rtraj[n];
    for (int i=0; i<n; i++){
        g_arm.setStartStateToCurrentState();
        tf::poseEigenToMsg(point[i] * arm_to_ee.inverse(), pose[i]);
        if (g_arm.computeCartesianPath({pose[i]}, 0.01, 1.4, rtraj[i]) < 0.99) {
            ROS_ERROR_STREAM("Cannot interpolate to the handle opening position(s)");
            return -1;
        }
        trajectories.push_back(rtraj[i]);
        g_arm.execute(trajectoryToPlan(rtraj[i]));
    }

// cartesian to open door (points[j])
    geometry_msgs::Pose poses[m];
    moveit_msgs::RobotTrajectory rtrajs[m];
    for (int j=0; j<m; j++){
        g_arm.setStartStateToCurrentState();
        tf::poseEigenToMsg(points[j] * arm_to_ee.inverse(), poses[j]);
        if (g_arm.computeCartesianPath({poses[j]}, 0.01, 1.4, rtrajs[j]) < 0.99) {
            ROS_ERROR_STREAM("Cannot interpolate to the door opening position(s)");
            return -1;
        }
        trajectories.push_back(rtrajs[j]);
        g_arm.execute(trajectoryToPlan(rtrajs[j]));
    }


    //Visualise all trajectories
    for (const auto &t : trajectories) {
        vis.publishTrajectoryLine(t, state.getLinkModel(ee_link), jmg);
    }
    vis.trigger();

    return 0;
}


/*
The door_demo demonstrates a proof of concept operation task of the door model
The code is loosely based on the 'robotic manipulation' -course's exercise 2: pick_and_place.cpp

This demo should work with the parametrisation script, as all translations are calulated based on the urdf.
The values the user might want to change are the:
 grasp_radius and depth_fix: the length from the handlecore to grasping location and the depth of the grasp
 n and m: the amount of points for handle opening and door opening paths
 handle_angle and door_angle: the angles the respective rotational joints are opened to
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
    const Eigen::Affine3d arm_to_ee = state.getGlobalLinkTransform(g_arm.getEndEffectorLink()).inverse() * state.getGlobalLinkTransform(ee_link); 

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

    // Geometry message which is populated with transformations from publish_frames.launch
    geometry_msgs::TransformStamped base_to_handle_core_pose;
    geometry_msgs::TransformStamped base_to_hinge_pose;
    geometry_msgs::TransformStamped hinge_to_handle_pose;

    // Read the tf into variable
	base_to_handle_core_pose = tfBuffer.lookupTransform("base_link", "handle_core", ros::Time(0), ros::Duration(20.0));
    base_to_hinge_pose = tfBuffer.lookupTransform("base_link", "door_link", ros::Time(0), ros::Duration(20.0));
    hinge_to_handle_pose = tfBuffer.lookupTransform("door_link", "handle_core", ros::Time(0), ros::Duration(20.0));

    // initialise
    Eigen::Affine3d handle_core;
    Eigen::Affine3d hinge;
    Eigen::Affine3d hinge_handle;

    // Convert the geometric message into eigen
	tf::transformMsgToEigen(base_to_handle_core_pose.transform, handle_core);
	tf::transformMsgToEigen(base_to_hinge_pose.transform, hinge);
    tf::transformMsgToEigen(hinge_to_handle_pose.transform, hinge_handle);

// Definitions of grasp locations

    // Radius of grasp location from handle core (in metres)
    float grasp_radius = 0.1; // seems good enough
    // Depth fix translation for the ee to grab the handle
    float depth_fix = 0.02; // the ee misses the handle, and 2cm puts the handle about directly in the middle of the fingers

    // Creates a translation where the axis is reoriented for the robot to approach from
    Eigen::Affine3d rotx = Eigen::Translation3d(0,0,0) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX());
    // Creates a "fix" translation where the robot is oriented correctly and the end effector depth is fixed so the fingers will make contact 
    // with the handle
    Eigen::Affine3d handle_core_fix = handle_core * rotx * Eigen::Translation3d(0, 0, depth_fix);

// Pregrasp pose
    Eigen::Affine3d pregrasp = handle_core_fix * Eigen::Translation3d(grasp_radius, 0, -0.1); //approach point which is 10cm away from handle

// Grasp pose
    Eigen::Affine3d grasp = handle_core_fix * Eigen::Translation3d(grasp_radius, 0, 0); //grasp point grasp_radius metres away from the rotational core

// Poses for opening the handle
    int n = 10; // number of points to move through
    float handle_angle = 30; //rotation angle in degrees
    Eigen::Affine3d translation; //initialise 
    Eigen::Affine3d point[n]; //initialise
    for (int i=1; i<=n; i++){ //for each point the robot will move through: calculate the pose
        float a = handle_angle/n*i * M_PI/180; //step angle in degrees, convert to rad
        float dx = cos(a)*grasp_radius; //step in x axis
        float dy = sin(a)*grasp_radius; //step in y axis
        //translation from angle core to grasp points which are circularly around the core
        translation = Eigen::Translation3d( dx, dy, 0) * Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()); 
        point[i-1] = handle_core_fix * translation; //base_link to grasp point
        vis.publishAxis(point[i-1]); //visualise point
    }

// Poses for opening the door
    int m = 30; // number of points to move through
    float door_angle = 90; // rotation angle in degrees
    Eigen::Affine3d points[m]; // initialise
    for (int j=1; j<=m; j++){ // for each point calculate the frame
        float a2 = door_angle/m*j * M_PI/180; //step angle in degrees, convert to rad
        // transformation from base_link to hinge * rotation around hinge * translation from hinge to handle * 
        // rotation to robot orientation * translation from core to grasp point
        points[j-1] = hinge * Eigen::AngleAxisd(-a2, Eigen::Vector3d::UnitZ()) * hinge_handle * rotx * translation;
        vis.publishAxis(points[j-1]); // visualise point
    }

    // visualise poses
    vis.publishAxis(pregrasp);
    vis.publishAxis(grasp);
    //vis.publishAxis(handle_core);
    //vis.publishAxis(hinge);

    // push the poses to rviz
	vis.trigger();

    // execute the trajectories
    std::vector<moveit_msgs::RobotTrajectory> trajectories;

    // path plan to the poses using RRT and cartesian path planning (planToState and computeCartesianPath), also open and close gripper

// plan to pregrasp
    g_arm.setStartState(state); // get starting state
    if (!state.setFromIK(jmg, pregrasp, ee_link)) { // calculate the move to pregrasp
        ROS_ERROR_STREAM("Cannot set arm position with IK"); // if error give notification
        return -1;
    }
	moveit_msgs::RobotTrajectory t1; //initialise
	t1 = planToState(g_arm, state); // give state to g_arm movegroup
    trajectories.push_back(t1); // push to trajectories
    if (trajectories.back().joint_trajectory.points.empty()) { //if the resulting trajectories are empty return -1
        return -1;
    }
    // calculate positions for each joint
    state.setVariablePositions(trajectories.back().joint_trajectory.joint_names, trajectories.back().joint_trajectory.points.back().positions);
	g_arm.execute(trajectoryToPlan(t1)); // execute trajectory

// open gripper
    g_hand.setStartStateToCurrentState(); // get starting state
    const auto traj1 = getGripperTrajectory(g_hand, true); // create trajectory to open-state
    if (!traj1.joint_trajectory.points.empty()) {
        g_hand.execute(trajectoryToPlan(traj1)); // if trajectory is not empty execute it
    }

// cartesian to grasp
	g_arm.setStartStateToCurrentState(); // get start state
    geometry_msgs::Pose pose1; // init
    tf::poseEigenToMsg(grasp * arm_to_ee.inverse(), pose1); // convert eigen to pose
    moveit_msgs::RobotTrajectory rtraj1; // init
    const auto d1 = g_arm.computeCartesianPath({pose1}, 0.01, 1.4, rtraj1); // compute the path with ee_stepsize 0.01 and jump_treshold 1.4
    if (d1 < 0.99) {
        ROS_ERROR_STREAM("Cannot interpolate to the grasping position"); // if the accuracy is lower than d1, return error (success factor, 1 is 100%)
        return -1;
    }
    trajectories.push_back(rtraj1); // push trajectory to trajectories
	g_arm.execute(trajectoryToPlan(rtraj1)); // execute

// close gripper
    g_hand.setStartStateToCurrentState();
    const auto traj2 = getGripperTrajectory(g_hand, false); //create trajectory to close-state
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


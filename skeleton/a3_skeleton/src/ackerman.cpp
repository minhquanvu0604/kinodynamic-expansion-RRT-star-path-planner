#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <deque>

#include "ackerman.h"
#include "kinematicExpansion/kinematicExpansion.h"

/**
 * @brief Default constructor
 * 
 * Default constructor should set all sensor attributes to a default value, here the default steering is 0
 * and the platform type to indicate ackerman
 * It also initialize a new thread object to execute the running of Ackerman in a new thread
*/
Ackerman::Ackerman(ros::NodeHandle nh) : steering_{0}
{
    nh_ = nh;

    platformType_ = pfms::PlatformType::ACKERMAN;

    subGoal_ = nh_.subscribe("/orange/goals", 1000, &Ackerman::goalCallback,this);

    pubBrake_ = nh_.advertise<std_msgs::Float64>("/orange/brake_cmd",3,false); ;
    pubSteering_ = nh_.advertise<std_msgs::Float64>("orange/steering_cmd",3,false); 
    pubThrottle_ = nh_.advertise<std_msgs::Float64>("/orange/throttle_cmd",3,false); ;

    //Publishing markers
    pubMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",1000,false);

    //Allowing an incoming service on /reach_goal (you need to change name depending on project)
    service_ = nh_.advertiseService("reach_goal", &Ackerman::request,this);    

    // Initialize a new thread and push it in the vector of threads
    // threads_.push_back(std::thread(&Ackerman::executeAckerman,this));
    threads_.push_back(std::thread(&Ackerman::executeAckerman,this));
    
    // Get parameters
    ros::NodeHandle pn("~");
    if (!pn.param<bool>("advance", advanced_, true))
        ROS_ERROR("Failed to get param 'advanced'");
};


void Ackerman::goalCallback (const geometry_msgs::PoseArray::ConstPtr& msg){
    
    demandGoals_ = *msg;
    demandGoalSet_ = true;

    auto goalMarkerArray = createGoalMarker(demandGoals_);
    pubMarker_.publish(goalMarkerArray);
}

bool Ackerman::request(std_srvs::SetBool::Request  &req,
             std_srvs::SetBool::Response &res)
{
    //When an incoming call arrives, we can respond to it here
    //Check what is in the service via rossrv info std_srvs/SetBool
    if (req.data){
        ROS_INFO_STREAM("Running requested");
        this->Controller::run();
    }
    else {
        ROS_INFO_STREAM("Stopping requested");
        running_ = false;
    }
    //We can reply in the two field of the return value
    res.success = true;
    res.message = "Service successful !!";

    return true; //We return true to indicate the service call sucseeded (your responce should indicate a value)
}


bool Ackerman::checkOriginToDestination(geometry_msgs::Pose origin,
                                    geometry_msgs::Point goal,
                                    double& distance,
                                    double& time,
                                    geometry_msgs::Pose& estimatedGoalPose){


    // ROS_INFO_STREAM("CHECKO ORIGIN: X=" << origin.position.x << ", Y=" << origin.position.y); 
    // ROS_INFO_STREAM("CHECKO GOAL: X=" << goal.x << ", Y=" << goal.y);    

    // Convert the coordinate of goal point from global frame to local frame    
    geometry_msgs::Point carOrigin = origin.position;
    geometry_msgs::Point globalGoal = goal;

    auto yaw = tf::getYaw(origin.orientation);
    geometry_msgs::Point localGoal = globalToLocalFrame(globalGoal, carOrigin, yaw);                    

    // Calculate the straight line distance from origin to target
    double linearDistanceToGoal = std::hypot(localGoal.x,localGoal.y);

    // Calculate alpha, this alpha ranges from -pi to pi
    double alpha = atan2(localGoal.y, localGoal.x);
    // ROS_INFO_STREAM("CHECKO LOCAL GOAL: X=" << localGoal.x << ", Y=" << localGoal.y);  

    // Find and correct the sweeping angle, which depends on the quadrant of goal point in local frame (while alpha does not) 
    double sweepingAngle = 2 * alpha; 

    // Calculate the turning radius R
    double turningRadius = linearDistanceToGoal / (2 * sin(std::abs(alpha)));

    // Update distancce and time to goal
    distance = std::abs(sweepingAngle) * turningRadius;
    time  = distance / CONSTANT_SPEED;

    // Update estimated goal pose
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    

    // Normalize the input angle to the range of -π (M_PI) to π
    // Convert from quartenion to radian
    yaw = tf::getYaw(origin.orientation);
    yaw = Controller::normalizeAngle(yaw + sweepingAngle);

    // Update steering 
    double steeringAngle = atan2(WHEELBASE * 2 * sin(alpha),linearDistanceToGoal);

    // if (updateSteering_){
    
    // Steering angle
    estimatedGoalPose.orientation = tf::createQuaternionMsgFromYaw(steeringAngle);
    steering_ = steeringAngle * STEERING_RATIO;
    ROS_INFO_STREAM("STEERING : " << steering_);

    // Check if less than the max steering angle allowed
    bool possible = std::abs(steeringAngle) < MAX_STEER_ANGLE;

    return possible;    
}


geometry_msgs::Point Ackerman::globalToLocalFrame(geometry_msgs::Point goal, geometry_msgs::Point robotPosition, double robotYaw) {
    // Calculate the relative position of the goal in the global frame
    double dx = goal.x - robotPosition.x;
    double dy = goal.y - robotPosition.y;

    geometry_msgs::Point goalRobotFrame;
    
    // Convert the relative position to the robot frame using the robot's yaw angle
    goalRobotFrame.x = dx * cos(robotYaw) + dy * sin(robotYaw);
    goalRobotFrame.y = -dx * sin(robotYaw) + dy * cos(robotYaw);

    return goalRobotFrame;
}


void Ackerman::sendCmd(double brake, double steering, double throttle) {

    std_msgs::Float64 brakef;
    brakef.data = brake;

    std_msgs::Float64 steeringf;
    steeringf.data = steering;

    std_msgs::Float64 throttlef;
    throttlef.data = throttle;

    pubBrake_.publish(brakef);
    pubSteering_.publish(steeringf);
    pubThrottle_.publish(throttlef);
}


/**
 * @brief Execute the Ackerman to reach goals
 * 
 * This function is ran in a separate thread to facilitate concurrency of execution
 * The function starts to execute when the new object is constructed. It then waits in the statement with wait() function 
 * of the conditional variable object. When all the necessary conditions are met in the @sa run function, the conditional variable 
 * sends notification to lock the mtxRunning_ and progress with execution. Then the Ackerman starts to run, hence the platform status
 * now turns to RUNNING
 * 
 * The start time is recorded from the start of the function and end time at the end to count the time duration of execution through all the goals
 * 
 * The running comprises two while loop for each goal, with one loop for the runnning and the other for braking
 * 
 * The workflow of the first loop is basically feeding the fixed steering for the current goal, and the throttle is constant to be 0.1 throughout. Additionally, 
 * the linear distance to goal is continuously updated at every time step to figure out how close the audi has got the goal. If that 
 * distance gets smaller than the brakeThreshold, the running loop ends. The brakeThreshold is calculated by a self-developed formula utilizing linear regression, which increases 
 * as the speed increases
 * 
 * The second loop handles the brake. It is provided by a different formula which increase the brake torque as the audi gets closer to the goal
 * The brake torque value starts from the variable indicating the min brake torque and gets to the max value when it is tolerance distance away 
 * from the goal. The second loop exits when the velocity of the audi becomes lower than the maxSpeedToProgressToNextGoal variable
 * 
 * In the two loops the distance travelled by the audi is recorded. It is the distance to goal of the whole paths between goals that have been
 * gone through, and the partial distance of the current position from the last goal
*/
void Ackerman::executeAckerman(){

    // Protecting running_ variable
    std::unique_lock<std::mutex> lckRunning(mtxRunning_);
    // Wait for running signal
    cvRun_.wait(lckRunning, [this]{return running_.load();});
    lckRunning.unlock();

    while (ros::ok()){

        // Advanced mode
        if (advanced_){
            ROS_INFO("ADV");
            autopilot();
        }
        // Normal mode
        else if (demandGoalSet_){

            LaserProcessing laserProcess;

            for (auto goal : demandGoals_.poses){
                goal_ = goal.position;

                // Get laserscan data
                std::unique_lock<std::mutex> lck (mtxLaser_);
                laserProcess.setLaserScan(laserScan_);
                lck.unlock();

                // Get current pose of ackerman
                auto odo = Controller::getOdometry();
                geometry_msgs::Pose pose = odo.pose.pose;
                Point ackermanPosition(pose.position.x, pose.position.y);

                // Get goal points
                auto conesInd = laserProcess.conePointIndex(&running_);
                auto cones = laserProcess.getPoints(conesInd, pose);

                // Determine if the goals are inside the path
                auto closestCones = laserProcess.getClosestPoints(cones, ackermanPosition);
                closestCones.push_back(ackermanPosition);

                auto hull = laserProcess.convexHull(closestCones);

                Point currentGoal = convertToPoint(goal_);
                bool inPath = laserProcess.pointInConvexPolygon(hull, currentGoal);
                if (!inPath)
                    {
                        ROS_ERROR("NOT IN PATH");

                    }
                
                // laserProcess.convexHull();
                reachGoal();
                braking();
            }

            // Ran all goals
            demandGoalSet_ = false;
        }
        rateLimit_.sleep();
    }   

}

// void Ackerman::executeADVAckerman(){

//     // Protecting running_ variable
//     std::unique_lock<std::mutex> lckRunning(mtxRunning_);
//     // Wait for running signal
//     cvRun_.wait(lckRunning, [this]{return running_.load();});
//     lckRunning.unlock();

//     while (ros::ok()){

//         if (demandGoalSet_){

//             LaserProcessing laserProcess;
            

//             for (auto goal : demandGoals_.poses){
//                 goal_ = goal.position;

//                 // Get laserscan data
//                 std::unique_lock<std::mutex> lck (mtxLaser_);
//                 laserProcess.setLaserScan(laserScan_);
//                 lck.unlock();

//                 // Get current pose of ackerman
//                 auto odo = Controller::getOdometry();
//                 geometry_msgs::Pose pose = odo.pose.pose;

//                 // Get goal points
//                 auto conesInd = laserProcess.conePointIndex(&running_);
//                 auto cones = laserProcess.getPoints(conesInd, pose);

//                 reachGoal();
//                 braking();
//             }

//             // Ran all goals
//             demandGoalSet_ = false;
//         }
//         rateLimit_.sleep();
//     }   

// }


void Ackerman::reachGoal(){

    // Protecting running_ variable
    std::unique_lock<std::mutex> lckRunning(mtxRunning_);
    // Wait for running signal
    cvRun_.wait(lckRunning, [this]{return running_.load();});
    lckRunning.unlock();

    LaserProcessing processLaser;
    KE ke(nh_);

    std::deque<std::vector<Point>> paths;
    double lastSteering;

    while (running_){

        double brake;        

        auto odo = Controller::getOdometry();
        geometry_msgs::Pose pose = odo.pose.pose;

        std::unique_lock<std::mutex> lck (mtxLaser_);
        processLaser.setLaserScan(laserScan_);
        lck.unlock();

        auto conesInd = processLaser.conePointIndex(&running_);
        auto cones = processLaser.getPoints(conesInd, pose);

        // Publish markers
        auto coneArray = createConesMarker(cones);
        pubMarker_.publish(coneArray);

        // //------------------INTEGRATE KE--------------------//
        // KNode startNode;
        // startNode.pose.position.x = pose.position.x;
        // startNode.pose.position.y = pose.position.y;
        // startNode.pose.yaw = tf::getYaw(pose.orientation);
        
        // // ROS_INFO("CREATE TREE");

        // ke.setInfo(startNode, cones);
        // ke.createTree();
        // // ROS_INFO("FINISH TREE");

        // Just to update steering_
        double dummyTravelledPath = 0;
        double dummyDistance = 0;
        geometry_msgs::Pose dummyEstimatedGoalPose;
        checkOriginToDestination(pose, goal_, dummyTravelledPath, dummyDistance, dummyEstimatedGoalPose);

        sendCmd(0,steering_, fixThrottle);

        // brakeThreshold: the minimum distance between the platform and goal to start applying brake
        // Tuning breakThreshold : plot the successful breakThreshold against the speed when braking starts 
        // Using linear regression found y=0.28x-0.43
        brakeThreshold_ = std::abs(0.28 * (std::abs(odo.twist.twist.linear.x)+std::abs(odo.twist.twist.linear.y)) - 0.48); 
        if (brakeThreshold_ < tolerance_) brakeThreshold_ = tolerance_; 

        // Stop running when tolerance away from the goal
        double instantDistanceToGoal = euclideanDistance(odo.pose.pose.position.x, odo.pose.pose.position.y, goal_.x, goal_.y);

        rateLimit_.sleep();

        // Break the running loop if it gets close to the goal by breakThreashold
        if(instantDistanceToGoal < brakeThreshold_) break;
    }
}


void Ackerman::braking(){

    // Protecting running_ variable
    std::unique_lock<std::mutex> lckRunning(mtxRunning_);
    // Wait for running signal
    cvRun_.wait(lckRunning, [this]{return running_.load();});
    lckRunning.unlock();

    while (running_){
        // Update odometry
        auto odo = Controller::getOdometry();

        // The break torque fed to the Pipe
        // The brake value is larger as the platforms approaches the goal, starting from minBrake to max brake torque
        double instantDistanceToGoal = euclideanDistance(odo.pose.pose.position.x, odo.pose.pose.position.y, goal_.x, goal_.y);
        double brake = ((MAX_BRAKE_TORQUE - kMinBrake_) / brakeThreshold_) * (brakeThreshold_ - instantDistanceToGoal) + kMinBrake_;

        // When the platform is tolerance_ value away from the goal, apply the max brake torque
        if (instantDistanceToGoal < tolerance_) brake = MAX_BRAKE_TORQUE;

        sendCmd(brake, steering_, 0);

        // Compute velocity
        double velocity = std::pow(std::pow(odo.twist.twist.linear.x,2)+ std::pow(odo.twist.twist.linear.x,2),0.5);

        rateLimit_.sleep();

        // Progress to next goal
        if(std::abs(velocity) < kMaxSpeedToProgressToNextGoal_) break;
    }
}


void Ackerman::autopilot(){

    ROS_WARN("ADVVVVV");

    // Protecting running_ variable
    std::unique_lock<std::mutex> lckRunning(mtxRunning_);
    // Wait for running signal
    cvRun_.wait(lckRunning, [this]{return running_.load();});
    lckRunning.unlock();

    LaserProcessing processLaser;
    KE ke(nh_);

    std::deque<std::vector<Point>> paths;
    double lastSteering;

    while (running_){
        
        rateLimit_.sleep();
        double brake = 0;        

        auto odo = Controller::getOdometry();
        geometry_msgs::Pose pose = odo.pose.pose;

        std::unique_lock<std::mutex> lck (mtxLaser_);
        processLaser.setLaserScan(laserScan_);
        lck.unlock();

        auto conesInd = processLaser.conePointIndex(&running_);
        auto cones = processLaser.getPoints(conesInd, pose);

        if (cones.empty())
            continue;

        // Publish markers
        auto coneArray = createConesMarker(cones);
        pubMarker_.publish(coneArray);


        // create KE tree
        KNode startNode;
        startNode.pose.position.x = pose.position.x;
        startNode.pose.position.y = pose.position.y;
        startNode.pose.yaw = tf::getYaw(pose.orientation);

        ke.setInfo(startNode, cones);
        ke.createTree();

        Point goal = ke.getGoal();
        // std::cout << "GET GOAL: X=" << goal.x << ", Y=" << goal.y << std::endl;
        geometry_msgs::Point geoGoal = convertToPointMsg(goal);
        // std::cout << "GE000 GOAL: X=" << geoGoal.x << ", Y=" << geoGoal.y << std::endl;

        // Just to update steering_
        double dummyTravelledPath = 0;
        double dummyDistance = 0;
        geometry_msgs::Pose estimatedGoalPose;
        checkOriginToDestination(pose, geoGoal, dummyTravelledPath, dummyDistance, estimatedGoalPose);

        // // Some tuning 
        double steeringAngle = std::abs(tf::getYaw(estimatedGoalPose.orientation));
        bool steepAngle = steeringAngle > M_PI / 8; 
        bool oscillation = std::abs(steeringAngle - lastSteering) > 0.08; 
        lastSteering = steeringAngle;

        double velocity = std::pow(std::pow(odo.twist.twist.linear.x,2)+ std::pow(odo.twist.twist.linear.x,2),0.5);
        bool highSpeed = velocity > 2;

        if (steepAngle && highSpeed)
            brake = steeringAngle *50 ;
        
        else if (oscillation && highSpeed)
            brake = steeringAngle;
        // --------------------------------------------------------------//
        sendCmd(0,steering_, fixThrottle);

    }

    // Apply hard brake
    while (true){
        sendCmd(8000, 0, 0);

        // Update odometry
        auto odo = Controller::getOdometry();
        double velocity = std::pow(std::pow(odo.twist.twist.linear.x,2)+ std::pow(odo.twist.twist.linear.x,2),0.5);

        rateLimit_.sleep();

        if(std::abs(velocity) < kMaxSpeedToProgressToNextGoal_) break;
    }

}




// ----------------------- VISUALIZATION -------------------------------------//

visualization_msgs::MarkerArray Ackerman::createConesMarker(std::vector<Point> goals){

    visualization_msgs::MarkerArray markerArray;
    for (auto pt : goals)
    {
        visualization_msgs::Marker marker;

        // We need to set the frame
        //  Set the frame ID and time stamp.
        marker.header.frame_id = "world";
        // single_marker_person.header.stamp = ros::Time();
        marker.header.stamp = ros::Time::now();

        // We set lifetime (it will dissapear in this many seconds)
        marker.lifetime = ros::Duration(0.2); // zero is forever

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "cones"; // This is namespace, markers can be in diofferent namespace
        marker.id = id_++;   // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

        // The marker type
        marker.type = visualization_msgs::Marker::CYLINDER;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pt.x;
        marker.pose.position.y = pt.y;
        marker.pose.position.z = 0;

        // Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 0.2m side, 0.5 height
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.5;

        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 1.0;

        markerArray.markers.push_back(marker);
    }
    return markerArray;
}

visualization_msgs::MarkerArray Ackerman::createGoalMarker(geometry_msgs::PoseArray goals){

    visualization_msgs::MarkerArray markerArray;
    // unsigned int ct=0;

    for (auto pt:goals.poses){
        visualization_msgs::Marker marker;

        //We need to set the frame
        // Set the frame ID and time stamp.
        marker.header.frame_id = "world";
        //single_marker_person.header.stamp = ros::Time();
        marker.header.stamp = ros::Time::now();

        //We set lifetime (it will dissapear in this many seconds)
        marker.lifetime = ros::Duration(0.2); //zero is forever

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "goals"; //This is namespace, markers can be in diofferent namespace
        marker.id = id_++; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

        // The marker type
        marker.type = visualization_msgs::Marker::CUBE;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pt.position.x;
        marker.pose.position.y = pt.position.y;
        marker.pose.position.z = pt.position.z;

        //Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1m side
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        //Let's send a marker with color (green for reachable, red for now)
        std_msgs::ColorRGBA color;
        color.a=0.5;//a is alpha - transparency 0.5 is 50%;
        color.r=0;
        color.g=1.0;
        color.b=0;

        marker.color = color;

        markerArray.markers.push_back(marker);
    }
    return markerArray;
}








/*------------------------------------------------------------------------------
* ------------------------------------------------------------------------------
*Author: Michael Wachl
*Year: 2018/2019
*Course: Technik autonomer Systeme
*Group: 5
*Univerity: Technische Universität München
* ------------------------------------------------------------------------------
------------------------------------------------------------------------------*/

//include header
#include "slalom.h"


/*******************************************************************************************
 * Call back to change parameter in runtime
 *******************************************************************************************/
void CSlalom::paramCallback(slalom::slalomConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure requested");

  //Update Parameter

  //Mode
  DYNAMIC_MODE_ = config.DYNAMIC_MODE_;
  GO_OUT_ = config.GO_OUT_;
  //trajectory parameter
  FIRST_DIST_ = config.FIRST_DIST_;
  CONE_DIST_ = config.CONE_DIST_;
  OFF_SET_Y_ = config.OFF_SET_Y_;
  OFF_SET_X_ = config.OFF_SET_X_;
  OVERSTEERING_ANGLE_ = config.OVERSTEERING_ANGLE_;
  //Parameter for cone detection
  X_LIM_MIN_ = config.X_LIM_MIN_;
  X_LIM_MAX_ = config.X_LIM_MAX_; //look ahead
  Y_LIM_MIN_ = config.Y_LIM_MIN_;
  Y_LIM_MAX_ = config.Y_LIM_MAX_;
  CLUSTER_TOLERANCE_ = config.CLUSTER_TOLERANCE_;
  CLUSTER_MIN_ = config.CLUSTER_MIN_;
  CLUSTER_MAX_ = config.CLUSTER_MAX_;

}


/*******************************************************************************************
 * callback function to receive inital position
 *******************************************************************************************/
void CSlalom::initPosCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    pos_res_ = true;
    //copy initial position
    start_pos_.pose.position.x = msg->pose.pose.position.x;
    start_pos_.pose.position.y = msg->pose.pose.position.y;
    start_pos_.pose.position.z = msg->pose.pose.position.z;
    start_pos_.pose.orientation.x = msg->pose.pose.orientation.x;
    start_pos_.pose.orientation.y = msg->pose.pose.orientation.y;
    start_pos_.pose.orientation.z = msg->pose.pose.orientation.z;
    start_pos_.pose.orientation.w = msg->pose.pose.orientation.w;
    ROS_INFO("Initial position received");

    cout<<"x: "<<start_pos_.pose.position.x<<"  y: "<<start_pos_.pose.position.y<<"  z: "<<start_pos_.pose.position.z<<std::endl;
    start_angle_ = tf::getYaw(start_pos_.pose.orientation);
    cout<<"angle: "<<start_angle_<<std::endl;

    if(!DYNAMIC_MODE_)
    {
    //Set slalom trajectory
    setTrajectory();

    //Publish trajectory for visualization only
    geometry_msgs::PoseStamped msg_pos;
    msg_pos.header.frame_id = "map";
    msg_pos.header.stamp = ros::Time::now();

    for(int i = 0; i < slalomtraject_.size(); ++i)
    {

        msg_pos.pose = slalomtraject_.at(i);
        pos_publisher_.publish(msg_pos);
        ROS_INFO("published");
        ros::Duration(1.0).sleep();

    }
    }
    else
         detect_first_cone_=true;
}


/*******************************************************************************************
 * callback function to receive pointcloud (to test cone detection)
 *******************************************************************************************/
void CSlalom::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //get current time
    current_time_ = cloud_msg->header.stamp;

    // Define containers
    pcl::PointCloud<PointT>::Ptr ptr_cloud(new pcl::PointCloud<PointT>); //PointCloud-XYZ's
    pcl::PointCloud<PointT>::Ptr ptr_cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PCLPointCloud2 point_cloud2;

    pcl::PassThrough<PointT> pass;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;

    //Convert Could2 to Cloud for additional filtering
    pcl_conversions::toPCL(*cloud_msg, point_cloud2);
    pcl::fromPCLPointCloud2(point_cloud2, *ptr_cloud);

    //Do passthrough cutoff "x" filtering
    pass.setInputCloud(ptr_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits(X_LIM_MIN_, X_LIM_MAX_);
    pass.filter(*ptr_cloud_filtered);

    // Build a passthrough filter to cutoff "y"
    pass.setInputCloud(ptr_cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(Y_LIM_MIN_, Y_LIM_MAX_); //check values in Matlab
    //pass.setFilterLimitsNegative (true);
    pass.filter(*ptr_cloud_filtered );

    //#####################################################################################
    //Do Clustering with Kd-Tree search to extract objects
    //#####################################################################################
    tree->setInputCloud(ptr_cloud_filtered);
    ec.setClusterTolerance(CLUSTER_TOLERANCE_); // 3cm
    ec.setMinClusterSize(CLUSTER_MIN_); //100
    ec.setMaxClusterSize(CLUSTER_MAX_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(ptr_cloud_filtered);
    ec.extract(cluster_indices);

    std::cout<<"PointCloud has: "<< cluster_indices.size() <<" clusters"<<std::endl;

    if(cluster_indices.size() == 1)
    {
         for (const auto it : cluster_indices)
         {
             //extract pointclouds
             pcl::ExtractIndices<PointT> extract;
             extract.setInputCloud (ptr_cloud_filtered);
             extract.setIndices (boost::make_shared<const pcl::PointIndices> (it));
             pcl::PointCloud<PointT>::Ptr ptr_currentclustercloud (new pcl::PointCloud<PointT>);
             extract.filter(*ptr_currentclustercloud);

             //compute centroid
             Eigen::Vector4f centroid;
             const pcl::PointCloud<PointT> cone_cloud = *ptr_currentclustercloud;
             if(pcl::compute3DCentroid(cone_cloud, centroid) != 0)
                  saveCone(centroid);

         }
    }

}


/*******************************************************************************************
 * callback function to receive front laserscan
 *******************************************************************************************/
void CSlalom::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PCLPointCloud2 point_cloud2;
    try{
    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud_msg, tfListener_);

    //get current time
    current_time_ = scan->header.stamp;

    // Define containers
    pcl::PointCloud<PointT>::Ptr ptr_cloud(new pcl::PointCloud<PointT>); //PointCloud-XYZ's
    pcl::PointCloud<PointT>::Ptr ptr_cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PCLPointCloud2 point_cloud2;
    pcl::PassThrough<PointT> pass;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;

    // Convert msg to PCL data type
    pcl_conversions::toPCL(cloud_msg, point_cloud2);
    //Convert Could2 to Cloud for additional filtering
    pcl::fromPCLPointCloud2(point_cloud2, *ptr_cloud);

    //Convert Could2 to Cloud for additional filtering
    pcl::fromPCLPointCloud2(point_cloud2, *ptr_cloud);

    /*Visiulize
     * int i = pcl::io::loadPCDFile("/home/miguel/catkin_ws/slalom_pcd/1.pcd", *cloud);
     *
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPointCloud<PointT> (ptr_cloud_filtered ,"sample cloud");
        viewer->spinOnce (100);
        ros::Duration(3.0).sleep();
    */

    //Do passthrough cutoff "x" filtering
    pass.setInputCloud(ptr_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits(X_LIM_MIN_, X_LIM_MAX_);
    pass.filter(*ptr_cloud_filtered);

    // Build a passthrough filter to cutoff "y"
    pass.setInputCloud(ptr_cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(Y_LIM_MIN_, Y_LIM_MAX_); //check values in Matlab
    //pass.setFilterLimitsNegative (true);
    pass.filter(*ptr_cloud_filtered );

    //#####################################################################################
    //Do Clustering with Kd-Tree search to extract objects
    //#####################################################################################
    tree->setInputCloud(ptr_cloud_filtered);
    ec.setClusterTolerance(CLUSTER_TOLERANCE_); // 3cm
    ec.setMinClusterSize(CLUSTER_MIN_); //100
    ec.setMaxClusterSize(CLUSTER_MAX_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(ptr_cloud_filtered);
    ec.extract(cluster_indices);

    //std::cout<<"PointCloud has: "<< cluster_indices.size() <<" clusters"<<std::endl;

    if(cluster_indices.size() == 1)
    {
         for (const auto it : cluster_indices)
         {
             //extract pointclouds
             pcl::ExtractIndices<PointT> extract;
             extract.setInputCloud (ptr_cloud_filtered);
             extract.setIndices (boost::make_shared<const pcl::PointIndices> (it));
             pcl::PointCloud<PointT>::Ptr ptr_currentclustercloud (new pcl::PointCloud<PointT>);
             extract.filter(*ptr_currentclustercloud);

             //compute centroid
             Eigen::Vector4f centroid;
             const pcl::PointCloud<PointT> cone_cloud = *ptr_currentclustercloud;

             if(pcl::compute3DCentroid(cone_cloud, centroid) != 0 && angleInTolerance(centroid))
             {
                   saveCone(centroid);
		   break;
             }
         }
    }

    }
    catch(tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    //publish filterd pointcloud
    //point_cloud_publisher_.publish(cloud);

}

/*******************************************************************************************
 * function to check if detected cone angle is too heigh
 *******************************************************************************************/
bool CSlalom::angleInTolerance(Eigen::Vector4f centroid)
{
    //next position
    //check which cone and save cone position
    bool transform_success= true;
    geometry_msgs::PointStamped current_pose;
    geometry_msgs::PointStamped current_point_map;
    geometry_msgs::PoseStamped current_pose_map;
    geometry_msgs::PoseStamped next_pose;
    current_pose.header.frame_id = "base_link";
    current_pose.header.stamp = current_time_;
    current_pose.point.x = centroid(0,0);
    current_pose.point.y = centroid(1,0);
    current_pose.point.z = centroid(2,0);
    //Transfor to map koordinates
     current_point_map = transformPoint(current_pose);
     current_pose_map.pose.position.x = current_point_map.point.x;
     current_pose_map.pose.position.y = current_point_map.point.y;
     current_pose_map.pose.position.z = current_point_map.point.z;
     //check if success
     if (current_pose_map.pose.position.x == 0 && current_pose_map.pose.position.y == 0 && current_pose_map.pose.position.z == 0)
         transform_success = false;
    if(detect_first_cone_ && transform_success)
    {
     cout<<"Check first angle"<<endl;
     //check angle
     next_pose = calculateAngleBetweenPoints(start_pos_, current_pose_map);
     double next_angle = tf::getYaw(next_pose.pose.orientation);
     if (abs(start_angle_-next_angle)<0.4) //~23 grad
         return true;
    }
    else if(detect_second_cone_ && transform_success)
    {
     cout<<"Check second angle"<<endl;
     //check angle
     next_pose = calculateAngleBetweenPoints(first_cone_, current_pose_map);
     double next_angle = tf::getYaw(next_pose.pose.orientation);
     if (abs(tf::getYaw(calculateAngleBetweenPoints(start_pos_,first_cone_).pose.orientation)-next_angle)<0.4) //~23 grad
         return true;
    }

    else if(detect_third_cone_ && transform_success)
    {
      cout<<"Check thrid angle"<<endl;
      //check angle
       next_pose = calculateAngleBetweenPoints(second_cone_, current_pose_map);
       double next_angle = tf::getYaw(next_pose.pose.orientation);

     if (abs(tf::getYaw(calculateAngleBetweenPoints(first_cone_,second_cone_).pose.orientation)-next_angle)<0.4) //~23 grad
         return true;
    }
    else if(detect_fourth_cone_ && transform_success)
    {
     cout<<"Check last angle"<<endl;
     //check angle
        next_pose = calculateAngleBetweenPoints(third_cone_, current_pose_map);
        double next_angle = tf::getYaw(next_pose.pose.orientation);
     if (abs(tf::getYaw(calculateAngleBetweenPoints(second_cone_,third_cone_).pose.orientation)-next_angle)<0.4) //~23 grad
         return true;
    }
    //else return false
    return false;
}


/*******************************************************************************************
 * function to save cone positions
 *******************************************************************************************/
void CSlalom::saveCone(Eigen::Vector4f centroid){

    float x = centroid(0,0), y = centroid(1,0), z = centroid(2,0);
    bool transform_success = true;

    //check which cone and save cone position
    geometry_msgs::PointStamped current_pose;
    geometry_msgs::PointStamped current_pose_map;
    current_pose.header.frame_id = "base_link";
    current_pose.header.stamp = current_time_;
    current_pose.point.x = x;
    current_pose.point.y = y;
    current_pose.point.z = z;
   //Transfor to map koordinates
    current_pose_map = transformPoint(current_pose);
   // cout<<"Map coordin: x: "<< current_pose_map.point.x <<"  y: " <<current_pose_map.point.y<<"  z: "<<current_pose_map.point.z<<endl;

    //check if success
    if (current_pose_map.point.x == 0 && current_pose_map.point.y == 0 && current_pose_map.point.z == 0)
        transform_success = false;


    //first cone
    if(detect_first_cone_ && transform_success)
    {first_cone_.header.frame_id = "map";
     first_cone_.header.stamp = current_time_;
     first_cone_.pose.position.x = current_pose_map.point.x;
     first_cone_.pose.position.y = current_pose_map.point.y;
     first_cone_.pose.position.z = current_pose_map.point.z;
     first_cone_.pose.orientation = start_pos_.pose.orientation;
     //if(  )
     //{
     first_cone_detected_ = true;
     detect_first_cone_ = false;
     //}
     //publish first cone pos
     geometry_msgs::PoseStamped msg_pos;
     msg_pos.header = first_cone_.header;
     msg_pos.pose= first_cone_.pose;
     pos_publisher_.publish(msg_pos);
     cout<<"First cone lokal x: "<<x<<"  y: "<<y<<endl;
    }
    //second cone
    else if(detect_second_cone_ && transform_success)
    {
     second_cone_.header.frame_id = "map";
     second_cone_.header.stamp = current_time_;
     second_cone_.pose.position.x = current_pose_map.point.x;
     second_cone_.pose.position.y = current_pose_map.point.y;
     second_cone_.pose.position.z = current_pose_map.point.z;
     second_cone_.pose.orientation = start_pos_.pose.orientation;
     second_cone_detected_ = true;
     detect_second_cone_ = false;
     //publish second cone pos
     geometry_msgs::PoseStamped msg_pos;
     msg_pos.header = second_cone_.header;
     msg_pos.pose= second_cone_.pose;
     pos_publisher_.publish(msg_pos);
     cout<<"First cone lokal x: "<<x<<"  y: "<<y<<endl;
    }
    //third cone
    else if(detect_third_cone_ && transform_success)
    {
     third_cone_.header.frame_id = "map";
     third_cone_.header.stamp = current_time_;
     third_cone_.pose.position.x = current_pose_map.point.x;
     third_cone_.pose.position.y = current_pose_map.point.y;
     third_cone_.pose.position.z = current_pose_map.point.z;
     third_cone_.pose.orientation = start_pos_.pose.orientation;
     third_cone_detected_ = true;
     detect_third_cone_ = false;
     //publish second cone pos
     geometry_msgs::PoseStamped msg_pos;
     msg_pos.header = third_cone_.header;
     msg_pos.pose= third_cone_.pose;
     pos_publisher_.publish(msg_pos);
     cout<<"Third cone lokal x: "<<x<<"  y: "<<y<<endl;
    }
    //fourth cone
    else if(detect_fourth_cone_ && transform_success)
    {
     fourth_cone_.header.frame_id = "map";
     fourth_cone_.header.stamp = current_time_;
     fourth_cone_.pose.position.x = current_pose_map.point.x;
     fourth_cone_.pose.position.y = current_pose_map.point.y;
     fourth_cone_.pose.position.z = current_pose_map.point.z;
     fourth_cone_.pose.orientation = start_pos_.pose.orientation;
     fourth_cone_detected_ = true;
     detect_fourth_cone_ = false;
     //publish second cone pos
     geometry_msgs::PoseStamped msg_pos;
     msg_pos.header = fourth_cone_.header;
     msg_pos.pose= fourth_cone_.pose;
     pos_publisher_.publish(msg_pos);
     cout<<"Fourth cone lokal x: "<<x<<"  y: "<<y<<endl;
    }
}


/*******************************************************************************************
 * function to transform from base_link coordinates to map coordinates
 *******************************************************************************************/
geometry_msgs::PointStamped CSlalom::transformPoint(geometry_msgs::PointStamped pose_local)
{
    //tf::StampedTransform transform;
    geometry_msgs::PointStamped pose_map;
    try
    {
      //tfListener_.waitForTransform("/map", "/base_link", current_time_, ros::Duration(2.0));
      //tfListener_.lookupTransform("/map", "/base_link",  current_time_, transform);
      tfListener_.transformPoint("/map", pose_local, pose_map);

      return pose_map;
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"map\": %s", ex.what());
    }
}


/*******************************************************************************************
 * function to calculate the angle between two points
 ********************************************************************************************/
geometry_msgs::PoseStamped CSlalom::calculateAngleBetweenPoints(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
{
    //calculate angle between two points
    geometry_msgs::PoseStamped return_pose;
    cout<<"point1: "<<pose1.pose.position.x <<" "<< pose1.pose.position.y<<endl;
    cout<<"point2: "<<pose2.pose.position.x <<" "<< pose2.pose.position.y<<endl;
    cout<<"Ursprung: "<<(pose2.pose.position.x-pose1.pose.position.x) <<" "<< (pose2.pose.position.y -pose1.pose.position.y)<<endl;
    double angle_between_points;
    if(pose2.pose.position.x-pose1.pose.position.x>=0)
        angle_between_points = atan((pose2.pose.position.y - pose1.pose.position.y)/(pose2.pose.position.x - pose1.pose.position.x));
    else
        angle_between_points = atan((pose2.pose.position.y - pose1.pose.position.y)/(pose2.pose.position.x - pose1.pose.position.x))+PI;

    //Oversteer for a smoother trajectory
    if(first_cone_detected_ || third_cone_detected_)
        angle_between_points -= (OVERSTEERING_ANGLE_*PI/180);
    else if(second_cone_detected_)
         angle_between_points += (OVERSTEERING_ANGLE_*PI/180);

    tf::Quaternion quat;
    //angle_between_points = -PI;
    quat = tf::createQuaternionFromYaw(angle_between_points);
    return_pose.pose.orientation.x = quat[0];
    return_pose.pose.orientation.y = quat[1];
    return_pose.pose.orientation.z = quat[2];
    return_pose.pose.orientation.w = quat[3];

    double return_angle = tf::getYaw(return_pose.pose.orientation);
    cout<<"Calculated angle :"<<angle_between_points<<"  returned angle: "<<return_angle<<endl;
   return return_pose;
}

/*******************************************************************************************
 * function to set trajectory for the static option
 *******************************************************************************************/
void CSlalom::setTrajectory(){
    //the map resolution is in meter/pixel: 0,05

    //Oversteer for a smoother trajectory
    double angle_leftcones,angle_rightcone;
    tf::Quaternion quat_l,quat_r;
    angle_leftcones = start_angle_ - (OVERSTEERING_ANGLE_*PI/180);
    angle_rightcone = start_angle_ + (OVERSTEERING_ANGLE_*PI/180);
    quat_l = tf::createQuaternionFromYaw(angle_leftcones);
    quat_r = tf::createQuaternionFromYaw(angle_rightcone);
    //drive left to first cone
    geometry_msgs::Pose leftfirst;
    leftfirst = start_pos_.pose;
    leftfirst.orientation.x = quat_l[0];
    leftfirst.orientation.y = quat_l[1];
    leftfirst.orientation.z = quat_l[2];
    leftfirst.orientation.w = quat_l[3];
    leftfirst.position.x = leftfirst.position.x + FIRST_DIST_*cos(start_angle_) - OFF_SET_X_*cos(start_angle_) - OFF_SET_Y_*sin(start_angle_);
    leftfirst.position.y = leftfirst.position.y + FIRST_DIST_*sin(start_angle_) - OFF_SET_X_*sin(start_angle_) - OFF_SET_Y_*(-cos(start_angle_));
    slalomtraject_.push_back(leftfirst);

    //drive right to second cone
    geometry_msgs::Pose rightsecond;
    rightsecond = start_pos_.pose;
    rightsecond.orientation.x = quat_r[0];
    rightsecond.orientation.y = quat_r[1];
    rightsecond.orientation.z = quat_r[2];
    rightsecond.orientation.w = quat_r[3];
    rightsecond.position.x = rightsecond.position.x + (CONE_DIST_+FIRST_DIST_)*cos(start_angle_) - OFF_SET_X_*cos(start_angle_) + OFF_SET_Y_*sin(start_angle_);
    rightsecond.position.y = rightsecond.position.y + (CONE_DIST_+FIRST_DIST_)*sin(start_angle_) - OFF_SET_X_*sin(start_angle_) + OFF_SET_Y_*(-cos(start_angle_));
    slalomtraject_.push_back(rightsecond);

    //drive left to third cone
    geometry_msgs::Pose leftthird;
    leftthird = start_pos_.pose;
    leftthird.orientation = leftfirst.orientation;
    leftthird.position.x = leftthird.position.x + (2*CONE_DIST_+FIRST_DIST_)*cos(start_angle_) - OFF_SET_X_*cos(start_angle_) - OFF_SET_Y_*sin(start_angle_);
    leftthird.position.y = leftthird.position.y + (2*CONE_DIST_+FIRST_DIST_)*sin(start_angle_) - OFF_SET_X_*sin(start_angle_) - OFF_SET_Y_*(-cos(start_angle_));
    slalomtraject_.push_back(leftthird);

    //drive right to fourth cone
    geometry_msgs::Pose rightfourth;
    rightfourth = start_pos_.pose;
    rightfourth.position.x = rightfourth.position.x + (3*CONE_DIST_+FIRST_DIST_)*cos(start_angle_) - OFF_SET_X_*cos(start_angle_) + OFF_SET_Y_*sin(start_angle_);
    rightfourth.position.y = rightfourth.position.y + (3*CONE_DIST_+FIRST_DIST_)*sin(start_angle_) - OFF_SET_X_*sin(start_angle_) + OFF_SET_Y_*(-cos(start_angle_));
    slalomtraject_.push_back(rightfourth);

    //drive out
    if(GO_OUT_)
    {
    geometry_msgs::Pose goout;
    goout = start_pos_.pose;
    goout.position.x = goout.position.x + (4*CONE_DIST_+FIRST_DIST_)*cos(start_angle_);
    goout.position.y = goout.position.y + (4*CONE_DIST_+FIRST_DIST_)*sin(start_angle_);
    slalomtraject_.push_back(goout);
    }
}


/*******************************************************************************************
 * function to calculate the current gaol
 *******************************************************************************************/
geometry_msgs::PoseStamped CSlalom::calculateGoal()
{
    geometry_msgs::PoseStamped goal;
    double current_angle;
    if(first_cone_detected_)
    {
      //calculate goal angle from start and first cone
      goal = calculateAngleBetweenPoints(start_pos_,first_cone_);
      current_angle = tf::getYaw(goal.pose.orientation);
      //set x and y of goal, left from cone
      goal.pose.position.x = first_cone_.pose.position.x - OFF_SET_X_*cos(current_angle) - OFF_SET_Y_*sin(current_angle);
      goal.pose.position.y = first_cone_.pose.position.y - OFF_SET_X_*sin(current_angle) - OFF_SET_Y_*(-cos(current_angle));
      goal.pose.position.z = 0;
      cout<<"First goal: x: "<<goal.pose.position.x<<"  y: "<<goal.pose.position.y<<endl;
      ROS_INFO("First goal calculated");
      return goal;
    }
    else if(second_cone_detected_)
    {
      //calculate goal angle from first cone and second cone
      goal = calculateAngleBetweenPoints(first_cone_,second_cone_);
      current_angle = tf::getYaw(goal.pose.orientation);
      //set x and y of goal, left from cone
      goal.pose.position.x = second_cone_.pose.position.x - OFF_SET_X_*cos(current_angle) + OFF_SET_Y_*sin(current_angle);
      goal.pose.position.y = second_cone_.pose.position.y - OFF_SET_X_*sin(current_angle) + OFF_SET_Y_*(-cos(current_angle));
      goal.pose.position.z = 0;
      ROS_INFO("Second goal calculated");
      return goal;
    }
    else if(third_cone_detected_)
    {
      //calculate goal angle from second cone and third cone
      goal = calculateAngleBetweenPoints(second_cone_,third_cone_);
      current_angle = tf::getYaw(goal.pose.orientation);
      //set x and y of goal, left from cone
      goal.pose.position.x = third_cone_.pose.position.x - OFF_SET_X_*cos(current_angle) - OFF_SET_Y_*sin(current_angle);
      goal.pose.position.y = third_cone_.pose.position.y - OFF_SET_X_*sin(current_angle) - OFF_SET_Y_*(-cos(current_angle));
      goal.pose.position.z = 0;
      ROS_INFO("Third goal calculated");
      return goal;
    }
    else if(fourth_cone_detected_)
    {
      //calculate goal angle from third cone and fourth cone
      goal = calculateAngleBetweenPoints(third_cone_,fourth_cone_);
      current_angle = tf::getYaw(goal.pose.orientation);
      //set x and y of goal, left from cone
      goal.pose.position.x = fourth_cone_.pose.position.x - OFF_SET_X_*cos(current_angle) + OFF_SET_Y_*sin(current_angle);
      goal.pose.position.y = fourth_cone_.pose.position.y - OFF_SET_X_*sin(current_angle) + OFF_SET_Y_*(-cos(current_angle));
      goal.pose.position.z = 0;
      ROS_INFO("Fourth goal calculated");
      return goal;
    }
    else if(go_out_)
    {
      //calculate goal angle from start and fourth cone
      goal = calculateAngleBetweenPoints(fourth_cone_, start_pos_);
      current_angle = tf::getYaw(goal.pose.orientation);
      //set x and y of goal, left from cone
      goal.pose.position.x = fourth_cone_.pose.position.x + FIRST_DIST_*cos(current_angle);;
      goal.pose.position.y = fourth_cone_.pose.position.y + FIRST_DIST_*sin(current_angle);;
      goal.pose.position.z = 0;
    }
}


/**
 * Callback function for succeeded
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}


/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
}


/*******************************************************************************************
 * feedback function to distance (not in use)
 *******************************************************************************************/
void CSlalom::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    //ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
    //if()
    //distance_ = sqrtf(pow(feedback->base_position.pose.position.x - start_pos_.pose.position.x, 2.0 ) + pow(feedback->base_position.pose.position.y - start_pos_.pose.position.y, 2.0));
    //cout<<"Distance: "<< distance_<<endl;
}



/*******************************************************************************************
 * Main function
 *******************************************************************************************/
int main(int argc, char** argv){

    ros::init(argc, argv, "slalom"); // init and set name
    ros::NodeHandle nh;

    CSlalom slalom(nh);

    MoveBaseClient ac("move_base", true); // action client to spin a thread by default

    //init parameter server
    dynamic_reconfigure::Server<slalom::slalomConfig> server;
    dynamic_reconfigure::Server<slalom::slalomConfig>::CallbackType f;
    f = boost::bind(&CSlalom::paramCallback,&slalom, _1, _2);
    server.setCallback(f);


   while (!ac.waitForServer(ros::Duration(5.0))) 
   { // wait for the action server to come up
      ROS_INFO("Waiting for the move_base action server to come up");
   }


    ROS_INFO("Wait for init pos");

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        //drive static trajectory
        if(!slalom.DYNAMIC_MODE_ && slalom.pos_res_ )
        { ROS_INFO("Static Mode");
          slalom.pos_res_ = false;
          move_base_msgs::MoveBaseGoal goal;
          goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

          // loop over all goal points, point by point
          for(int i = 0; i < slalom.slalomtraject_.size(); ++i)
          {
                goal.target_pose.header.stamp = ros::Time::now(); // set current time
                goal.target_pose.pose = slalom.slalomtraject_.at(i);
                ROS_INFO("Sending goal");
                ac.sendGoal(goal, &doneCb, &activeCb, boost::bind(&CSlalom::feedbackCb, &slalom, _1)); // send goal and register callback handler
                ac.waitForResult(); // wait for goal result

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        ROS_INFO("The base moved to %d goal", i);

                else
                        ROS_INFO("The base failed to move to %d goal for some reason", i);

          }
          //empty trajectory
          slalom.slalomtraject_.erase (slalom.slalomtraject_.begin(),slalom.slalomtraject_.end());
          slalom.pos_res_ = false;

         }

         //drive dynamic trajectory
         if(slalom.DYNAMIC_MODE_ && slalom.pos_res_ )
         {
            //check if a cone is detected
            if(slalom.first_cone_detected_ || slalom.second_cone_detected_ || slalom.third_cone_detected_ ||
                    slalom.fourth_cone_detected_ || slalom.go_out_)
            {
              ROS_INFO("Dynamic Mode, calculation next goal");

              //calculate next goal
              geometry_msgs::PoseStamped next_goal = slalom.calculateGoal();
              move_base_msgs::MoveBaseGoal goal;
              goal.target_pose.pose = next_goal.pose;

              goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates
              goal.target_pose.header.stamp = ros::Time::now(); // set current time

              ROS_INFO("Sending goal");
              ac.sendGoal(goal, &doneCb, &activeCb, boost::bind(&CSlalom::feedbackCb, &slalom, _1)); // send goal and register callback handler
              ac.waitForResult(); // wait for goal result

              if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && slalom.first_cone_detected_)
              {
                  ROS_INFO("The base moved next to first cone");
                  slalom.detect_second_cone_= true;
                  slalom.first_cone_detected_ = false;
              }
              else if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && slalom.second_cone_detected_)
              {
                  ROS_INFO("The base moved next to second cone");
                  slalom.detect_third_cone_= true;
                  slalom.second_cone_detected_ = false;
              }
              else if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && slalom.third_cone_detected_)
              {
                  ROS_INFO("The base moved next to third cone");
                  slalom.detect_fourth_cone_= true;
                  slalom.third_cone_detected_ = false;
              }
              else if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && slalom.fourth_cone_detected_)
              {
                  ROS_INFO("The base moved next to fourth cone");
                  slalom.fourth_cone_detected_ = false;
                  //if goout not selected
                  if(slalom.GO_OUT_)
                   {
		    ROS_INFO("---------------Trajectory finished---------------");
                    slalom.go_out_ = true;
		}

              }
              else if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && slalom.go_out_)
              {
                  ROS_INFO("The base moved behind fourth cone");
                  slalom.go_out_= false;
                  ROS_INFO("---------------Trajectory finished---------------");
                  //terminate node
                  return 0;
              }
              else
                      ROS_INFO("The base failed to move to goal for some reason");

           }
       }

    ros::spinOnce();
    loop_rate.sleep();

    }
    return 0;
}

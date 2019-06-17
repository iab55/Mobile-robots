#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/ColorRGBA.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <math.h>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


struct Cluster
{
  double x;
  double y;
  double width;
  int start_index;

  bool operator < (const Cluster& c) const
        {
        return (width > c.width);
        }
};

struct Landmark
{
  double x;
  double y;
  double radius;
};

struct Pose
{
  double x;
  double y;
  double th;
};

class Localization
{
  public:
    Localization()
    {
      this->laser_sub  = n.subscribe("laserscan", 1, &Localization::laser_callback, this);
      this->markers_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 1);
      this->pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose",1);

      this->red.r=1.0;
      this->red.a=0.5;
      this->green.g=1.0;
      this->green.a=0.5;
      this->blue.b=1.0;
      this->blue.a=0.5;

      //transform from platform to sensor
      this->static_x = 0.00; //[m]
      this->static_y = 0.00; //[m]
      this->static_z = 0.44; //[m]//ignore. Used for displaying.
      this->static_th= 0.0; //[radians]

      this->landmarks.resize(3);
      //Coordinates associated to localization.world cylinders positions and sizes
      this->landmarks[0].x    = -2.0;
      this->landmarks[0].y    =  2.0;
      this->landmarks[0].radius=  0.5;
      this->landmarks[1].x    =  0.0;
      this->landmarks[1].y    = -1.0;
      this->landmarks[1].radius=  0.3;
      this->landmarks[2].x    =  1.0;
      this->landmarks[2].y    =  1.0;
      this->landmarks[2].radius=  0.15;

      this->received_scan=false;
      this->rate=2; //HZ
    }

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      this->scan_msg = *msg;
      this->received_scan=true;
    }

    std::vector<Cluster> getClusters()
    {
      std::vector<Cluster> clusters;

      int size=this->scan_msg.ranges.size();
      std::vector<double> x;
      std::vector<double> y;

      //TODO 1 (1.1 to 1.5)
      //Fill clusters.

      //Polar to Cartesian (x,y)
      for(unsigned int i=0; i<size; i++)
      {
        if(!isinf(this->scan_msg.ranges[i]))
        {
          //TODO 1.1 BEGIN
          //Polar to Cartesian (x,y). Same as in session 2
    //      x = this->scan_msg.ranges[i]*cos(this->scan_msg.angle_min+i*this->scan_msg.angle_increment);
    //      y = this->scan_msg.ranges[i]*sin(this->scan_msg.angle_min+i*this->scan_msg.angle_increment);
          //Add element by element
          x.push_back(this->scan_msg.ranges[i]*cos(this->scan_msg.angle_min+i*this->scan_msg.angle_increment));
          y.push_back(this->scan_msg.ranges[i]*sin(this->scan_msg.angle_min+i*this->scan_msg.angle_increment));
          //TODO 1.1 END
        }
      }

      //Group points and fill clusters (x,y,width)

      //Add 0,0 point to close last cluster
      x.push_back(0.0);
      y.push_back(0.0);

      size=x.size();
      double jumpdist=0.2;

      //First cluster, starts on first point (0)
      clusters.resize(1);
      clusters[0].start_index=0;

      int j=0;
      double sum_dist;
      double group_x;
      double group_y;
      for(unsigned int i=1; i<size; i++)
      {
        double dist=0.0;
        //TODO 1.2 BEGIN
        //Euclidean distance between current point and previous point
        j+=1;
        dist = sqrt((x[i]-x[i-1])*(x[i]-x[i-1])+(y[i]-y[i-1])*(y[i]-y[i-1]));
        sum_dist+=dist;
        group_x+=x[i-1];
        group_y+=y[i-1];

        //TODO 1.2 END
        if(dist>jumpdist)
        {
          //TODO 1.3 BEGIN
          // Compute and fill last cluster x and y

          clusters.back().x = group_x/j;
          clusters.back().y = group_y/j;
          group_x=0;
          group_y=0;
          j=0;

          //TODO 1.3 END

          double width=0.0;
          //TODO 1.4 BEGIN
          // Compute and fill last cluster width

          width = sum_dist/M_PI;
          clusters.back().width = width;
          sum_dist=0;

          //TODO 1.4 END

          //Start new cluster
          Cluster c;
          c.start_index=i;
          clusters.push_back(c);
        }
      }

      //TODO 1.5 BEGIN
      // Clean clusters down to having 3 and/or match clusters with the 3 landmarks

      // A cluster on the back of the robot can be divided by the scan begin/end.
      // You can try to detect that and join them in one

      // Match clusters with our 3 landmarks.
      // You can reduce the number of clusters to 3, before matching them with landmarks
      // Or you can directly match them (cluster.width <--> landmark.radius)


      ////For example, this simply sorts clusters by width (max...min) and then deletes the last(small) ones until having only 3
      std::sort(clusters.begin(), clusters.end());
      while(clusters.size()>3)
        clusters.pop_back();

      //TODO 1.5 END

      for(unsigned int i=0; i<clusters.size(); i++)
        ROS_INFO("cluster[%d] x,y,width=%f, %f, %f", i, clusters[i].x, clusters[i].y, clusters[i].width );
      return clusters;
    }

    Pose compute_pose()
    {
      Pose p;
      double scale;

      if(this->clusters.size() != this->landmarks.size() )
        ROS_WARN("Warning, your clusters number (%lu) is different from the landmarks number (%lu)",this->clusters.size(), this->landmarks.size());

      //TODO 2 (2.1 to 2.3)

      //Some matrix and vector examples of usage (using Eigen library):
      /*std::cout << "Eigen examples START" << std::endl;
      int rows = 2;
      int cols = 4;
      Eigen::MatrixXd M(rows,cols);
      Eigen::VectorXd V(cols);
      V << 2.0,4.4,6.33,8.21;
      M.row(0) = V;
      M.row(1) << 2, 20, 200, 2000;
      std::cout << "M: " << std::endl;
      std::cout <<  M    << std::endl;
      std::cout << "M': " << std::endl;
      std::cout <<  M.transpose()    << std::endl;

      Eigen::MatrixXd N(2,2);
      N << 1, 2, 3, 4;
      std::cout << "N: " << std::endl;
      std::cout <<  N    << std::endl;
      std::cout << "N^-1': " << std::endl;
      std::cout <<  N.inverse() << std::endl;
      std::cout << "Eigen examples END" << std::endl;*/


      //Following the naming from the theory class slides:
      //TODO 2.1 BEGIN
      //Create and fill A matrix (landmarks)
      int rowsA = 6;
      int colsA = 4;
      Eigen::MatrixXd A(rowsA,colsA);
      A.row(0)<< -2,2,1,0;
      A.row(1)<< 2,2,0,1;
      A.row(2)<< 0,-1,1,0;
      A.row(3)<< -1,0,0,1;
      A.row(4)<< 1,1,1,0;
      A.row(5)<< 1,-1,0,1;


      //TODO 2.1 END
      //This prints A on screen
      std::cout << "A: \n " << A << std::endl;

      //TODO 2.2 BEGIN
      //Create and Fill B matrix/vector (clusters)
      Eigen::VectorXd B(6);
      B<< clusters[0].x,clusters[0].y,clusters[1].x,clusters[1].y,clusters[2].x,clusters[2].y;
      //TODO 2.2 END
      //This prints B on screen
      std::cout << "B: \n" << B << std::endl;

      Eigen::VectorXd X = Eigen::VectorXd::Zero(4);
      //TODO 2.3: BEGIN
      //Solve the system for X

      X=(A.transpose()*A).inverse()*A.transpose()*B;


      //TODO 2.3 END
      //This prints X on screen
      std::cout << "X: \n" << X << std::endl;

      p.x = X.coeff(2, 0);
      p.y = X.coeff(3, 0);
      p.th= atan2( - X.coeff(1, 0) , X.coeff(0, 0) );
      scale = sqrt(X.coeff(0, 0)*X.coeff(0, 0)+X.coeff(1, 0)*X.coeff(1, 0));

      ROS_INFO("pose.x,pose.y= %f, %f.   pose.th, scale= %f, %f",p.x, p.y, p.th, scale);
      return p;
    }

    void loop()
    {
      if(this->received_scan)
      {
        this->received_scan=false;

        this->clusters = this->getClusters();
        this->publish_markers(this->clusters,"laser_link","cluster_pos",-this->static_z,this->red,this->scan_msg.header.stamp);

        this->pose = this->compute_pose();
        this->publish_pose(this->pose.x, this->pose.y, this->pose.th, "laser_link", -this->static_z, this->red, this->scan_msg.header.stamp );

        ROS_INFO("---");
      }
    }

    void publish_pose(double x, double y, double th, std::string frame_id, double z, std_msgs::ColorRGBA color, ros::Time t)
    {
      this->pose_msg.header.stamp = t;
      this->pose_msg.header.frame_id = frame_id;
      this->pose_msg.pose.position.x = x;
      this->pose_msg.pose.position.y = y;
      this->pose_msg.pose.position.z = z;
      this->pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(th);
      this->pose_pub.publish(this->pose_msg);
    }

    void publish_markers(std::vector<Cluster> c, std::string frame_id, std::string name_space,double z, std_msgs::ColorRGBA color, ros::Time t)
    {
      this->markers_msg.markers.clear();
      for(unsigned int i=0; i<c.size(); i++)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp     = t;
        marker.ns               = name_space;
        marker.id               = i;
        //marker.id             = this->markers_msg.markers.size();
        marker.type             = visualization_msgs::Marker::CYLINDER;
        marker.action           = visualization_msgs::Marker::ADD;

        double radius=c[i].width;
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.scale.z = 1.0;

        marker.pose.position.x = c[i].x;
        marker.pose.position.y = c[i].y;
        marker.pose.position.z = z + marker.scale.z/2.0 ;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        //marker.color.r = 1.0;
        //marker.color.g = 0.0;
        //marker.color.b = 0.0;
        //marker.color.a = 0.5;
        marker.color=color;
        marker.lifetime = ros::Duration(1.0/this->rate);

        //only publish if not NaNs
        if(c[i].x==c[i].x && c[i].y == c[i].y)
        {
          this->markers_msg.markers.push_back(marker);
        }
      }
      this->markers_pub.publish(this->markers_msg);
    }

    double getRate()
    {
      return this->rate;
    }

  private:
    ros::NodeHandle n;
    double rate;
    ros::Subscriber laser_sub;
    sensor_msgs::LaserScan scan_msg;
    bool received_scan;
    ros::Publisher markers_pub;
    visualization_msgs::MarkerArray markers_msg;
    ros::Publisher pose_pub;
    geometry_msgs::PoseStamped pose_msg;
    std_msgs::ColorRGBA red;
    std_msgs::ColorRGBA green;
    std_msgs::ColorRGBA blue;
    double static_x, static_y, static_z, static_th;
    std::vector<Landmark> landmarks;
    std::vector<Cluster> clusters;
    Pose pose;
    tf::TransformListener tf_listener;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization");
  Localization my_localization;
  ros::Rate loop_rate(my_localization.getRate());
  while (ros::ok())
  {
    my_localization.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

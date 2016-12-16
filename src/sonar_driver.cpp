#include <time.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "sonar_driver");

  ros::NodeHandle n;
  ros::Publisher sonar_pub = n.advertise<sensor_msgs::Range>("sonar", 50);

  ros::Rate r(240.0);
  while(n.ok()){
    //generate some fake data for our sonar
    srand(time(0));
    double range = 0.1524 + ((float)rand()/(float)(RAND_MAX)) * 6.2992;

    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::Range msg;
    msg.header.stamp = scan_time;
    msg.header.frame_id = "sonar_frame";
    msg.radiation_type = 0;
    msg.field_of_view = 0;
    msg.min_range = 0.15;
    msg.max_range = 6.45;
    msg.range = range;

    sonar_pub.publish(msg);
    r.sleep();
  }
}

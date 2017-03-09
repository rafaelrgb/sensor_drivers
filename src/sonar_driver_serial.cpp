#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include "wiringSerial.h"

int main(int argc, char** argv){

  // Initialize ROS
  ros::init(argc, argv, "sonar_driver");

  // Declare the sonar publisher and inform the master that it will be publishing messages on topic "/sonar"
  ros::NodeHandle n;
  ros::Publisher sonar_pub = n.advertise<sensor_msgs::Range>("sonar", 50);

  // Use wiringPi to connect to the serial port
  int fd ;
  if ((fd = serialOpen ("/dev/ttyACM0", 9600)) < 0)
  {
    ROS_INFO("Unable to open serial device.\n");
    ros::shutdown();
    return 1 ;
  }

  // While the node is running, try to get data from the sensor
  while(n.ok()){

    // This call will block the program until it is able to get data from the serial port
    char c = serialGetchar(fd);

    // Sensor sends data in the format of Rxxx, where each of the x is a number and the three numbers form the measure in inches
    // If a R character is read, the next three digits represent the measure
    if ( c == 'R' )
    {
      // Variable to store the measure
      int inches = 0;

      // Read the next three digits and store the equivalent value in inches
      // Use ( c - '0' ) to convert the character to its corresponding integer value
      c = serialGetchar(fd);
      inches += 100 * ( c - '0' );
      c = serialGetchar(fd);
      inches += 10 * ( c - '0' );
      c = serialGetchar(fd);
      inches += ( c - '0' );

      // Convert to meters
      double meters = static_cast< double >( inches ) * 0.0254;

      // Get current time to fill the header of the message
      ros::Time scan_time = ros::Time::now();

      // Create and populate a Range message
      sensor_msgs::Range msg;
      msg.header.stamp = scan_time;
      msg.header.frame_id = "sonar_frame";
      msg.radiation_type = 0;
      msg.field_of_view = 0;
      msg.min_range = 0.15;
      msg.max_range = 6.45;
      msg.range = meters;

      // Publish the message
      sonar_pub.publish(msg);

      // Discard extra data received
      serialFlush(fd);
    }
  }

  return 0;
}

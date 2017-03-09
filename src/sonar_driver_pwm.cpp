#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <wiringPi.h>

int main(int argc, char** argv){

  // Local variables
  int pin;
  double distance;
  unsigned int start, end, duration;

  // Select GPIO17
  pin = 0;

  // Initialize ROS
  ros::init(argc, argv, "sonar_driver");

  // Declare the sonar publisher and inform the master that it will be publishing messages on topic "/sonar"
  ros::NodeHandle n;
  ros::Publisher sonar_pub = n.advertise<sensor_msgs::Range>("sonar", 50);

  // Configure the GPIO pins using wiringPi
  wiringPiSetup();
  pinMode( pin, INPUT );

  // While the node is running, try to get data from the sensor
  while(n.ok()){

    // Read the pin until a high signal ( 1 ) is received
    if ( digitalRead( pin ) )
    {
        // When a high signal is read, count the time until a low signal is read
        start = micros();
        while ( digitalRead( pin ) );
        end = micros();
        duration = end - start;

        // The sensor sends a pulse with 147uS for each inch it detects.
        // So the distance in meters can be calculated by:
        distance = 0.0254 * static_cast< double >( duration ) / 147;

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
        msg.range = distance;

        // Publish the message
        sonar_pub.publish(msg);
    }
  }

  return 0;
}

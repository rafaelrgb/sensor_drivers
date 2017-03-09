#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <wiringPi.h>

int main(int argc, char** argv){

  // Local variables
  int nReads, count, pin, i;
  double duty, distance;

  // Select GPIO17 and a reasonable number of readings
  pin = 0;
  nReads = 2790000;

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

    // Read the pin nReads times and count how many times a high signal is read
    count = 0;
    for ( i = 0; i < nReads; i++ )
    {
        count += digitalRead( pin );
    }

    // Each distance measurement by the sensor takes 49mS. In this period there is a 11.5mS time
    // when the PWM signal is low, because the sensor is doing other functions, and a 37.5mS time
    // when the sensor is sending the PWM signal. We're reading the pin at semi-random times, so
    // about 25% of the readings fall into the 11.5mS time and are therefore useless. The readings
    // That fall in the 37.5mS useful time have a % chance of returning 1 equal to the duty cycle.
    // So we compare the number of high signals read to 75% of the number of readings to find
    // the duty cycle, and then use it to find the distance in meters, knowing that the signal
    // stays high during 147uS for each inch.
    duty = static_cast< double >( count ) / ( 0.75 * static_cast< double >( nReads ) );
    distance = 0.0254 * ( ( duty * 37500 ) / 147 );


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

  return 0;
}

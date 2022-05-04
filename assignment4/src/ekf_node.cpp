#include "ekf_node.hpp"
#include "utilities.h"

 // constructor
EKFNode::EKFNode( ros::NodeHandle& nh, ros::NodeHandle& nhp, double r ) : 
  nh( nh ),
  nh_private( nhp ),
  sub_gps( nh, "fix", 2 ),
  sub_imu( nh, "imu", 2 ),
  sync( Policy(20), sub_gps, sub_imu ),
  z( 6 ),
  u(3),
  rate(r){

  sync.registerCallback( &EKFNode::callback_sensors, this );

  // advertise our estimation
  pub_pose=nh_private.advertise<PoseWithCovarianceStamped>("posterior",10);
  sub_twist=nh.subscribe( "cmd_vel", 2, &EKFNode::callback_command, this );

}

// Sensor callback
// This only copies the measurement
void EKFNode::callback_sensors( const NavSatFix& nsf, const Imu& imu ){

  // combine the sensors
  z(1) = nsf.latitude;
  z(2) = nsf.longitude;
  z(3) = nsf.altitude;

  tf::Quaternion q;
  tf::quaternionMsgToTF( imu.orientation, q );
  tf::Matrix3x3 R( q );

  double r, p, y;
  rotm2rpy( R, r, p, y );

  z(4) = r;
  z(5) = p;
  z(6) = y;

}

// Velocity command callback
// This calls the EKF with the latest measurements
void EKFNode::callback_command( const Twist& vw ){
  u(1) = vw.linear.x;
  u(2) = vw.angular.z;
  u(3) = rate;
  if(!ekf.is_initialized())
    {ekf.initialize();}
}


void EKFNode::update(){
  if( ekf.is_initialized() ){ 
    ekf.update( ros::Time::now(), z, u ); 
    pub_pose.publish( ekf.get_posterior() );
  }
}

int main(int argc, char **argv){
  
  // Initialize ROS
  ros::init(argc, argv, "EKF");
  ros::NodeHandle nh, nhp("~");

  // create filter class
  EKFNode ekf_node( nh, nhp, 1.0/25.0 );
  ros::Rate r(25.0);
  
  while(nh.ok()){
    ekf_node.update();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

 
#include "ekf_models.hpp"
#include <tf/tf.h>
#include "utilities.h"
#include <math.h>

const double Pi = 3.14159265358979323846;
/**
   TODO
   Fill in the value of the process covariance matrix. The rows/columns of WMWt are
   in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ].
   \param[out] WMWt Covariance matrix of the system.
   \param state_in    The current state estimate
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
void sys_evaluate_WMWt( double WMWt[6][6], const State& state, double v, double w, double dt ){

  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      WMWt[r][c] = 0.0;

  // TODO fill in the matrix WMWt

  double a1 = 0.000000000000000001;
  double a2 = 0.000000000000000001;
  double a3 = 0.000000000000000001;
  double a4 = 0.000000000000000001;
  double row = state.x[3];
  double pitch = state.x[4];
  double yaw = state.x[5];
  WMWt[0][0] = (dt*dt)*pow(cos(pitch),2.0)*pow(cos(yaw),2.0)*(a1*(v*v)+a2*(w*w));
  WMWt[0][1] = (dt*dt)*cos(pitch)*cos(yaw)*(cos(row)*sin(yaw)+cos(yaw)*sin(pitch)*sin(row))*(a1*(v*v)+a2*(w*w));
  WMWt[0][2] = (dt*dt)*cos(pitch)*cos(yaw)*(sin(row)*sin(yaw)-cos(row)*cos(yaw)*sin(pitch))*(a1*(v*v)+a2*(w*w));
  WMWt[1][0] = (dt*dt)*cos(pitch)*cos(yaw)*(cos(row)*sin(yaw)+cos(yaw)*sin(pitch)*sin(row))*(a1*(v*v)+a2*(w*w));
  WMWt[1][1] = (dt*dt)*pow(cos(row)*sin(yaw)+cos(yaw)*sin(pitch)*sin(row),2.0)*(a1*(v*v)+a2*(w*w));
  WMWt[1][2] = (dt*dt)*(cos(row)*sin(yaw)+cos(yaw)*sin(pitch)*sin(row))*(sin(row)*sin(yaw)-cos(row)*cos(yaw)*sin(pitch))*(a1*(v*v)+a2*(w*w));
  WMWt[2][0] = (dt*dt)*cos(pitch)*cos(yaw)*(sin(row)*sin(yaw)-cos(row)*cos(yaw)*sin(pitch))*(a1*(v*v)+a2*(w*w));
  WMWt[2][1] = (dt*dt)*(cos(row)*sin(yaw)+cos(yaw)*sin(pitch)*sin(row))*(sin(row)*sin(yaw)-cos(row)*cos(yaw)*sin(pitch))*(a1*(v*v)+a2*(w*w));
  WMWt[2][2] = (dt*dt)*pow(sin(row)*sin(yaw)-cos(row)*cos(yaw)*sin(pitch),2.0)*(a1*(v*v)+a2*(w*w));
  WMWt[5][5] = (dt*dt)*(a3*(v*v)+a4*(w*w));
}

/**
   TODO
   Fill in the value of the measurement covariance matrix. The rows/columns of C
   are in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ]
   \param[out] R Covariance matrix of the sensors.
   \param state_in    The current state estimate
*/
void meas_evaluate_R( double R[6][6], const State& state ){

  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      R[r][c] = 0.0;

  // TODO fill in the matrix R
  double k = 0.0000000001;
  R[3][3] = k*2.597777756995556e-05;
  R[4][4] = k*2.597777756995556e-05;
  R[0][0] = k*0.0012514613759999997;
  R[1][1] = k*0.0036;
  R[2][2] = k*0.0009060099999999998;
  R[5][5] = k*0.00000000001;
}


/**
   TODO
   Evaluate the system function.
   Compute the process model.
   This function returns the prediction of the next state based on the 
   current state estimate and the commmand input (linear/angular velocities).
   \param state_in    The current state estimate
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
State sys_evaluate_g( const State& state_in, double v, double w, double dt ){

  State state_out;

  // TODO Given state_in and v and w and dt (time increment) determine the prior
  // estimate state_out

  double row = state_in.x[3];
  double pitch = state_in.x[4];
  double yaw = state_in.x[5];
  state_out.x[0] = state_in.x[0] + dt*v*cos(pitch)*cos(yaw);
  state_out.x[1] = state_in.x[1] + dt*v*(cos(row)*sin(yaw)+cos(yaw)*sin(pitch)*sin(row));
  state_out.x[2] = state_in.x[2] + dt*v*(sin(row)*sin(yaw)-cos(row)*cos(yaw)*sin(pitch));
  state_out.x[3] = state_in.x[3];
  state_out.x[4] = state_in.x[4];
  if (state_in.x[5]>(2*Pi))
  {
    state_out.x[5] = state_in.x[5] + dt*w - 2*Pi;
  }
  else if (state_in.x[5]<0)
  {
    state_out.x[5] = state_in.x[5] + dt*w + 2*Pi;
  }
  else
  {
    state_out.x[5] = state_in.x[5] + dt*w;
  }
  return state_out;
}

/**
   TODO
   Evaluate the system Jacobian.
   This function evaluates the Jacobian of the system functions g (see 
   sys_evaluate_g). The entry G[i][j] represents ( d g_i / d s_j )
   \param[out] G      The 6x6 Jacobian of the function g
   \param state       The state of the robot
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
void sys_evaluate_G( double G[6][6], const State& state, double v, double w, double dt ){
  
  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      G[r][c] = 0.0;
  
  // TODO
  // Given state, v and w, compute the system Jacobian G

  double row = state.x[3];
  double pitch = state.x[4];
  double yaw = state.x[5];
  G[0][0] = 1.0;
  G[0][4] = -dt*v*cos(yaw)*sin(pitch);
  G[0][5] = -dt*v*cos(pitch)*sin(yaw);
  G[1][1] = 1.0;
  G[1][3] = -dt*v*(sin(row)*sin(yaw)-cos(row)*cos(yaw)*sin(pitch));
  G[1][4] = dt*v*cos(pitch)*cos(yaw)*sin(row);
  G[1][5] = dt*v*(cos(row)*cos(yaw)-sin(pitch)*sin(row)*sin(yaw));
  G[2][2] = 1.0;
  G[2][3] = dt*v*(cos(row)*sin(yaw)+cos(yaw)*sin(pitch)*sin(row));
  G[2][4] = -dt*v*cos(pitch)*cos(row)*cos(yaw);
  G[2][5] = dt*v*(cos(yaw)*sin(row)+cos(row)*sin(pitch)*sin(yaw));
  G[3][3] = 1.0;
  G[4][4] = 1.0;
  G[5][5] = 1.0;
}

/**
   TODO
   Evaluate the GPS observation function.
   This function returns the expected satellite fix given the state of the robot
   \param state The state estimate
   \return      A satellite navigation fix (only the latitute, longitude
                and altitude members are used)
*/
sensor_msgs::NavSatFix meas_evaluate_gps( const State& state ){

  sensor_msgs::NavSatFix nsf;

  // TODO
  // Given prior estimate state, determine the expected GPS measurement nsf

  nsf.latitude = 2.53926534066196e-06*state.x[0] + 8.76154349649383e-06*state.x[1] + 2.19895528452696e-07*state.x[2] + 35.8594561687988;

  nsf.longitude = 1.05533638183886e-05*state.x[0] - 3.12635674326600e-06*state.x[1] - 2.18307260269057e-07*state.x[2] - 108.236838618080;

  nsf.altitude =  0.00294975815753729*state.x[0] + 0.00245516512975730*state.x[1] + 1.00254930559765*state.x[2] + 64.0026108047787;
  return nsf;
}

/**
   TODO
   Evaluate the IMU observation function.
   This function computes the expected imu orientation given the state of the 
   robot.
   \param state_in The current state estimate
   \return         A inertial navigation unit measurement (only the orientation
                   member is used).
*/
sensor_msgs::RPY meas_evaluate_imu( const State& state ){
  sensor_msgs::RPY rpy;

  // TODO
  // Given the prior estimate state, determine the expected RPY measurement rpy 

  rpy.roll = state.x[3];
  rpy.pitch = state.x[4];
  rpy.yaw = state.x[5];
  return rpy;
}

/** 
    TODO
    Observation Jacobian of the GPS
    This function returns the 3x3 observation Jacobian of the GPS. Essentially,
    this is the Jacobian of your meas_evaluate_gps function.
    \param[out] Hgps The 3x3 GPS Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Hgps( double Hgps[3][3], const State& state ){

  // TODO
  // Fill the Jacobian matrix Hgps of the GPS observations
  
Hgps[0][0] = 2.53926534066196e-06;
Hgps[0][1] = 8.76154349649383e-06;
Hgps[0][2] = 2.19895528452696e-07;
Hgps[1][0] = 1.05533638183886e-05;
Hgps[1][1] = - 3.12635674326600e-06;
Hgps[1][2] = - 2.18307260269057e-07;
Hgps[2][0] = 0.00294975815753729;
Hgps[2][1] = 0.00245516512975730;
Hgps[2][2] = 1.00254930559765;

}

/** 
    Observation Jacobian of the IMU
    This function returns the 3x3 observation Jacobian of the IMU. Essentially,
    this is the Jacobian of your meas_evaluate_imu function.
    \param[out] Himu The 3x3 IMU Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Himu( double Himu[3][3], const State& state ){

  // TODO
  // Fill the Jacobian matrix Himu of the IMU observations
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (i == j)
      {
        Himu[i][j] = 1;
      }
      else{
        Himu[i][j] = 0;
      }
    } 
  }
  
}


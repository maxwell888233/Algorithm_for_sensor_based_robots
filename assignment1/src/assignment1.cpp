/**
   EN.601.463/663
   Assignment #1

   Cartesian trajectory generation

 */
#include "assignment1.hpp"
#include <cmath>

// Compute the forward kinematics (position and orientation)
// input: the joints angles
// output: the 4x4 homogeneous transformation
void ForwardKinematics(double q1, double q2, double q3, double E[4][4])
{

  // TODO
  // Fill the values of the forward kinematics (homogeneous matrix E)
  double E_temp[4][4] = {
{     sin(q1), cos(q2 + q3)*cos(q1), sin(q2 + q3)*cos(q1), cos(q1)*(sin(q2)*(0.213*cos(q3) - 0.486*sin(q3) + 0.0892) - 0.0892*sin(q2) + cos(q2)*(0.486*cos(q3) + 0.213*sin(q3) + 0.425)) - 0.109*sin(q1)},
{-1.0*cos(q1), cos(q2 + q3)*sin(q1), sin(q2 + q3)*sin(q1), 0.109*cos(q1) + sin(q1)*(sin(q2)*(0.213*cos(q3) - 0.486*sin(q3) + 0.0892) - 0.0892*sin(q2) + cos(q2)*(0.486*cos(q3) + 0.213*sin(q3) + 0.425))},
{           0,    -1.0*sin(q2 + q3),         cos(q2 + q3),              cos(q2)*(0.213*cos(q3) - 0.486*sin(q3) + 0.0892) - 0.0892*cos(q2) - 1.0*sin(q2)*(0.486*cos(q3) + 0.213*sin(q3) + 0.425) + 0.0892},
{           0,                    0,                    0,                                                                                                                                           1.0}
      };

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      E[i][j] = E_temp[i][j];
    }
  }
}

// Compute the inverse of the forward kinematics (position and orientation)
// input: the joints angles
// output: the 4x4 homogeneous transformation
void ForwardKinematicsInverse(double q1, double q2, double q3, double E[4][4])
{

  // TODO
  // Fill the values of the inverse of the forward kinematics (homogeneous matrix E)
  double E_temp[4][4] = {
{             sin(q1),         -1.0*cos(q1),                 0,                                          0.1091},
{cos(q2 + q3)*cos(q1), cos(q2 + q3)*sin(q1), -1.0*sin(q2 + q3),   0.08916*sin(q2 + q3) - 0.425*cos(q3) - 0.4858},
{sin(q2 + q3)*cos(q1), sin(q2 + q3)*sin(q1),      cos(q2 + q3), - 0.08916*cos(q2 + q3) - 0.425*sin(q3) - 0.2125},
{                   0,                    0,                 0,                                             1.0}
      };

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      E[i][j] = E_temp[i][j];
    }
  }
  // std::cout<<"**********************************************\n";
  // std::cout<< q1_temp << "' "<< q2_temp << "' "<< q3_temp << std::endl;
  // std::cout<< E[0][0] << ", "<< E[0][1] << ", "<< E[0][2] << "; "<<std::endl;
  // std::cout<< E[1][0] << ", "<< E[1][1] << ", "<< E[1][2] << "; "<<std::endl;
  // std::cout<< E[2][0] << ", "<< E[2][1] << ", "<< E[2][2] << "; \n"<<std::endl;
}

// Compute the Adjoint transformation inverse matrix
// input E: the rotation/translation between the base and the hand frame
//          (as computed by the forward kinematics)
// output Ad: the 6x6 adjoint transformation inverse matrix
void AdjointTransformationInverse(double E[4][4], double Ad[6][6])
{

  // TODO
  // Compute the Adjoint Transformation Inverse A^-1
  double g11 = E[0][0];
  double g12 = E[0][1];
  double g13 = E[0][2];
  double g14 = E[0][3];
  double g21 = E[1][0];
  double g22 = E[1][1];
  double g23 = E[1][2];
  double g24 = E[1][3];
  double g31 = E[2][0];
  double g32 = E[2][1];
  double g33 = E[2][2];
  double g34 = E[2][3];
  double g41 = E[3][0];
  double g42 = E[3][1];
  double g43 = E[3][2];
  double g44 = E[3][3];

  double Ad_temp[6][6] = {
      {g11, g21, g31, g24 * g31 - g21 * g34, g11 * g34 - g14 * g31, g14 * g21 - g11 * g24},
      {g12, g22, g32, g24 * g32 - g22 * g34, g12 * g34 - g14 * g32, g14 * g22 - g12 * g24},
      {g13, g23, g33, g24 * g33 - g23 * g34, g13 * g34 - g14 * g33, g14 * g23 - g13 * g24},
      {0, 0, 0, g11, g21, g31},
      {0, 0, 0, g12, g22, g32},
      {0, 0, 0, g13, g23, g33}};

  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      Ad[i][j] = Ad_temp[i][j];
    }
  }
}

// Compute and return the Jacobian of the robot given the current joint
// positions
// input: the joints angles
// output: the 6x3 Jacobian (position only)
void Jacobian(double q1, double q2, double q3, double J[6][3])
{

  // TODO
  // Fill the values of the Jacobian matrix J
  double J_temp[6][3] = {
{  0, -0.08916*cos(q1), 2.776e-18*cos(q1)*(1.531e+17*sin(q2) - 3.212e+16)},
{  0, -0.08916*sin(q1), 2.776e-18*sin(q1)*(1.531e+17*sin(q2) - 3.212e+16)},
{  0,                0,                                     0.425*cos(q2)},
{  0,     -1.0*sin(q1),                                      -1.0*sin(q1)},
{  0,          cos(q1),                                           cos(q1)},
{1.0,                0,                                                 0}
      };

  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      J[i][j] = J_temp[i][j];
    }
  }
}

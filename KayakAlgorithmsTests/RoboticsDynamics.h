/*
 * =====================================================================================
 *
 *       Filename:  RoboticsDynamics.h
 *
 *    Description:  Some Dynamics algorithms from Matlab for axises
 *
 *        Version:  1.0
 *        Created:  9/24/2019 4:36:56 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Alexander Hsu (), chenyu.xu@surgnova.com
 *   Organization:  Surgnova
 *
 * =====================================================================================
 */

#ifndef _ROBOTICS_DYNAMICS
#define _ROBOTICS_DYNAMICS

#include <Eigen/Dense>

namespace rd {

using namespace Eigen;

Matrix<double, 7, 1> G_NoInstrument(Vector3d g, Matrix<double, 7, 1> q);
Matrix<double, 7, 1> D_NoInstrument(Matrix<double, 7, 1> q, Matrix<double, 7, 1> ddqd);
Matrix<double, 7, 1> V_NoInstrument(Matrix<double, 7, 1> q, Matrix<double, 7, 1> dq);
Matrix<double, 7, 1> PD_P_NoInstrument(Matrix<double, 7, 1> Kp, Matrix<double, 7, 1> q, Matrix<double, 7, 1> qd);
Matrix<double, 7, 1> PD_D_NoInstrument(Matrix<double, 7, 1> Kv, Matrix<double, 7, 1> q, Matrix<double, 7, 1> dq, Matrix<double, 7, 1> dqd);

double dynamics_torque_T1(Vector4d q, Vector4d dq, Vector4d dqd, Vector4d ddqd, Vector3d DesiredFlange,
                          Vector4d kp, Vector4d w);

double dynamics_torque_T2(Vector4d q, Vector4d dq, Vector4d dqd, Vector4d ddqd, Vector3d DesiredFlange,
                          Vector4d kp, Vector4d w);

double dynamics_torque_T3(Vector4d q, Vector4d dq, Vector4d dqd, Vector4d ddqd, Vector3d DesiredFlange,
                          Vector4d kp, Vector4d w);

double dynamics_torque_T4(Vector4d q, Vector4d dq, Vector4d dqd, Vector4d ddqd, Vector3d DesiredFlange,
                          Vector4d kp, Vector4d w);

bool prepare_DH(double dh_param[], int rowNumber, double q[]);

Vector3d forward_kinematics(Vector4d q);

Matrix<double, 7, 1> DesiredPosition(Matrix<double, 7, 1> q, Matrix<double, 7, 1> dqd, double deltaT);

Matrix<double, 9, 1> torque_G_nominalDH(Matrix<double, 9, 1> q);
Matrix<double, 9, 1> torque_SpringEnd_nominalDH(Matrix<double, 9, 1> q, Matrix<double, 6, 1> DesiredEnd, Matrix<double, 6, 1> KSpring);
Matrix<double, 9, 1> torque_V_nominalDH(Matrix<double, 9, 1> dq, Matrix<double, 9, 1> dqd, Matrix<double, 9, 1> Kv);

double distance_to_trocar_nominalDH(Matrix<double, 9, 1> q, Vector3d trocar);
Matrix<double, 9, 1> torque_Trocar_nominalDH(Matrix<double, 9, 1> q, Vector3d trocar, double Kt, double D_FT);
Matrix<double, 9, 1> torque_V_nominalDH(Matrix<double, 9, 1>q, Matrix<double, 9, 1> dq);

// feed forward
Matrix<double, 7, 1> ff_G_CalibratedDH_NoInstrument(Matrix<double, 7, 1> q);
Matrix<double, 7, 1> ff_V_CalibratedDH_NoInstrument(Matrix<double, 7, 1> q, Matrix<double, 7, 1> dq);

Matrix<double, 7, 1> ff_G_NominalDH_NoInstrument(Matrix<double, 7, 1> q);
Matrix<double, 7, 1> ff_V_NominalDH_NoInstrument(Matrix<double, 7, 1> q, Matrix<double, 7, 1> dq);

Matrix<double, 7, 1> ff_G_CalibratedDH_with_instrument(Matrix<double, 7, 1> q);
Matrix<double, 7, 1> ff_V_CalibratedDH_with_instrument(Matrix<double, 7, 1> q, Matrix<double, 7, 1> dq);

Matrix<double, 7, 1> ff_G_NominalDH_with_instrument(Matrix<double, 7, 1> q);
Matrix<double, 7, 1> ff_V_NominalDH_with_instrument(Matrix<double, 7, 1> q, Matrix<double, 7, 1> dq);

// feed back
Matrix<double, 7, 1> fb_V_CalibratedDH_with_instrument(Matrix<double, 7, 1> q, Matrix<double, 7, 1> dq, Matrix<double, 7, 1> dqd, Matrix<double, 7, 1> Kv);

Matrix<double, 7, 1> fb_End_CalibratedDH_NoInstrument(Matrix<double, 7, 1> q, Vector3d  trocar, Matrix<double, 2, 1> stiffness, double D_FT);
Matrix<double, 7, 1> fb_End_CalibratedDH_with_instrument(Matrix<double, 7, 1> q, Matrix<double, 6, 1> trocar, Matrix<double, 6, 1> stiffness, double D_FT);
Matrix<double, 7, 1> fb_End_NominalDH_with_instrument(Matrix<double, 7, 1> q, Matrix<double, 6, 1> trocar, Matrix<double, 6, 1> stiffness, double D_FT);


Matrix<double, 6, 6> EndEffectorJacobian_6_Axes(Matrix<double, 6, 1> theta);
Matrix<double, 7, 6> EndEffectorJacobian_7_Axes(Matrix<double, 7, 1> theta, double D_FT);

Matrix<double, 4, 4> EndEffector_FK_7_Axes(Matrix<double, 7, 1> q);
Matrix<double, 4, 4> EndEffector_FK_9_Axes(Matrix<double, 9, 1> theta);

Vector3d getTrocarPosFromFlange(Matrix<double, 4, 4> flangeHomo, double dLen);
Matrix<double, 6, 1> TrocarVelocity_from_6DOF(Vector3d theta, Vector3d dqd);

// return q1 - q3 of gimbal of 6DOF robot
Vector3d GimbalInitJoint_from_Trocar(Matrix<double, 3, 3> rot);

// q: gimbal joints(first three joints of 6DOF)
Vector3d TrocarEuler_from_GimbalJoint(Vector3d q);

double Distance_from_Flange_to_Trocar(Matrix<double, 7, 1> q, Vector3d trocar);

Vector3d posErrTrocar2FootInBase(Matrix<double, 7, 1> q, Vector3d trocar, double D_FT);

double Distance_from_Trocar_to_Shalft(Matrix<double, 7, 1> q, Vector3d trocar);
}
#endif

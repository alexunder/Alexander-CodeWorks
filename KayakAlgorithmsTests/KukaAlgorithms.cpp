 /* =====================================================================================
 *
 *       Filename:  KukaAlgorithms.cpp
 *
 *    Description:  Some algorithms need to run on Workstation.
 *
 *        Version:  1.0
 *        Created:  8/9/2021 4:36:56 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Alexander Hsu (), chenyu.xu@surgnova.com
 *   Organization:  Surgnova
 *
 * =====================================================================================
 */

#include "KukaAlgorithms.h"
#include "Algorithm.h"
#include "Quaternion.h"



void save_matrix_file(vector<Matrix<double, 6 , 1>> matrix, const string name) {
    string filename = name + ".txt";
    ofstream out(filename, ios::out);
    if (out.is_open()) {
        for (auto iter : matrix) {
            out.width(15);
            out << right << iter.transpose() << endl;
        }
        out.close();
    }
}

void TrajectoryPlanningInternal(double q0, double q1, unsigned int t,
                                vector<double> & s,
                                vector<double> & sd,
                                vector<double> & ssd) {
    if (t == 0) {
        return;
    }

    vector<unsigned int> time_frame;
    unsigned int i;

    unsigned int tf = t - 1;
    double v = ((q1 - q0) / tf) * 1.5;

    for (i = 0; i < t; i++) {
        s.push_back(0.0);
        sd.push_back(0.0);
        ssd.push_back(0.0);
    }

    double tb = (q0 - q1 + v*tf) / v;
    double a = v / tb;

    for (i = 0; i < t; i++) {
        if (i <= static_cast<unsigned int>(tb)) {
            s[i] = q0 + (a / 2)*(i*i);
            sd[i] = a*i;
            ssd[i] = a;
        } else if (i <= static_cast<unsigned int>(tf - tb)) {
            s[i] = (q1 + q0 -v*tf) / 2 + v*i;
            sd[i] = v;
            ssd[i] = 0;
        } else {
            s[i] = q1 - (a/2)*(tf*tf) + a*tf*i - (a/2)*(i*i);
            sd[i] = a*tf - a*i;
            ssd[i] = -a;
        }
    }
}

Matrix<double, 3, 1> calculateRotateVelocity(Matrix<double, 3, 1> angle1,
                                             Matrix<double, 3, 1> angle0) {

    double a0[3] = {angle0(0, 0), angle0(1, 0), angle0(2, 0)};
    double a1[3] = {angle1(0, 0), angle1(1, 0), angle1(2, 0)};

    double * r0 = Algorithm::eul2rotm(a0);
    double * r1 = Algorithm::eul2rotm(a1);

    Matrix<double, 3, 3> rotate_0;
    rotate_0 << r0[0], r0[1], r0[2],
                r0[3], r0[4], r0[5],
                r0[6], r0[7], r0[8];

    Matrix<double, 3, 3> rotate_1;
    rotate_1 << r1[0], r1[1], r1[2],
                r1[3], r1[4], r1[5],
                r1[6], r1[7], r1[8];
    delete [] r0;
    delete [] r1;

    Matrix<double, 3, 3> delta_rotate = rotate_1*rotate_0.transpose() - MatrixXd::Identity(3, 3);
    //cout << "delta_rotate=" << delta_rotate << endl;
    Matrix<double, 3, 1> velocity_rotate;
    velocity_rotate << delta_rotate(2, 1) - delta_rotate(1, 2),
                        delta_rotate(0, 2) - delta_rotate(2, 0),
                        delta_rotate(1, 0) - delta_rotate(0, 1);

    return velocity_rotate / 2;
}

void angle2quat(double * angle, double * q) {

    CQuat::angle2quat(angle[0], angle[1], angle[2], q);
    if (q[0] < 0) {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }
}

void TrajectoyPlanningCartesianMaxtrix(Matrix<double, 6, 1> start,
                                       Matrix<double, 6, 1> end,
                                       unsigned int num,
                                       vector<Matrix<double, 6, 1>> & full_traj,
                                       vector<Matrix<double, 6, 1>> & full_velocity
                                       ) {
    vector<double> s;
    vector<double> sd;
    vector<double> ssd;

    TrajectoryPlanningInternal(0.0, 1.0, num, s, sd, ssd);

    Matrix<double, 3, 1> p0 = start.block(0, 0, 3, 1);
    Matrix<double, 3, 1> p1 = end.block(0, 0, 3, 1);

    vector<Matrix<double, 3, 1>> xyz_Traj;
    vector<Matrix<double, 3, 1>> abc_Traj;
    vector<Matrix<double, 4, 1>> quat_Traj;

    unsigned int i;
    for (i = 0; i < num; i++) {
        xyz_Traj.push_back(p0 + (p1 - p0)*s[i]);
    }

    Matrix<double, 3, 1> angle0 = start.block(3, 0, 3, 1);
    Matrix<double, 3, 1> angle1 = end.block(3, 0, 3, 1);

    double euler0[3] = {angle0(0, 0), angle0(1, 0), angle0(2, 0)};
    double euler1[3] = {angle1(0, 0), angle1(1, 0), angle1(2, 0)};
    double q0[4];
    double q1[4];
    angle2quat(euler0, q0);
    angle2quat(euler1, q1);
    // CQuat::angle2quat(angle0(0, 0), angle0(1, 0), angle0(2, 0), q0);
    // CQuat::angle2quat(angle1(0, 0), angle1(1, 0), angle1(2, 0), q1);

    Matrix<double, 4, 1> quat0(q0[0], q0[1], q0[2], q0[3]);
    Matrix<double, 4, 1> quat1(q1[0], q1[1], q1[2], q1[3]);

    Matrix<double, 4, 1> delta;

    Matrix<double, 4, 1> temp = quat0.array().abs() - quat1.array().abs();
    if (temp.norm() < 1e-15) {
        delta = MatrixXd::Zero(4, 1);
    } else {
        delta = quat1 - quat0;
    }

    for (i = 0; i < num; i++) {
        quat_Traj.push_back(quat0 + delta*s[i]);
    }


    for (i = 0; i < num; i++) {
        double q[4] = { quat_Traj[i](0, 0),
                        quat_Traj[i](1, 0),
                        quat_Traj[i](2, 0),
                        quat_Traj[i](3, 0) };
        double angle[3];
        CQuat::quat2angle(q, angle);
        abc_Traj.push_back(Vector3d(angle[0], angle[1], angle[2]));
    }

    for (i = 0; i < num; i++) {
        Matrix<double, 6, 1> item;
        item << xyz_Traj[i](0,0), xyz_Traj[i](1,0), xyz_Traj[i](2,0),
                abc_Traj[i](0,0), abc_Traj[i](1,0), abc_Traj[i](2,0);
        full_traj.push_back(item);
    }

    // Veclocity Generation
    vector<Matrix<double, 3, 1>> position_velocity;
    vector<Matrix<double, 3, 1>> rotation_velocity;

    for (i = 0; i < num - 1; i++) {
        position_velocity.push_back(xyz_Traj[i + 1] - xyz_Traj[i]);
        rotation_velocity.push_back(calculateRotateVelocity(abc_Traj[i + 1], abc_Traj[i]));
    }

    position_velocity.push_back(MatrixXd::Zero(3, 1));
    rotation_velocity.push_back(MatrixXd::Zero(3, 1));

    for (i = 0; i < num; i++) {
        Matrix<double, 6, 1> item;
        item << position_velocity[i](0,0), position_velocity[i](1,0), position_velocity[i](2,0),
                rotation_velocity[i](0,0), rotation_velocity[i](1,0), rotation_velocity[i](2,0);
        full_velocity.push_back(item);
    }
    return;
}

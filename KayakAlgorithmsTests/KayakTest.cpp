#include <gtest/gtest.h>

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <signal.h>
#include <unistd.h>
#include <vector>
#include <bitset>
#include <gtest/gtest.h>

#include "RoboticsDynamics.h"
#include "Algorithm.h"
#include "Quaternion.h"
#include "KukaAlgorithms.h"
#include "hls_helper.h"

using namespace std;
using namespace rd;

class kayak_test : public testing::Test {};

TEST(kayak_test, hls_conversion) {
    cout << "ap_fixed to double case:" << endl;
    cout << "case 1: positive value case:" << endl;
    unsigned int ap_fixed_value = 0b000100100001101000110110;
    unsigned int W = 24;
    unsigned int I = 4;
    const double exp_value = 1.1314;
    double value = hls_helper::hls_apfixed_to_double(ap_fixed_value, W, I);
    cout << "positive double value=" << value << endl;
    cout << "Desired value=" << exp_value << endl;
    EXPECT_NEAR(value, exp_value, 0.001);

    cout << "case 2: negative value case:" << endl;
    ap_fixed_value = 0b111011011110010111001001;
    const double exp_value_1 = -1.1314;
    value = hls_helper::hls_apfixed_to_double(ap_fixed_value, W, I);
    cout << "negative double value=" << value << endl;
    EXPECT_NEAR(value, exp_value_1, 0.001);

    cout << "double to ap_fixed case:" << endl;
    cout << "case 3: positive case:" << endl;
    double d_value = -1.1314;
    const unsigned int exp_ap_value = 0b111011011110010111001001;
    unsigned int ap_value = hls_helper::double_to_hls_apfixed(d_value, W, I);
    cout << "ap_value=" << ap_value << endl;
    cout << "ap_value actual bin=" << bitset<24>(ap_value) << endl;
    cout << "ap_value expect bin=" << bitset<24>(ap_fixed_value) << endl;
    EXPECT_EQ(ap_value, exp_ap_value);

    cout << "case 4: negative case:" << endl;
    d_value = 1.1314;
    const unsigned int exp_ap_value_1 = 0b000100100001101000110110;
    ap_value = hls_helper::double_to_hls_apfixed(d_value, W, I);
    cout << "ap_value=" << ap_value << endl;
    cout << "ap_value actual bin=" << bitset<24>(ap_value) << endl;
    cout << "ap_value expect bin=" << bitset<24>(~ap_fixed_value) << endl;
    EXPECT_EQ(ap_value, exp_ap_value_1);

    cout << "case 5: ap_fixed<32, 10> positive:" << endl;
    ap_fixed_value = 0b00100101100010000110100011011011;
    W = 32;
    I = 10;
    const double exp_value_2 = 150.1314;
    value = hls_helper::hls_apfixed_to_double(ap_fixed_value, W, I);
    cout << "positive double value=" << value << endl;
    EXPECT_NEAR(value, exp_value_2, 0.001);

    cout << "case 6: ap_fixed<32, 10> negative:" << endl;
    ap_fixed_value = 0b11011010011101111001011100100100;
    const double exp_value_3 = -150.1314;
    value = hls_helper::hls_apfixed_to_double(ap_fixed_value, W, I);
    cout << "negative double value=" << value << endl;
    EXPECT_NEAR(value, exp_value_3, 0.001);

    cout << "double to ap_fixed case:" << endl;
    cout << "case 7: positive case:" << endl;
    d_value = 150.1314;
    const unsigned int exp_ap_value_2 = 0b00100101100010000110100011011011;
    ap_value = hls_helper::double_to_hls_apfixed(d_value, W, I);
    cout << "ap_value=" << ap_value << endl;
    cout << "ap_value actual bin=" << bitset<32>(ap_value) << endl;
    cout << "ap_value expect bin=" << bitset<32>(~ap_fixed_value) << endl;

    cout << "case 8: negative case:" << endl;
    d_value = -150.1314;
    const unsigned int exp_ap_value_3 = 0b11011010011101111001011100100100;
    ap_value = hls_helper::double_to_hls_apfixed(d_value, W, I);
    cout << "ap_value=" << ap_value << endl;
    cout << "ap_value actual bin=" << bitset<32>(ap_value) << endl;
    cout << "ap_value expect bin=" << bitset<32>(ap_fixed_value) << endl;
}

TEST(kayak_test, trajectory_plan) {
    Matrix<double, 6, 1> start;
    start << -0.37, 0.033, 0.3766, -M_PI + 0.1, 0.1, M_PI - 0.1;
    Matrix<double, 6, 1> end;
    end << -0.3, 0.133, 0.2766, M_PI - 0.1, 0.5, M_PI - 1;
    unsigned int num = 100;
    vector<Matrix<double, 6, 1>> full_traj;
    vector<Matrix<double, 6, 1>> full_velocity;
    TrajectoyPlanningCartesianMaxtrix(start, end, num, full_traj, full_velocity);

    print_std_vector(full_traj,  "Trajectory:");
    print_std_vector(full_velocity,  "Velocity:");
    //save_matrix_file(full_traj, "trajectory");
    //save_matrix_file(full_velocity, "velocity");
}

void Kayak_normal_operation() {
    const double deltaT = 0.0005;

    //Calculate the 6 virtual axes
    Matrix<double, 6, 1> velocity;
    velocity << 0.1, 0.2, 0.3, 0.0, 0.0, 0.0;

    Matrix<double, 6, 1> Virtual6Axes;
    Matrix<double, 6, 6> pinvJacobianiVirtual = rd::EndEffectorJacobian_6_Axes(Virtual6Axes);
    Matrix<double, 6, 1> dqdVirtual = pinvJacobianiVirtual * velocity;

    Vector3d TrocarPosition;
    TrocarPosition << 0.259287724604067, 0.0185841672967625, 0.830093109958027;

    Matrix<double, 4, 4> EndEffectorHomoInBase;
    EndEffectorHomoInBase <<  0.761753164310131,  6.52176728677884e-05,  -0.647867357111128,   0.444882473197092,
                             -0.150332072177128,    -0.972688103939943,  -0.176856214277729, -0.0179889336618959,
                             -0.630184405343732,     0.232116023144405,  -0.740938436755168,   0.676780452769591,
                                              0,                     0,                   0,                   1;
    Virtual6Axes = Algorithm::NewSurgnova_IK_6Joints(EndEffectorHomoInBase, TrocarPosition);


    Vector3d virtualTheta;
    virtualTheta << Virtual6Axes(0), Virtual6Axes(1), Virtual6Axes(2);
    Vector3d dqd3;
    dqd3 << dqdVirtual(0), dqdVirtual(1), dqdVirtual(2);
    Matrix<double, 6, 1> trocarVelocity = rd::TrocarVelocity_from_6DOF(virtualTheta, dqd3);

    // Get the actual positionand velocities
    Matrix<double, 7, 1> q1;
    q1 << 0,-0.930185999962371, 0.115691665360561, 1.843301564225779,
          -0.147062522608250, 1.367368461792402, -0.033192245330492;
    Vector3d trocar1;
    trocar1 << 0.259287724604067, 0.0185841672967625, 0.830093109958027;
    double err1 = Distance_from_Trocar_to_Shalft(q1, trocar1);

    Matrix<double, 7, 1> theta2;
    theta2 << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    double d_ft = 0.2;
    Matrix<double, 7, 6> pinvJacobianReal = EndEffectorJacobian_7_Axes(theta2, d_ft);

    // The real torque calculating
    Matrix<double, 7, 1> realTheta;
    realTheta << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    Matrix<double, 7, 1> dq;
    dq << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7;

    Matrix<double, 7, 1> axesTorque = rd::ff_G_NominalDH_with_instrument(realTheta);
    axesTorque += rd::ff_V_NominalDH_with_instrument(realTheta, dq);
    // Trocar Eular is changing
    Vector3d jointQ;
    jointQ << 2.9470527694122, 2.2518270813473, 2.8381966986751;
    Vector3d mTrocarEuler = rd::TrocarEuler_from_GimbalJoint(jointQ);

    Matrix<double, 7, 1> q;
    q << 0.1, 0.2, 0.3, 0.4, 0.2, 0.3, 0.4;

    Matrix<double, 6, 1> trocar;
    trocar << 0.2, 0.3, 0.4, 1.2, 1.3, 1.4;

    Matrix<double, 6, 1> stiffness;
    stiffness << 100, 200, 300, 400, 500, 600;
    axesTorque += rd::fb_End_CalibratedDH_with_instrument(q, trocar, stiffness, d_ft);
}

TEST(kayak_test, oneshot ) {
    clock_t begin = clock();

    Kayak_normal_operation();

    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;

    Matrix<double, 10, 1> theta;
    theta << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
    cout << "theta(0, 0)=" << theta(0, 0) << endl;
    cout << "theta(1, 0)=" << theta(1, 0) << endl;
    cout << "theta(2, 0)=" << theta(2, 0) << endl;
    Matrix<double, 5, 1> block1 = theta.block(0, 0, 5, 1);
    cout << "Block1:" << endl;
    cout << block1 << endl;
    Matrix<double, 5, 1> block2 = theta.block(5, 0, 5, 1);
    cout << "Block2:" << endl;
    cout << block2 << endl;
    printf("Kayak_normal_operation took %f seconds to execute \n", time_spent);
}

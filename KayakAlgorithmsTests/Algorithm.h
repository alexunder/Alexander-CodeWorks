/******************************************************************************

  Copyright (C), 2015-2017, Sino Co., Ltd.

 ******************************************************************************
  File Name     : Algorithm.h
  Version       : 555ce31f9e446577d1e8dc4cb8a177e50242346a
  Author        : Lin Lin <lin.lin@sinosurgical.cn>
  Created       :
  Last Modified : Thu Sep 21 18:18:08 2017 +0800
  Description   : algorithm header file
  Function List :
  History       :
  1.Date        : Mon Mar 13 16:45:57 2017 +0800
    Author      : deming.qiao <deming.qiao@sinosurgical.cn>
    Modification: Created file

    commit 27191ba535b58bac2724d7f11b43b3908f057ec6
    Author: Lin Lin <lin.lin@sinosurgical.cn>
    Date:   Thu Sep 21 11:58:49 2017 +0800
        1, overload the algorithm of FK_DH;
        2, overload the algorithm of homo2RPY;

    commit eaed7100bb001be978c2f47fb62bcfd0c9367ff9
    Author: Lin Lin <lin.lin@sinosurgical.cn>
    Date:   Fri Sep 22 15:14:26 2017 +0800
        optimize algorithm of findNearest;
        (Total time : 0 s, 6135 us,
        findNearest_new, Algorithm.cpp/h)

    commit 86d032bd8ebeb26c4bf6b9c820069688cde6dff3
    Author: Lin Lin <lin.lin@sinosurgical.cn>
    Date:   Fri Sep 22 16:03:15 2017 +0800
        optimize algorithm of findNearest;
        (Total time : 0 s, 5280 us, findNearest_new, Algorithm.cpp/h)

    Author: Lin Lin <lin.lin@sinosurgical.cn>
    Date:   Fri Sep 22 17:05:10 2017 +0800
        1, optimize algorithm of findNearest;
        2, optimize algorithm of AdjustEx;
        (Total time : 0 s, 5084 us, Algorithm.cpp/h)

    commit 94e2d1fde495ce40592a700c3a2777549023e741
    Author: Lin Lin <lin.lin@sinosurgical.cn>
    Date:   Fri Sep 22 19:37:13 2017 +0800
            1, optimize algorithm of findNearest;
            (Total time : 0 s, 4799 us, Algorithm.cpp/h)


    commit 6e12c9b4db3807d69704f8d537f10754b9f44f38
    Author: Lin Lin <lin.lin@sinosurgical.cn>
    Date:   Fri Sep 22 23:29:36 2017 +0800
        1, optimize algorithm of findNearest;
        2, the optFunc may be have problem.
        (Total time : 0 s, 3766 us, Algorithm.cpp/h)

    commit 633e9aab23dbc6bdeba39e3563887a7d9ef9b910
    Author: Lin Lin <lin.lin@sinosurgical.cn>
    Date:   Tue Sep 26 14:41:08 2017 +0800
        1, optimize the algorithm of translating DH to HomoMatrix (DHfromPostposition2Homo, Algorithm.h/cpp);
        2, optimize the algorithm of IK_DH, FK_DH(DHfromPostposition2Homo, Algorithm.h/cpp).
        (inter time[0] : 0 s, 1167 us)

    commit 2fec3cfe6ab1203117e7b16eb36b77ad4f60b42d
    Author: Lin Lin <lin.lin@sinosurgical.cn>
    Date:   Wed Sep 27 14:50:22 2017 +0800
        1, optimize quaternion class, to some member functions, I use Lower Programming to improve performance (Quaternion.h/cpp);
        2, optimize algorithm of QuatDiff by using Lower Programming (Algorithm.h/cpp);
        3, add some uts for this modification (QuaternionTest.cpp, AlgorithmUnitTest.cpp).

    commit 555ce31f9e446577d1e8dc4cb8a177e50242346a
    Author: Lin Lin <lin.lin@sinosurgical.cn>
    Date:   Wed Sep 27 21:53:30 2017 +0800
        1, inline member function process in class CQuat (Quaternion.h/cpp);
        2, code optimization(Quaternion.h/cpp, Algorithm.h/cpp);
        3, modified some uts according this amend version (AlgorithmUnitTest.cpp).
        (inter time[0] : 0 s, 934 us)
        Change-Id: I2a2bd8d545618a08bce6a8e04fcfec9751d38ebf
******************************************************************************/
#ifndef _ALGORITHM_H_  // to avoid the confliction of <algorithm>
#define _ALGORITHM_H_
#include <math.h>

#include "Utils/GeometryUtils.h"
#include "Utils/Frame.h"
#include "Utils/Rotation.h"
#include "Utils/Joint.h"
#include "Quaternion.h"

#include <algorithm>
#include <vector>
/* BEGIN: Added by lin.lin, 2017/9/21 */
#include <sys/time.h>
/* END:   Added by lin.lin, 2017/9/21   PN: */
#include "Utils/UtilFunc.h"

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

/*----------------------------------------------*
 * external variables                           *
 *----------------------------------------------*/

/*----------------------------------------------*
 * external routine prototypes                  *
 *----------------------------------------------*/
template<typename T>
struct OBJ_FUNC {
    int               nID ;               // source data ID
    int               nOptPitchPointId ;  //
    int               nRollListId ;
    int               nPitchListId ;
    int               nYawListId ;
    bool              bIsInRange;      // reserved
    T                 tRotEvaluateValue;
    T                 tPosEvaluateValue;
    T                 tEvaluateValue;
};

int time_substract(struct timeval *result, struct timeval *begin, struct timeval *end);

/*----------------------------------------------*
 * internal routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * project-wide global variables                *
 *----------------------------------------------*/

/*----------------------------------------------*
 * module-wide global variables                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * constants                                    *
 *----------------------------------------------*/

/*----------------------------------------------*
 * macros                                       *
 *----------------------------------------------*/
#define RMRC_KINEMATIC_SINGULARITIES
#define IK_RMRC_ERR_POSITION_THRESHOLD      (0.0001)
#define IK_RMRC_ERR_ORIENTATION_THRESHOLD   (0.0001)
#define RMRC_MAX_JOINT_NUM                  (8)

#define ALG_SORT_ASCEND  (0)
#define ALG_SORT_DESCEND (1)
//#define TEST_MODE

#ifdef TEST_MODE
    #define PRIVATE public
#else
    #define PRIVATE public
#endif

//#define dbg_IK_ex
//#define dbg_IK_ex_new

#define HOMO_MATRIX_ELEM_CNT (16)
#define DBG_ALGORITHM

#define TOOL_ROLL_DEGREE_TO_FLANGE        -90.0

//#define DBG_ALGORITHM_LOG
#define ALGORITHM_EPS                     (2.2204e-16)
#define ALGORITHM_EPS_TYPENAME            (2.2204e-6)//         (2.2204e-5)

#define ALGORITHM_HOMO_EULER_EPS          (1.2e-16)
#define ALGORITHM_HOMO_EULER_EPS_TYPENAME (1.2e-6)  //(1.2e-5)

#define ALGORITHM_PI_EPS                  (5.0e-8)
#define ALGORITHM_PI_EPS_TYPENAME         (5.0e-6)

#define EPS_IK                            (2.3e-15)  // which can little greater than 2.2204e-16
#define EPS_IK_TYPENAME                   (2.3e-6)  //(2.3e-5)   // used in template function

#define JOINTTYPE_PRISMATIC (0)
#define JOINTTYPE_ROTATION  (1)
#define DH_CONVENTION_STD   (0)
#define DH_CONVENTION_CRAIG (1)
#define JOINT_NUM_MAX       (100)
#define TR2RPY_MATRIXTYPE_ROTM (0)
#define TR2RPY_MATRIXTYPE_HOMO (1)
typedef enum tag_rmrc_orient_err_type {
    RMRC_ERRO_EULER = 0,  // Euler angles
    RMRC_ERRO_ANG_AX,     // Angle and axis
    RMRC_ERRO_QUAT        // Unit quaternion
}RMRC_ORIENT_ERR_TYPE;


#ifdef TEST_MODE
#define _180_DIV_PI (180/PI)   // degrees in unit rad; 180/pi
#define PI_DIV_180  (PI/180)  // rad in unit degree; pi/180
#define DEG2RAD_1   (PI/180)  // rad of deg 1; pi/180
#define DEG2RAD_45  (PI/4)  // rad of deg 45;   pi/4
#define DEG2RAD_90  (PI/2)   // rad of deg 90;   pi/2
#define DEG2RAD_135 (PI*0.75)   // rad of deg 135;  pi/4*3
#else
#define _180_DIV_PI (57.2957795131)   // degrees in unit rad; 180/pi
#define PI_DIV_180  (0.01745330)  // rad in unit degree; pi/180
#define DEG2RAD_1   (0.01745330)  // rad of deg 1; pi/180
#define DEG2RAD_45  (0.78539816)  // rad of deg 45;   pi/4
#define DEG2RAD_90  (1.57079633)   // rad of deg 90;   pi/2
#define DEG2RAD_135 (2.35619449)   // rad of deg 135;  pi/4*3
#endif

/* BEGIN: Added by lin.lin, 2017/9/26 */
#ifndef time_val_max
#define timeval_max_cnt (502008+1)  /// core dumped on 512008
                                    /// passed on 502008+1
#endif
#if defined (WIN32) || defined (WIN64)
    #define _UT_INIT_TIME
    #define _UT_START_TIME
    #define _UT_END_TIME
    #define _UT_PRINT_TIME
    #define _UT_UNINIT_TIME
#else
    #define _UT_INIT_TIME                                                                                \
        struct timeval g_timevalTimeStart, g_timevalTimeEnd, g_timevalTimeDiff[timeval_max_cnt] = {0};   \
        int g_iTotalTime = 0, g_iIntervalCnt = 0;

    #define _UT_START_TIME    gettimeofday(&g_timevalTimeStart, NULL);
    #define _UT_END_TIME                                                                                 \
        gettimeofday(&g_timevalTimeEnd, NULL);                                                           \
        time_substract(&g_timevalTimeDiff[g_iIntervalCnt++], &g_timevalTimeStart, &g_timevalTimeEnd);    \
        if ( g_iIntervalCnt > timeval_max_cnt-1 ){                                                       \
            g_iIntervalCnt  = 0;                                                                         \
        }
    #define _UT_PRINT_TIME                                                                               \
    g_iTotalTime = 0;                                                                                    \
    for (int timeval_i = 0 ; timeval_i < g_iIntervalCnt; ++timeval_i ){                                  \
        g_iTotalTime += (int)g_timevalTimeDiff[timeval_i].tv_sec * 1000 + (int)g_timevalTimeDiff[timeval_i].tv_usec; \
        printf("inter time[%d] : %d s, %d us, total %d us\n", timeval_i,                                 \
      (int)g_timevalTimeDiff[timeval_i].tv_sec, (int)g_timevalTimeDiff[timeval_i].tv_usec, g_iTotalTime);\
    }
    #define _UT_UNINIT_TIME                                                                              \
    g_iTotalTime = g_iIntervalCnt = 0;
#endif

// constraint condition to protect instrument
// MaryLand
#define MARYLAND_PYTOMOTORS_PITCH_MIN  (-70)
#define MARYLAND_PYTOMOTORS_PITCH_MAX  (70)
#define MARYLAND_PYTOMOTORS_YAW_MIN    (-90)
#define MARYLAND_PYTOMOTORS_YAW_MAX    (90)
#define MARYLAND_PYTOMOTORS_CLIP_MAX   (120)                     // left or right bounary of max clip; 1/2 of max_clip;
#define MARYLAND_PYTOMOTORS_CLIP_CRI   (5)                       //the critical value of clip
#define MARYLAND_PYTOMOTORS_CLIP_MIN   (-5)

// Mono
#define MONO_PYTOMOTORS_PITCH_MIN  (-85)
#define MONO_PYTOMOTORS_PITCH_MAX  (85)

#define MONO_PYTOMOTORS_YAW_MIN    (-100)
#define MONO_PYTOMOTORS_YAW_MAX    (100)

#define MONO_PYTOMOTORS_CLIP_MAX   (32)
#define MONO_PYTOMOTORS_CLIP_CRI   (0)                  //the critical value of clip
#define MONO_PYTOMOTORS_CLIP_MIN   (-0)

// Mega
#define MEGA_PYTOMOTORS_PITCH_MIN  (-85)
#define MEGA_PYTOMOTORS_PITCH_MAX  (85)
#define MEGA_PYTOMOTORS_YAW_MIN    (-115)
#define MEGA_PYTOMOTORS_YAW_MAX    (115)
#define MEGA_PYTOMOTORS_CLIP_MAX   (230/2.0)
#define MEGA_PYTOMOTORS_CLIP_CRI   (5)                        //the critical value of clip
#define MEGA_PYTOMOTORS_CLIP_MIN   (-5)

#define MEGA_B_PYTOMOTORS_CLIP_CRI   (10)                     //the critical value of clip
#define MEGA_B_PYTOMOTORS_CLIP_MIN   (-10)
// Large
#define LARGE_PYTOMOTORS_PITCH_MIN  (-80)//(-100)//(-90)//(-80)
#define LARGE_PYTOMOTORS_PITCH_MAX  (80)//(100)//(90) //(80)
#define LARGE_PYTOMOTORS_YAW_MIN    (-110)
#define LARGE_PYTOMOTORS_YAW_MAX    (110)
#define LARGE_PYTOMOTORS_CLIP_MAX   (220/2.0)            // the max clip value is 2 times of the max yaw value.
#define LARGE_PYTOMOTORS_CLIP_CRI   (13)                  //the critical value of clip
#define LARGE_PYTOMOTORS_CLIP_MIN   (-13)

#define LARGE_MOTORS2PY_MOTOR1_MAX  (LARGE_PYTOMOTORS_PITCH_MAX+1)
#define LARGE_MOTORS2PY_MOTOR1_MIN  (LARGE_PYTOMOTORS_PITCH_MIN-1)
#define LARGE_MOTORS2PY_MOTOR2_MAX  (90 )
#define LARGE_MOTORS2PY_MOTOR2_MIN  (-90)
#define LARGE_MOTORS2PY_MOTOR3_MAX  (90 )
#define LARGE_MOTORS2PY_MOTOR3_MIN  (-90)

#define PrintTotalTime printf("total time : %d us\n", g_iTotalTime);

#define NEW_INSTR_DH
#define NEW_INSTR_DH_GEAR

/* END:   Added by lin.lin, 2017/9/26   PN: */

#define SURGNOVA_PYTOMOTORS_CLIP_MAX 124
#define SURGNOVA_PYTOMOTORS_CLIP_MIN (-15)
#define SURGNOVA_PYTOMOTORS_CLIP_CRI 15

#ifdef TEST_MODE
    #define SQRT(x) sqrt(x)
#else
    #define SQRT(x) sqrt(x)//(1/Algorithm::InvSqrt(x))
#endif
/*----------------------------------------------*
 * routines' implementations                    *
 *----------------------------------------------*/
class classStlSort
{
public:
    classStlSort(){};
    classStlSort(int index, double dFuncValue) : index(index), dFuncValue(dFuncValue){};
    int index;                                              //index
    double dFuncValue;                                      //value of obj function
/* BEGIN: Added by lin.lin, 2017/9/8 */
    double yaw;  //unit: deg
    double pitch;//unit: deg
    double clipNewHomo[4][4];
    double flangeNewHomo[16];
/* END:   Added by lin.lin, 2017/9/8   PN: */

    bool operator < (const classStlSort &m)const
    {
        return dFuncValue < m.dFuncValue;
    }
    // user custom compared function
    static bool cmpAscend(const classStlSort & m1, const classStlSort & m2)
    {
        //cout << "bool less(const classStlSort & m1, const classStlSort & m2) " << endl;
        return m1.dFuncValue < m2.dFuncValue;
    }
    static bool cmpDescend(const classStlSort & m1, const classStlSort & m2)
    {
        return m1.dFuncValue > m2.dFuncValue;
    }
};

class Algorithm
{
public:
    //input : one two three
    //output : trans
    static void KukaAndWorldTransmatrix(const Point& one,const Point& two,const Point& three,double* trans);
    //input :motor1Position motor2Position motor3Position motor1HomePosition motor2HomePosition motor3HomePosition flange
    //output :instrmentFrame clip
    static void GetInstrumentFrame(const double& motor1Position,const double& motor2Position,const double& motor3Position,
                                    const double& motor1HomePosition,const double& motor2HomePosition,const double& motor3HomePosition,const Frame& flange,Frame& instrumentFrame,double& clip);
    //input : instrumentFrameInSlave factor
    //output : return value
    static Frame LaparoToEyeMapping(const Frame& instrumentFrameInSlave,const double& factor);
    //input : clipFrameInMaster factor
    //output : return value
    static Frame EyeToLaparoMapping(const Frame& clipFrameInMaster,const double& factor);
    //input : inputLeft inputRight
    //output : outputLeft outputRight
    static void SpaceConvert(const Frame& inputLeft,const Frame& inputRight,Frame& outputLeft,Frame& outputRight);
    //input : inputLeft inputRight
    //output : outputLeft outputRight
    static void SpaceReconvert(const Frame& inputLeft,const Frame& inputRight,Frame& outputLeft,Frame& outputRight);
    //input : clipFrame trocarPosition shalftLength jointLength
    //output : pitch roll flangeFrame
    static void KinematicMapping(const Frame& clipFrame,const Vector& trocarPosition,const double& shalftLength,const double& jointLength,double& yaw,double& pitch,double& roll,Frame& flangeFrame);
    //input : yaw pitch clip motor1HomePosition motor2HomePotision motor3HomePosition
    //output : motor1 motor2 motor3
    static void RPYToMotors(const double& yaw,const double& pitch,const double& clip,const double& motor1HomePosition,const double& motor2HomePosition,const double& motor3HomePosition,double& motor1,double& motor2,double& motor3);

template<typename T>
    static void MARYLAND_PYToMotors(const T yaw, const T pitch,  T& clip,
                            T& motor1, T& motor2, T& motor3);
template<typename T>
    static void MEGA_PYToMotors(const T yaw, const T pitch,  T& clip,
                            T& motor1, T& motor2, T& motor3);
template<typename T>
    static void MONO_PYToMotors(const T yaw, const T pitch,  T& clip,
                            T& motor1, T& motor2, T& motor3);
template<typename T>
    static void Large_PYToMotors(const T yaw, const T pitch,  T& clip,
                T& motor1, T& motor2, T& motor3);


    static void Large_Motors2PY(double dMotor1, double dMotor2, double dMotor3,
                                double& dYaw, double& dPitch,  double& dClip);
    static void MEGA_MotorsToPY(const double motor1, const double motor2, const double motor3,
                double& yaw, double& pitch,  double& clip);
    static void MARYLAND_MotorsToPY(const double motor1, const double motor2, const double motor3,
                double& yaw, double& pitch,  double& clip);

    static void Surgnova_PYToMotors(const double yaw, const double pitch, const double clip,
        double& motor1, double& motor2, double& motor3, const double gripDegree = 15.0);

    static void Surgnova_PYToMotors_velocity(const double yaw, const double pitch, const double clip,
        double& motor1, double& motor2, double& motor3);

    static void Surgnova_MotorsToPY(const double motor1, const double motor2, const double motor3,
        double& yaw, double& pitch, double& clip);

    static void NewSurgnova_PYToMotors(const double yaw, const double pitch, const double clip,
        double& motor1, double& motor2, double& motor3, const double gripDegree = 15.0);

    static void NewSurgnova_PYToMotors_velocity(const double yaw, const double pitch, const double clip,
        double& motor1, double& motor2, double& motor3);

    static void NewSurgnova_MotorsToPY(const double motor1, const double motor2, const double motor3,
        double& yaw, double& pitch, double& clip);
    static void CartesianVelocityToJointVelocity(Eigen::Matrix4d m1, Eigen::Matrix4d m2, double JointVelocity[]);
/* BEGIN: Added by Linl, 2017/3~4 */
    static double traj(double t, double ua, double ub, double va, double vb, double ta, double tb);
    static double traj_cubic_a0(double ua, double ub, double va, double vb, double ta, double tb);
    static double traj_cubic_a1(double ua, double ub, double va, double vb, double ta, double tb);
    static double traj_cubic_a2(double ua, double ub, double va, double vb, double ta, double tb);
    static double traj_cubic_a3(double ua, double ub, double va, double vb, double ta, double tb);
    static double traj_cubic_u(double t, double a0, double a1, double a2, double a3);
    static double traj_cubic_v(double t, double a1, double a2, double a3);
    static Eigen::Vector3d traj_cubic_a0(Eigen::Vector3d ua, Eigen::Vector3d ub, Eigen::Vector3d va, Eigen::Vector3d vb, double ta, double tb);
    static Eigen::Vector3d traj_cubic_a1(Eigen::Vector3d ua, Eigen::Vector3d ub, Eigen::Vector3d va, Eigen::Vector3d vb, double ta, double tb);
    static Eigen::Vector3d traj_cubic_a2(Eigen::Vector3d ua, Eigen::Vector3d ub, Eigen::Vector3d va, Eigen::Vector3d vb, double ta, double tb);
    static Eigen::Vector3d traj_cubic_a3(Eigen::Vector3d ua, Eigen::Vector3d ub, Eigen::Vector3d va, Eigen::Vector3d vb, double ta, double tb);
    static Eigen::Vector3d traj_cubic_u(double t, Eigen::Vector3d a0, Eigen::Vector3d a1, Eigen::Vector3d a2, Eigen::Vector3d a3);
    static Eigen::Vector3d traj_cubic_v(double t, Eigen::Vector3d a1, Eigen::Vector3d a2, Eigen::Vector3d a3);
    static Eigen::Vector3d traj_cubic_a(double t, Eigen::Vector3d a2, Eigen::Vector3d a3);
    static Eigen::Matrix<double, 7, 1> traj_cubic_a0(Eigen::Matrix<double, 7, 1> ua, Eigen::Matrix<double, 7, 1> ub, Eigen::Matrix<double, 7, 1> va, Eigen::Matrix<double, 7, 1> vb, double ta, double tb);
    static Eigen::Matrix<double, 7, 1> traj_cubic_a1(Eigen::Matrix<double, 7, 1> ua, Eigen::Matrix<double, 7, 1> ub, Eigen::Matrix<double, 7, 1> va, Eigen::Matrix<double, 7, 1> vb, double ta, double tb);
    static Eigen::Matrix<double, 7, 1> traj_cubic_a2(Eigen::Matrix<double, 7, 1> ua, Eigen::Matrix<double, 7, 1> ub, Eigen::Matrix<double, 7, 1> va, Eigen::Matrix<double, 7, 1> vb, double ta, double tb);
    static Eigen::Matrix<double, 7, 1> traj_cubic_a3(Eigen::Matrix<double, 7, 1> ua, Eigen::Matrix<double, 7, 1> ub, Eigen::Matrix<double, 7, 1> va, Eigen::Matrix<double, 7, 1> vb, double ta, double tb);
    static Eigen::Matrix<double, 7, 1> traj_cubic_u(double t, Eigen::Matrix<double, 7, 1> a0, Eigen::Matrix<double, 7, 1> a1, Eigen::Matrix<double, 7, 1> a2, Eigen::Matrix<double, 7, 1> a3);
    static Eigen::Matrix<double, 7, 1> traj_cubic_v(double t, Eigen::Matrix<double, 7, 1> a1, Eigen::Matrix<double, 7, 1> a2, Eigen::Matrix<double, 7, 1> a3);
    static Eigen::Matrix<double, 7, 1> traj_cubic_a(double t, Eigen::Matrix<double, 7, 1> a2, Eigen::Matrix<double, 7, 1> a3);
    static Eigen::Matrix<double, 10, 1> traj_cubic_a0(Eigen::Matrix<double, 10, 1> ua, Eigen::Matrix<double, 10, 1> ub, Eigen::Matrix<double, 10, 1> va, Eigen::Matrix<double, 10, 1> vb, double ta, double tb);
    static Eigen::Matrix<double, 10, 1> traj_cubic_a1(Eigen::Matrix<double, 10, 1> ua, Eigen::Matrix<double, 10, 1> ub, Eigen::Matrix<double, 10, 1> va, Eigen::Matrix<double, 10, 1> vb, double ta, double tb);
    static Eigen::Matrix<double, 10, 1> traj_cubic_a2(Eigen::Matrix<double, 10, 1> ua, Eigen::Matrix<double, 10, 1> ub, Eigen::Matrix<double, 10, 1> va, Eigen::Matrix<double, 10, 1> vb, double ta, double tb);
    static Eigen::Matrix<double, 10, 1> traj_cubic_a3(Eigen::Matrix<double, 10, 1> ua, Eigen::Matrix<double, 10, 1> ub, Eigen::Matrix<double, 10, 1> va, Eigen::Matrix<double, 10, 1> vb, double ta, double tb);
    static Eigen::Matrix<double, 10, 1> traj_cubic_u(double t, Eigen::Matrix<double, 10, 1> a0, Eigen::Matrix<double, 10, 1> a1, Eigen::Matrix<double, 10, 1> a2, Eigen::Matrix<double, 10, 1> a3);
    static Eigen::Matrix<double, 10, 1> traj_cubic_v(double t, Eigen::Matrix<double, 10, 1> a1, Eigen::Matrix<double, 10, 1> a2, Eigen::Matrix<double, 10, 1> a3);
    static Eigen::Matrix<double, 10, 1> traj_cubic_a(double t, Eigen::Matrix<double, 10, 1> a2, Eigen::Matrix<double, 10, 1> a3);

    static void trajEx(double t, int dimension, double theta_start[], double theta_end[], double thetadot_mid[], double tf, double theta[], double omega[]);
template<typename T>
    static void axis2rot(const T theta, const T* pk, T* pRotMatrix);
    static void axis2rot(const double theta, const double pk[], double pRotMatrix[]);
    static void rot2axis(double rotMatrix[], double& theta, double k[]);
    static void trajplan_cartesian(double T_start[], double T_end[], double total_time, double t,
                            double R_t_tool[], double Pos_t_tool[]);
    //overload trajplan_cartesian
    static void trajplan_cartesian(double T_start[], double T_end[], double total_time, double t,
                            Frame &frame);
    //input : clipFrame trocarPosition shalftLength jointLength
    //output : roll pitch flangeFrame
    static void IK(
        const Frame& clipFrame,const Vector& trocarPosition,
        const double& shalftLength,const double& clipLength,const double& jointLength,
        double& pitch,double& yaw,Frame& flangeFrame);
/* END:   Added by lin.lin, 2017/3~4   PN: */
template<typename T>
    static void getPitchYawByGeo(const T* clipHomo, const T* pdTrocarPoint,
      const T shaftLength, const T clipLength, const T jointLength,
      T& pitch, T& yaw, T& pitch2, T& yaw2, T* pDataOut=NULL);

    static void IK_ex(const double* clipHomo, const double* pdTrocarPoint,
        const double shaftLength, const double clipLength, const double jointLength, double *pDH_param, const int nDHparamRow,
        double& pitch, double& yaw, double* dHomoFlange, double& pitch2, double& yaw2, double* dHomoFlange2=NULL, double* pDataOut=NULL);

    static void IK_ex(
        const Frame& clipHomo, const Vector& trocarPoint,
        const double& shaftLength, const double& clipLength, const double& jointLength,
        double& pitch, double& yaw, Frame& flangeFrame, double* pDataOut = 0);

    static void fkine(const double* a,const double* d,const double* alpha,const double* theta,Frame& frame);

    static Eigen::Matrix<double, 6, 1> NewSurgnova_IK_6Joints(Eigen::Matrix<double, 4, 4> clipHomo, Eigen::Vector3d trocarPosition);

template<typename T>
    static void homogeneous4x4Invers(const T* pdSrc, T* pdDst);
template<typename T>  // inverse of rotation matrix
    static void rotMatrix3x3Inv  (const T* pdSrc, T* pdDst);
template<typename T>
    static void matrix3x1Multiply(const T* pdA, const T* pdB, T* pdC);
template<typename T>
    static void matrix3x3Multiply(const T* pdA, const T* pdB, T* pdC);
template<typename T>
    static void matrix4x4Multiply(const T* pdA, const T* pdB, T* pdC);



    static void homoFrmMultiplyFrm(const double* pdA, const Frame& frmB, Frame& frmC) ;
    static void homohomoMultiplyFrm(const double* pdA, const double* pdB, Frame& frmC) ;
    static void frameHomoMultiply(const Frame& frmA, const double* pdB, Frame& frmC) ;
    static void ortHomoMultiplyHomo(const double* pOrt, const double* pHomoIn, double* pHomoOut) ;
    static void homoOrtMultiplyOrt(const double* pHomo, const double* pOrtIn, double* pOrtOut) ;

    static void homo2SevenData(const double* pSrcHomoData, float* pDstSevenData) ;
    static void homo2SevenData(const double* pSrcHomoData, double* pDstSevenData) ;
    static void homo2Quat(const double* pSrcHomoData, double* pDstQuatData) ;
    static void sevenData2Homo(const float* pSrcSevenData, double* pDstHomoData) ;
    static void sevenData2Homo(const double* pSrcSevenData, double* pDstHomoData) ;

    static void framePosAndOrtTrans(const Frame& frmSrc, const double dZoomPreTrans, const double* pdTransHomogeneous, const double dZoomPostTrans, double* pdDstFrameHomo) ;
    static void framePosAndOrtTrans(const Frame& frmSrc, const double dZoomPreTrans, const double* pdTransHomogeneous, const double dZoomPostTrans, Frame& frmDst) ;

    static void computeHaption2EyeTransform(const double* dKuka2EyeTrans, const double* dKuka2HaptionTrans, double* dHaption2EyeTrans) ;

    static void kukaTransFromEuler(double x, double y, double z, double aDeg, double bDeg, double cDeg, double trans[][4]);
    static int  prepareTrajPlanCubicParam(const Frame& frmStart, const Frame& frmEnd, double dSpeedStart, double dSpeedEnd, double dDuration, double* pStartQuat, double* pEndQuat, double* pCubicParamC, int* pIsSlerpLinear, double* pSlerpTheta, double* pSlerpInvSinTheta) ;
    static int  prepareTrajPlanCubicParam(const double* pSevenDataStart, const double* pSevenDataEnd, double dSpeedStart, double dSpeedEnd, double dDuration, double* pCubicParamC, int* pIsSlerpLinear, double* pSlerpTheta, double* pSlerpInvSinTheta, double* pQuatEndOut) ;
    static int  trajPlanCartesianCubic(const double* pCubicParamC, double dCurTime, double dDuration, const double* pStartQuat, const double* pEndQuat, int iIsSlerpLinear, double dSlerpTheta, double dSlerpInvSinTheta, Frame& frmNext, double* pSpeedNext) ;
    static int  trajPlanCartesianCubic(const double* pCubicParamC, double dCurTime, double dDuration, const double* pStartQuat, const double* pEndQuat, int iIsSlerpLinear, double dSlerpTheta, double dSlerpInvSinTheta, double* pSevenDataNext, double* pSpeedNext) ;

    static int  prepareTrajPlanCubicParam(const Frame& frmStart, const Frame& frmEnd, double dSpeedStart, double dSpeedEnd, double dDuration, double* pCubicParamC, double* pRotAxisVector) ;

    static int  trajPlanCartesianCubic(const double* pCubicParamC, double dCurTime, const double* pAxisVector, const double* pRotStart, Frame& frmNext, double* pSpeedNext) ;

    static int  prepareTrajPlanCubicParam(double dPosStart, double dPosEnd, double dSpeedStart, double dSpeedEnd, double dDuration, double* pCubicParamC) ;

    static int  trajPlanCartesianCubic(const double* pCubicParamC, double dCurTime, double* pPosNext, double* pSpeedNext) ;

    static int  prepareMotionFuseLinear(const double* pHomoA, const double* pHomoB, double* pOutDeltaPos, double* pOutDeltaAxisVector, double* pOutDeltaAxisTheta) ;

    static int  motionFuseLinear(const double* pHomoA, const double* pDeltaPos, const double* pDeltaAxisVector, double dDeltaAxisTheta, double dCurTick, double* pOutHomoB) ;

    static int  prepareMotionFuseCubic(const double* pHomoA, const double* pHomoB, const double dDuration, double* pOutDeltaAxisVector, double* pOutDeltaAxisTheta, double* pOutTransOrt, double* pCubicParam);

    static int  motionFuseCubic(const double* pHomoA, const double* pDeltaAxisVector, const double* pDeltaTransOrt, const double* pCubicParam, double dCurTick, double* pOutHomoB) ;
    static void FK(const Frame& flangeFrame,
        const double& shalftLength,const double& clipLength,const double& jointLength,
        double pitch,double yaw,Frame& clipFrame
    );
template<typename T>
    static void FK_DH(const T* parOrigin, const T* pDH_param, const int nRow, T* parrDest, const int nFlag = DH_CONVENTION_STD);//recommend!
    static void FK_DH(const Frame& flangeFrame, const double& shaftLength,const double& clipLength,const double& jointLength,
        double pitch,double yaw,Frame& clipFrame);
    static void FK_DH(const Frame& OriginFrame,     const double DH_param[][4], const int nRow, Frame& DestFrame);

    //not recommanded, use FK_DH instead.
    static void FK_DH_Crag(const double* parOrigin, const double* pDH_param, const int nRow, double* parrDest) ;
    static void FK_DH_Crag(const Frame& flangeFrame, const double& shaftLength, const double& clipLength, const double& jointLength, double pitch, double yaw, Frame& clipFrame) ;

template<typename T>
    static void IK_DH(const T* parrEndHomo, const T* pDH_param, const int nRow, T* parrStartHomo, const int nFlag = DH_CONVENTION_STD);
    static void IK_DH(const double* parrEndHomo, const double* pDH_param, const int nRow, double* parrStartHomo);
    /* BEGIN: Added by lin.lin, 2017/8/14 */
template<typename T>
    static void eye(T* arrd, int nOrder=1);
    static void eye(double* arrd, int nOrder=1);

template<typename T>
    static T      QuatDiff(const T* qSrc1, const T* qSrc2);
    static double QuatDiff(const double* qSrc1, const double* qSrc2);
    static double QuatDiff(const CQuat &qSrc1,  const CQuat &qSrc2, CQuat &q2invq1);
    static double QuatDiff(const CQuat &q1,     const CQuat &q2);
template<typename T>
    static void   getErrO(const T* tarrQd, const T* tarrQe, T* ptErrO);
template<typename T>
    static T      getErrO(const T* tarrQd, const T* tarrQe);
template<typename T>
    static      T mean(T* tarrSrc, int nTotalNum);
template<typename T>
    static      T StandardDeviation(T* tarrSrc, int nTotalNum);
template<typename T>
    static int    getMeanAndStd(const OBJ_FUNC<T>* pObj_func, const int nTotalNum,
                                    T& tPosFuncMean, T& tRotFuncMean, T& tPosFuncStd, T& tRotFuncStd);
    static void   sort(const double* pdSrcData, int nTotalNumber, int* pnSortIndex, double* pdSortData, int iFlag=ALG_SORT_ASCEND);
    static void   sort(vector< classStlSort > &vect);
    /* END:   Added by lin.lin, 2017/8/14   PN: */

/* BEGIN: Added by lin.lin, 2017/9/6 */

template<typename T>
    static void findNearest(const T* tarrDestClipHomo, const T* tarrTrocarPoint, const T shaftLength,  const T clipLength, const T jointLength,
        const T tMinYaw, const T tMaxYaw, const T tMinPitch, const T tMaxPitch, const T tMinClip, const T tMaxClip, T dAdjustShpereR,
        T &tSrcYawDeg, T &tSrcPitchDeg, T* tarrOptFlangeHomo, T* tarrAdjustPitchPoint, T* pOutData=NULL);

    static float InvSqrt(float number);

/* END:   Added by lin.lin, 2017/9/6   PN: */
    static int   avgHomo(const double* pHomoSrc, const double* pWeight, int iHomoCnt, double* pHomoDst) ;
    static int   avgFrame(const Frame* pFrameArray, const double* pWeight, int iFrameCnt, Frame& pFrameDst) ;
    static int   avgQuaternion(const double* pQuaternionSrc, const double* pWeight, int iQuatCnt, double* pQuaternionDst) ;
    static int   avgQuaternionXYZW(const double* pQuaternionSrc, const double* pWeight, int iQuatCnt, double* pQuaternionDst) ;
    static int   eign4x4(double* pMatrix, double* pOutVec) ;
    static int   eign4x4MaxVec(double* pMatrix, double* pOutVec, double* pMaxVec) ;
    static int   suppressFrame(const Frame& frmSrc, const Frame& frmDst, double dMaxPos, double dMaxOrtTheta, double dMaxThetaZ, Frame& frmOut) ;

template<typename T>
    static void  homo2RPY(const T* pHomo, T& dRoll, T& dPitch, T& dYaw);
    static void  homo2RPY(const double* pHomo, double& dRoll, double& dPitch, double& dYaw);
    static int   homo2RPY(const double* pHomo, double* pRoll, double* pPitch, double* pYaw);

template<typename T>
    static void  rpy2Homo(T dRoll, T dPitch, T dYaw, T* pHomoDst);
    static void  rpy2Homo(double dRoll, double dPitch, double dYaw, double* pHomoDst) ;
template<typename T>
    static void  DHfromPostposition2Homo(const T alpha, const T a, const T d, const T theta, T* pHomoDst);
template<typename T>
    static void  DhModified2Homo(const T alphaRad, const T a, const T d, const T thetaRad, T* pHomoDst);
    static void  DHfromPreposition2Homo(const double alpha, const double a, const double d, const double theta, double* pHomoDst) ;// see DhModified2Homo

    static int   svd(double* a, const int m, const int n, double* u, double* v, const double eps, const int ka);
    static Eigen::Matrix<double, 7, 6> pinv(Eigen::Matrix<double, 6, 7> mtrxSrc);
    static int pinv(const double* tarrM, const int iRow, const int iCol, double* tarrPinvM);
    static int   pinv(double a[], int m, int n, double aa[], double eps, double u[], double v[], int ka);
    static double cond(const double* ptA, const int m, const int n, const int p=2);
    static void fk_dh_6joint(const double* theta, double* homo);
    static void fk_dh_7joint(const double* theta, const double dft, double* homo);
    static void getJacobianEx_6joint(const double* theta, double* homo);
    static void getJacobianEx_7joint(const double* theta, const double dft, double* homo);
    static double getGainOfJacobTranspose(const double* J, const int iRow, const int iColumn, const double* e);
    static int getJointVelocityByJT(const double* desiredCartesianVector, double* q, double* dq);
    static Matrix<double, 6, 1> getGainOfJacobTranspose(Matrix<double, 6, 6> J, Matrix<double, 6, 1> e);
    static Matrix<double, 6, 1> getDqByJacobTranspose(Matrix<double, 6, 1> K, Matrix<double, 6, 6> J, Matrix<double, 6, 1> e);
template<typename T>
    static void tr2rpy(const T* ptHomo, T* ptRpyRad, int nMatrixType=TR2RPY_MATRIXTYPE_ROTM);
template<typename T>
    static void rpy2r(T tRoll, T tPitch, T tYaw, T* ptHomoDst);

template<typename T>
    static void rpy2jac(const T tRollRad, const T tPitchRad, const T tYawRad, T* ptB);
template<typename T>
    static void t2r(const T* ptHomo, T* ptR);
template<typename T>
    static void r2t(const T* ptR, T* ptHomo);
template<typename T>
    static void jacob0(const T* ptDH_param, const int nJointNum, const int nDof, const int* pnJointType, const int nDhConvention, T* ptJ, T* ptLog);
template<typename T>
    static void  IK_rmrc(const T *pt_dxd, const T *pt_qs, const T *tarr_K, const int nCase, const T *pt_xd, const T *tarrSampleTimeSec,
                            T *pt_qe, T *pt_xe, T *pt_log);
template<typename T>
    static void  IK_rmrc(
                            const T *pt_xd,              const T *ptDotXdesire,      const T  *ptQsRad,
                            const T  tarrSampleTimeSec,  const T *tarr_K,            const int nCase,
                            T* DH_param,
                            T *ptQeRad,                        T *tarrXe,                  T  *tarrErr,

                            const int iDhConvention=DH_CONVENTION_CRAIG,
                            T *tarrQim=NULL, T *tarrQiM=NULL, const int nJointNum=7,
                            const RMRC_ORIENT_ERR_TYPE iErrOrientType=RMRC_ERRO_EULER,
                            T *pt_log=NULL);

template<typename T>
    static void vex(const T *ptS, T* ptV);
template<typename T>
    static void skew(const T* ptV, T* ptS, const int iDim=3);
template<typename T>
    static void skew(const T tq, T* ptS);

template<typename T>
    static void tr2delta(const T *ptT0, const T* ptT1, T* ptDelta);
template<typename T>
    static void transl(const T *ptT0, T* ptTdst);
template<typename T>
    static void eul2rotm(T* pEuler, T* ptRotm);

template<typename T>
    static  void normrnd(T aver, T sigma, int row, int column, T* p);
template<typename T>
    static int jcbi (T* a, const int n, T* v, const T eps, const int jt);
template<typename T>
    static void jcbj(T* a, const int n, T* v, const T eps);
template<typename T>
    static T min(const T* pdSrcData, const int nSize, int* pIndex=NULL);
template<typename T>
    static T max(const T* pdSrcData, const int nSize, int* pIndex=NULL);
template<typename T>
    static int gmiv(T* a, int m, int n, T* b, T* x, T* aa, T eps, T* u, T* v, int ka);

template<typename T>
    static  void swap(T *a, T *b);
template<typename T>
    static void getDlsFactors(const T tMaxLamda, const T tEpsilon, const T *tarrSigma, const int iSigNum, T *tarrLamda);
template<typename T>
    static void getMatrixWithDamp(T* tarrSymMatrix, const int iMatrixDim, const T tMaxLamda, const T tEpsilon=0.05);
template<typename T>
    static int MatrixRank(T* tarrMatrix, int m, int n);
template<typename T>
    static void diag(const T* tarrVecW, const int iDim, T* tarrW);
template<typename T>
    static void getWeightRightPinv(const T* tarrM, const int iRow, const int iCol, T* tarrPinvM, T* tarrVecW=NULL);
template<typename T>
    static void getWeightLeftPinv(const T* tarrM, const int iRow, const int iCol, T* tarrPinvM, T* tarrVecW=NULL);
template<typename T>
    static  int MatrixInv(const T* tarrA, const int row, const int column, T* p);
    static double* eul2rotm(double Euler[]);
    static bool  checkJointLimitSuccess(const Joint &joint);
    static void  QuatMultiply(const float* pQuatFirst, const float* pQuatSecond, float* pQuatResult) ;
    static void  QuatMultiply(const double* pQuatFirst, const double* pQuatSecond, double* pQuatResult) ;
    static void  QuatConj(const float* pQuatSrc, float* pQuatConj) ;
    static void  QuatConj(const double* pQuatSrc, double* pQuatConj) ;
    static void  QuatDiff(const float* pQuatFirst, const float* pQuatSecond, float* pQuatDiff) ;
    static void  QuatDiff(const double* pQuatFirst, const double* pQuatSecond, double* pQuatDiff);

PRIVATE:
template<typename T>
    static void CreateAdjustedVectors(const T* darrSphereCenterV, const T dSphereR, T** ppdNeighbourVectTrans);
    static void CreateAdjustedVectors(const double* darrSphereCenterV, const double dSphereR, double** ppdNeighbourVectTrans);
template<typename T>
    static void CreateAdjustedVectorsEx(const T* darrSphereCenterV,  const T dSphereR, T** ppdNeighbourVectTrans);
template<typename T>
    static void CreateAdjustedVectorsEx(const T* tarrSphereCenterV,  const T tSphereR,
                                        const T  tLongtitudeStepDeg, const T tLatitudeStepDeg,
                                        T**      pptNeighbourVectTrans);
template<typename T>
    static void GetWeight(const T** pdarrHomoSample, const int nLen, T& dMiu1, T& dMiu2, T& dMean1, T& dMean2, T* pOutData=NULL);
template<typename T>
    static void GetPosWeight(const T** pptarrHomoSample, const int nAdjustSampleNum, T& dPosMiu, T* pOutData);
template<typename T>
    static void AdjustEx(const T* pTrocarOldHomo, const T* pitchOldV,  const T* pitchNewV,
                             const T Roll,            const T* pDH_param,  const int nRow,
                             T* pEndowristNewHomo,    T* pFlangeNewHomo);
template<typename T>
    static void AdjustEx(const T* pTrocarOldHomo, const T* pitchOldV,  const T* pitchNewV,
                            const T Roll,            const T* pDH_param,
                            T* pEndowristNewHomo,    T* pPitchNewHomo,    T* pYawNewHomo, T* pFlangeNewHomo);

template<typename T>
    static T      GetObjFuncValue(const T funcPos, const T* quatSrc, const T* quatDest, const T tRotMiu, const T tPosMiu, T& dQuatFunc);
template<typename T>
    static T      GetObjFuncValue(const T funcPos, const T* quatSrc, const T* quatDest, const T tRotMiu, const T tPosMiu);
template<typename T>
    static T      GetObjFuncValue(const T* tarrQuatSrc,      const T* tarrPosSrc,      const T* tarrQuatDest,      const T* darrPosDest,      const T tRotMiu,    const T tPosMiu);


template<typename T>
    static void   GetTrocarFrame(const T* trocarOldHomo,      const T*      pitchOldV, const T*      pitchNewV, const T Roll, T* trocarNewHomo);

// from GeometryUtils
template<typename T>
    static T     Norm(const T arrd[3]);
template<typename T>
    static T     Distance(const T a[3], const T b[3]);

template<typename T>
    static T    vectorDot  (const T arrd1[3],    const T arrd2[3]);
template<typename T>
    static void vectorCross(const T arrdSrcA[3], const T arrdSrcB[3], T arrdRelsult[3]);

template<typename T>
    static void rotateX(const T angle, T* result);
template<typename T>
    static void rotateY(const T angle, T* result);
template<typename T>
    static void rotateZ(const T angle, T* result);
template<typename T>
    static void transX2Homo(const T x, T* pResult);


// from CQuat
template<typename T>
    static void QuatMul(const T* pQuatFirst, const T* pQuatSecond, T* pQuatResult);
template<typename T>
    static void QuatMul(const T* pQuat, T* pQuatResult);
template<typename T>
    static void Conjugate(const T* qSrc, T* qDst);
template<typename T>
    static void rotation2Quat(const T* m, T* q);
template<typename T>
    static T MovingAverageFilter(const T tSrcData, const int nWidth, T* tarrValueBuf);
template<typename T>
    static void GramSchmidtOrth(T *pDstHomoData );

private: // the group of member functions will be discarded.
    //static void   GetWeight(const classStlSort *obj,  double& u1, double& u2, const int nSampleSize = 405);
    static void   ppp(double a[], double e[], double s[], double v[], int m, int n);
    static void   sss(double fg[2], double cs[2]);
    static bool   cmpDescend(const classStlSort & m1, const classStlSort & m2);
};

// public:
//inverse of a homoMatrix

template<typename T>
inline void Algorithm::homogeneous4x4Invers(const T* pdSrc, T* pdDst)
{
    pdDst[0]  = pdSrc[0] ;
    pdDst[1]  = pdSrc[4] ;
    pdDst[2]  = pdSrc[8] ;
    pdDst[4]  = pdSrc[1] ;
    pdDst[5]  = pdSrc[5] ;
    pdDst[6]  = pdSrc[9] ;
    pdDst[8]  = pdSrc[2] ;
    pdDst[9]  = pdSrc[6] ;
    pdDst[10] = pdSrc[10] ;
    pdDst[12] = 0.0 ;
    pdDst[13] = 0.0 ;
    pdDst[14] = 0.0 ;
    pdDst[15] = 1.0 ;
    pdDst[3]  = -(pdSrc[0] * pdSrc[3] + pdSrc[4] * pdSrc[7] + pdSrc[8] * pdSrc[11]) ;
    pdDst[7]  = -(pdSrc[1] * pdSrc[3] + pdSrc[5] * pdSrc[7] + pdSrc[9] * pdSrc[11]) ;
    pdDst[11] = -(pdSrc[2] * pdSrc[3] + pdSrc[6] * pdSrc[7] + pdSrc[10] * pdSrc[11]) ;
}
template<typename T>  // inverse of rotation matrix
inline void Algorithm::rotMatrix3x3Inv(const T* pdSrc, T* pdDst)
{
    pdDst[0]  = pdSrc[0] ;
    pdDst[1]  = pdSrc[3] ;
    pdDst[2]  = pdSrc[6] ;
    pdDst[3]  = pdSrc[1] ;
    pdDst[4]  = pdSrc[4] ;
    pdDst[5]  = pdSrc[7] ;
    pdDst[6]  = pdSrc[2] ;
    pdDst[7]  = pdSrc[5] ;
    pdDst[8]  = pdSrc[8] ;
}
/*****************************************************************************
 Prototype    : matrix3x1Multiply
 Description  : get the product matrix C(3*1) of matrix A(3*3) and matrix B(3*1)
 Input        : const T* pdA : Left multiplied matrix
                const T* pdB : Right multiplied matrix
 Output       : T* pdC       : the product matrix
 Return Value : inline
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/10/20
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
inline void Algorithm::matrix3x1Multiply(const T* pdA, const T* pdB, T* pdC)
{
     pdC[0] = pdA[0] * pdB[0]  +  pdA[1] * pdB[1]  +  pdA[2] * pdB[2];
     pdC[1] = pdA[3] * pdB[0]  +  pdA[4] * pdB[1]  +  pdA[5] * pdB[2];
     pdC[2] = pdA[6] * pdB[0]  +  pdA[7] * pdB[1]  +  pdA[8] * pdB[2];
}
/*****************************************************************************
 Prototype    : matrix3x3Multiply
 Description  : get the product matrix C(3*3) of matrix A(3*3) and matrix B(3*3)
 Input        : const T* pdA : Left multiplied matrix
                const T* pdB : Right multiplied matrix
 Output       : T* pdC       : the product matrix
 Return Value : inline

 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/10/20
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
inline void Algorithm::matrix3x3Multiply(const T* pdA, const T* pdB, T* pdC)
{
    pdC[0] = pdA[0] * pdB[0] + pdA[1] * pdB[3] + pdA[2] * pdB[6] ;
    pdC[1] = pdA[0] * pdB[1] + pdA[1] * pdB[4] + pdA[2] * pdB[7] ;
    pdC[2] = pdA[0] * pdB[2] + pdA[1] * pdB[5] + pdA[2] * pdB[8] ;
    pdC[3] = pdA[3] * pdB[0] + pdA[4] * pdB[3] + pdA[5] * pdB[6] ;
    pdC[4] = pdA[3] * pdB[1] + pdA[4] * pdB[4] + pdA[5] * pdB[7] ;
    pdC[5] = pdA[3] * pdB[2] + pdA[4] * pdB[5] + pdA[5] * pdB[8] ;
    pdC[6] = pdA[6] * pdB[0] + pdA[7] * pdB[3] + pdA[8] * pdB[6] ;
    pdC[7] = pdA[6] * pdB[1] + pdA[7] * pdB[4] + pdA[8] * pdB[7] ;
    pdC[8] = pdA[6] * pdB[2] + pdA[7] * pdB[5] + pdA[8] * pdB[8] ;
}
/*****************************************************************************
 Prototype    : Algorithm.matrix4x4Multiply
 Description  : get the product matrix C(4*4) of matrix A(4*4) and matrix B(4*4)
 Input        : const T* pdA : Left multiplied matrix
                const T* pdB : Right multiplied matrix
 Output       : T* pdC       : the product matrix
 Return Value : inline

 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/10/20
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
inline void Algorithm::matrix4x4Multiply(const T* pdA, const T* pdB, T* pdC)
{
    pdC[0]  = pdA[0] *  pdB[0] + pdA[1] * pdB[4]  + pdA[2] *  pdB[8]  + pdA[3]  * pdB[12] ;
    pdC[1]  = pdA[0] *  pdB[1] + pdA[1] * pdB[5]  + pdA[2] *  pdB[9]  + pdA[3]  * pdB[13] ;
    pdC[2]  = pdA[0] *  pdB[2] + pdA[1] * pdB[6]  + pdA[2] *  pdB[10] + pdA[3]  * pdB[14] ;
    pdC[3]  = pdA[0] *  pdB[3] + pdA[1] * pdB[7]  + pdA[2] *  pdB[11] + pdA[3]  * pdB[15] ;
    pdC[4]  = pdA[4] *  pdB[0] + pdA[5] * pdB[4]  + pdA[6] *  pdB[8]  + pdA[7]  * pdB[12] ;
    pdC[5]  = pdA[4] *  pdB[1] + pdA[5] * pdB[5]  + pdA[6] *  pdB[9]  + pdA[7]  * pdB[13] ;
    pdC[6]  = pdA[4] *  pdB[2] + pdA[5] * pdB[6]  + pdA[6] *  pdB[10] + pdA[7]  * pdB[14] ;
    pdC[7]  = pdA[4] *  pdB[3] + pdA[5] * pdB[7]  + pdA[6] *  pdB[11] + pdA[7]  * pdB[15] ;
    pdC[8]  = pdA[8] *  pdB[0] + pdA[9] * pdB[4]  + pdA[10] * pdB[8]  + pdA[11] * pdB[12] ;
    pdC[9]  = pdA[8] *  pdB[1] + pdA[9] * pdB[5]  + pdA[10] * pdB[9]  + pdA[11] * pdB[13] ;
    pdC[10] = pdA[8] *  pdB[2] + pdA[9] * pdB[6]  + pdA[10] * pdB[10] + pdA[11] * pdB[14] ;
    pdC[11] = pdA[8] *  pdB[3] + pdA[9] * pdB[7]  + pdA[10] * pdB[11] + pdA[11] * pdB[15] ;
    pdC[12] = pdA[12] * pdB[0] + pdA[13] * pdB[4] + pdA[14] * pdB[8]  + pdA[15] * pdB[12] ;
    pdC[13] = pdA[12] * pdB[1] + pdA[13] * pdB[5] + pdA[14] * pdB[9]  + pdA[15] * pdB[13] ;
    pdC[14] = pdA[12] * pdB[2] + pdA[13] * pdB[6] + pdA[14] * pdB[10] + pdA[15] * pdB[14] ;
    pdC[15] = pdA[12] * pdB[3] + pdA[13] * pdB[7] + pdA[14] * pdB[11] + pdA[15] * pdB[15] ;
}
// Corresponding matlab cmd: rotm2eul(pHomo(1:3, 1:3));
template<typename T>
void Algorithm::homo2RPY(const T* pHomo, T& dRoll, T& dPitch, T& dYaw)
{
    //T thetaX, thetaY, thetaZ;
    T angle = pHomo[8];
    //if( angle < 1.0)
    if(1.0 - angle > (ALGORITHM_HOMO_EULER_EPS_TYPENAME))
    {
        //if( angle > -1.0)
        if(angle + 1.0 > ALGORITHM_HOMO_EULER_EPS_TYPENAME)
        {
            dPitch = asin(-angle);
            dRoll  = atan2(pHomo[4], pHomo[0]);
            dYaw   = atan2(pHomo[9], pHomo[10]);
        }
        else
        {
            dPitch = DEG2RAD_90;
            dRoll  = atan2(pHomo[6], pHomo[5]);
            dYaw   = 0.0;
        }
    }
    else
    {
        dPitch = -DEG2RAD_90;
        dRoll  = atan2(-pHomo[6], pHomo[5]);
        dYaw   = 0.0;
    }
    // mapping -180 deg to 180 deg
    if ( (dYaw+PI) < ALGORITHM_PI_EPS_TYPENAME )
    {
        dYaw = PI;
    }
    if ( (dRoll+PI) < ALGORITHM_PI_EPS_TYPENAME )
    {
        dRoll = PI;
    }
}
// Corresponding matlab cmd: eul2rotm([dRoll, dPitch, dYaw]);
// Algorithm::rpy2Homo is equivalent to Algorithm::eul2rotm. Recommend the former
template<typename T>
void Algorithm::rpy2Homo(T dRoll, T dPitch, T dYaw, T* pHomoDst)
{
    T dCx = cos(dYaw);
    T dCy = cos(dPitch);
    T dCz = cos(dRoll);
    T dSx = sin(dYaw);
    T dSy = sin(dPitch);
    T dSz = sin(dRoll);

    pHomoDst[0]  = dCy * dCz ;
    pHomoDst[1]  = dCz * dSx * dSy - dCx * dSz ;
    pHomoDst[2]  = dCx * dCz * dSy + dSx * dSz ;
    pHomoDst[3]  = 0.0;
    pHomoDst[4]  = dCy * dSz ;
    pHomoDst[5]  = dCx * dCz + dSx * dSy * dSz ;
    pHomoDst[6]  = -dCz * dSx + dCx * dSy * dSz ;
    pHomoDst[7]  = 0.0;
    pHomoDst[8]  = -dSy ;
    pHomoDst[9]  = dCy * dSx ;
    pHomoDst[10] = dCx * dCy ;
    pHomoDst[11] = 0.0;
    pHomoDst[12]  = 0.0;
    pHomoDst[13]  = 0.0;
    pHomoDst[14]  = 0.0;
    pHomoDst[15]  = 1.0;
}

// 1, GetTrocarFrame group
/*****************************************************************************
 Prototype    : GetTrocarFrame
 Description  : get trocar frame from adjusted pitch joint.
 Input        : trocarOldHomo : old trocar homo matrix in kuka base coordinate system.
                pitchOldV     : old pitch joint point coordinate in kuka base coordinate system.
                pitchNewV     : adjusted pitch joint point coordinate in kuka base coordinate system.
                Roll          : adjust shaft's rolling angle (radius).
                trocarNewHomo[out] : New trocar homo matrix in kuka base coordinate system.
 Output       : None
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/14
    Author       : lin.lin
    Modification : Created function
  2.Date         : 2017/8/24
    Author       : lin.lin
    Modification : 1, fixed a bug: amended the variable "double ddot" from "int ndot".
                   2, optimized code.
note: 1, called by adjust_ex.
      2, run time: about 2 us, 20180103 version; see _GetTrocarFrame_
      3, see GetTrocarFrame.m
*****************************************************************************/
template<typename T>
void Algorithm::GetTrocarFrame(const T* trocarOldHomo, const T* pitchOldV, const T* pitchNewV, const T Roll,
                    T* trocarNewHomo)
{
    T dEPS              = 1e-15;
    //T dEPS             =  1e-6;
    T trocarOldR[9]    = {0.0};
    T trocarNewR[9]    = {0.0};
    T Vold[3]          = {0.0};
    T Vnew[3]          = {0.0};
    T ddot             = 0.0;                          //dot product
    T theta            = 0.0;
    T R[9]             = {0.0};
    T arrdRotz[9]      = {0.0};
    T arrd3x3Temp[9]   = {0.0};
    T Vcross[3]        = {0.0};
    T dNorm            = 0.0;

    trocarOldR[0] = trocarOldHomo[0];
    trocarOldR[1] = trocarOldHomo[1];
    trocarOldR[2] = trocarOldHomo[2];
    trocarOldR[3] = trocarOldHomo[4];
    trocarOldR[4] = trocarOldHomo[5];
    trocarOldR[5] = trocarOldHomo[6];
    trocarOldR[6] = trocarOldHomo[8];
    trocarOldR[7] = trocarOldHomo[9];
    trocarOldR[8] = trocarOldHomo[10];

    Vold[0] = pitchOldV[0] - trocarOldHomo[3];
    Vnew[0] = pitchNewV[0] - trocarOldHomo[3];
    Vold[1] = pitchOldV[1] - trocarOldHomo[7];
    Vnew[1] = pitchNewV[1] - trocarOldHomo[7];
    Vold[2] = pitchOldV[2] - trocarOldHomo[11];
    Vnew[2] = pitchNewV[2] - trocarOldHomo[11];

    T dNormVnew     = SQRT(Vnew[0]*Vnew[0] + Vnew[1]*Vnew[1]  + Vnew[2]*Vnew[2]);
    T dNormVold     = SQRT(Vold[0]*Vold[0] + Vold[1]*Vold[1]  + Vold[2]*Vold[2]);
    ddot            = vectorDot(Vold, Vnew);
    T tDotNormalize = ddot / (dNormVold * dNormVnew);
/* BEGIN: Added by lin.lin, 2017/12/18 */
    if ( fabs(tDotNormalize-1.0) < dEPS)
    {
        tDotNormalize = 1.0;
        theta = 0;
        R[0]  = 1;
        R[4]  = 1;
        R[8]  = 1;
    }else if ( fabs(tDotNormalize+1.0) < dEPS)
    {
        theta = PI;
        tDotNormalize = -1.0;
        R[0] = -1;
        R[4] = -1;
        R[8] =  1;
    }
    /* END:   Added by lin.lin, 2017/12/18   PN: */
    else
    {
        theta = acos(tDotNormalize);
        vectorCross(Vold, Vnew, Vcross);
        dNorm = SQRT(Vcross[0]*Vcross[0] + Vcross[1]*Vcross[1]  + Vcross[2]*Vcross[2]);

        Vcross[0] = Vcross[0] / dNorm;
        Vcross[1] = Vcross[1] / dNorm;
        Vcross[2] = Vcross[2] / dNorm;

        axis2rot(theta, Vcross, R);
    }
    // trocarNewR = R * trocarOldR * arrdRotz
    rotateZ((T)Roll, arrdRotz);
    matrix3x3Multiply(R,           trocarOldR, arrd3x3Temp);
    matrix3x3Multiply(arrd3x3Temp, arrdRotz,   trocarNewR);

    trocarNewHomo[0]  = trocarNewR[0];
    trocarNewHomo[1]  = trocarNewR[1];
    trocarNewHomo[2]  = trocarNewR[2];


    trocarNewHomo[4]  = trocarNewR[3];
    trocarNewHomo[5]  = trocarNewR[4];
    trocarNewHomo[6]  = trocarNewR[5];


    trocarNewHomo[8]  = trocarNewR[6];
    trocarNewHomo[9]  = trocarNewR[7];
    trocarNewHomo[10] = trocarNewR[8];

    trocarNewHomo[3]  = trocarOldHomo[3];
    trocarNewHomo[7]  = trocarOldHomo[7];
    trocarNewHomo[11] = trocarOldHomo[11];

    trocarNewHomo[12] = 0;
    trocarNewHomo[13] = 0;
    trocarNewHomo[14] = 0;
    trocarNewHomo[15] = 1;

#ifdef DBG_ALGORITHM_
    //static int n_count = 0;
    int i, j;
    //if ( g_nDebugCount <3 )
    if ( 1 )
    {
        //ddot / (dNormVold * dNormVnew)
        printf("ddot, dNormVold, dNormVnew = %15.8f, %15.8f, %15.8f, \n", ddot, dNormVold, dNormVnew);
        printf("(ddot / (dNormVold * dNormVnew)) = %100.90f\n", (ddot / (dNormVold * dNormVnew)));
        cout << "theta = " << theta << endl;

        cout << "Vcross: " << endl;
        for (j=0 ; j<3 ; ++j)
        {
            printf("%15.8f, ", Vcross[j]);
        }
        printf("\n");

        cout << "R: " << endl;
        for (i=0 ; i<3 ; ++i)
        {
            for (j=0 ; j<3 ; ++j)
            {
                printf("%15.8f, ", R[i*3+j]);
            }
            printf("\n");
        }

        cout << "arrd3x3Temp: " << endl;
        for (i=0 ; i<3 ; ++i)
        {
            for (j=0 ; j<3 ; ++j)
            {
                printf("%15.8f, ", arrd3x3Temp[i*3+j]);
            }
            printf("\n");
        }


        cout << "arrdRotz: " << endl;
        for (i=0 ; i<3 ; ++i)
        {
            for (j=0 ; j<3 ; ++j)
            {
                printf("%15.8f, ", arrdRotz[i*3+j]);
            }
            printf("\n");
        }

        cout << "trocarNewR: " << endl;
        for (i=0 ; i<3 ; ++i)
        {
            for (j=0 ; j<3 ; ++j)
            {
                printf("%15.8f, ", trocarNewR[i*3+j]);
            }
            printf("\n");
        }
    }

#endif
}

template<typename T>
void Algorithm::axis2rot(const T theta, const T* pK, T* pRotMatrix)
{
    T v = 1.0 - cos(theta);
    T c = cos(theta);
    T s = sin(theta);

    pRotMatrix[0] = pK[0] * pK[0] * v + c;
    pRotMatrix[1] = pK[0] * pK[1] * v - pK[2] * s;
    pRotMatrix[2] = pK[0] * pK[2] * v + pK[1] * s;

    pRotMatrix[3] = pK[0] * pK[1] * v + pK[2] * s;
    pRotMatrix[4] = pK[1] * pK[1] * v + c;
    pRotMatrix[5] = pK[1] * pK[2] * v - pK[0] * s;

    pRotMatrix[6] = pK[0] * pK[2] * v - pK[1] * s;
    pRotMatrix[7] = pK[1] * pK[2] * v + pK[0] * s;
    pRotMatrix[8] = pK[2] * pK[2] * v + c;
}

// 2, AdjustEx group
/*****************************************************************************
 Prototype    : Algorithm.AdjustEx
 Description  : adjust pitch, yaw and endowrist joint point to find more suitable position and guesture.
 Input        : trocarOldHomo : old trocar homo matrix in kuka base coordinate system.
                pitchOldV: old pitch joint point coordinate in kuka base coordinate system.
                pitchNewV: adjusted pitch joint point coordinate in kuka base coordinate system.
                Roll: adjust shaft's rolling angle (radius).
                DH_param: DH parameters.
 Output       :
                [out]endowristNewHomo: New Homogeneous Matrix of endowrist.
                [out]pFlangeNewHomo:   New Homogeneous Matrix of flange.
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function
2.Date         : 2017/8/24
  Author       : lin.lin
  Modification : 1, optimized code.
3.Date         : 2017/11/29
  Author       : lin.lin
  Modification : 1, optimized code.
note: run time: about 14 us, 20180103 version
*****************************************************************************/
template<typename T>
void Algorithm::AdjustEx(const T* pTrocarOldHomo, const T* pitchOldV,  const T* pitchNewV,
                         const T Roll,            const T* pDH_param,  const int nRow,
                         T* pEndowristNewHomo,    T* pFlangeNewHomo)
{
    T darrTrocarNewHomo[16]          = {0.0};
    GetTrocarFrame(pTrocarOldHomo, pitchOldV, pitchNewV, Roll, darrTrocarNewHomo);
    IK_DH(darrTrocarNewHomo, pDH_param, 2, pFlangeNewHomo);
    FK_DH(pFlangeNewHomo, pDH_param, nRow, pEndowristNewHomo);
}
/*****************************************************************************
 Prototype    : Algorithm.AdjustEx
 Description  : adjust pitch, yaw and endowrist joint point to find more suitable position and guesture.
 Input        : trocarOldHomo : old trocar homo matrix in kuka base coordinate system.
                pitchOldV: old pitch joint point coordinate in kuka base coordinate system.
                pitchNewV: adjusted pitch joint point coordinate in kuka base coordinate system.
                Roll: adjust shaft's rolling angle (radius).
                DH_param: DH parameters.
 Output       :
                [out]endowristNewHomo: New Homogeneous Matrix of endowrist.
                [out]pitchNewHomo: New Homogeneous Matrix of pitch joint.
                [out]yawNewHomo: New Homogeneous Matrix of yaw joint.
                [out]pFlangeNewHomo:   New Homogeneous Matrix of flange.
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function
2.Date         : 2017/8/24
  Author       : lin.lin
  Modification : 1, optimized code.
3.Date         : 2017/11/29
  Author       : lin.lin
  Modification : 1, optimized code.
note: run time: about 14 us, 20180103 version; see _AdjustEx_
*****************************************************************************/
template<typename T>
void Algorithm::AdjustEx(const T* pTrocarOldHomo, const T* pitchOldV,  const T* pitchNewV,
                         const T Roll,            const T* pDH_param,
                         T* pEndowristNewHomo,    T* pPitchNewHomo,    T* pYawNewHomo, T* pFlangeNewHomo)
{
    // cout << "func, line = " << __func__ << "  " << __LINE__ << endl;

    T darrTrocarNewHomo[16]        = {0.0};
    int nRow                       = 3;
    int i = 0, j = 0;

    GetTrocarFrame(pTrocarOldHomo, pitchOldV, pitchNewV, Roll, darrTrocarNewHomo);

#ifndef NEW_INSTR_DH
    IK_DH(darrTrocarNewHomo, pDH_param,        2, pFlangeNewHomo);
    FK_DH(darrTrocarNewHomo, pDH_param + 8,    1, pPitchNewHomo);
    FK_DH(pPitchNewHomo,     pDH_param + 12,   1, pYawNewHomo);
    FK_DH(pYawNewHomo,       pDH_param + 16,   1, pEndowristNewHomo);
#else
    IK_DH(darrTrocarNewHomo,  pDH_param,      1,  pFlangeNewHomo,    DH_CONVENTION_CRAIG);
    FK_DH(darrTrocarNewHomo,  pDH_param + 4,  2,  pPitchNewHomo,     DH_CONVENTION_CRAIG);
    FK_DH(pPitchNewHomo,      pDH_param + 12, 1,  pYawNewHomo,       DH_CONVENTION_CRAIG);
    FK_DH(pYawNewHomo,        pDH_param + 16, 1,  pEndowristNewHomo, DH_CONVENTION_CRAIG);
#endif

#ifdef DBG_ALGORITHM_
    T tarr_test_1[4], tarr_test_2[4];
    memcpy(tarr_test_1, DH_param_trocar_to_end[1], sizeof(T)*4);
    memcpy(tarr_test_2, DH_param_trocar_to_end[2], sizeof(T)*4);
    //if ( n_count <5+1620 )

    cout << "func, line: " << __func__ << "    " << __LINE__ << endl;
    printf("output pDH_param: \n");
    for ( i = 0 ; i <5 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", pDH_param[i*4+j]);
        }
        printf("\n");
    }

    printf("output DH_param_trocar_to_end: \n");
    for ( i = 0 ; i <3 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", DH_param_trocar_to_end[i][j]);
        }
        printf("\n");
    }

    printf("output DH_param_trocar_to_end[1]: \n");
    for ( i = 0 ; i <4 ; ++i )
    {
        printf("%7.4f\t", tarr_test_1[i]);
    }
    printf("output DH_param_trocar_to_end[2]: \n");
    for ( i = 0 ; i <4 ; ++i )
    {
        printf("%7.4f\t", tarr_test_2[i]);
    }
    printf("\n");
    cout << "pTrocarOldHomo: " << endl;
    for (i=0 ; i<4 ; ++i)
    {
        for (j=0 ; j<4 ; ++j)
        {
            printf("%7.4f, ", pTrocarOldHomo[i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    cout << "pitchOldV, pitchNewV: " << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%7.4f, \t%7.4f, \n", pitchOldV[i], pitchNewV[i]);
    }
    printf("\n");
    cout << "Roll =  " << Roll << endl;
    cout << "darrTrocarNewHomo: " << endl;
    for (i=0 ; i<4 ; ++i)
    {
        for (j=0 ; j<4 ; ++j)
        {
            printf("%7.4f, ", darrTrocarNewHomo[i*4+j]);
        }
        printf("\n");

    }
    printf("output pPitchNewHomo: \n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%7.4f\t", pPitchNewHomo[i*4+j]);
        }
        printf("\n");
    }
    printf("output pYawNewHomo: \n");
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", pYawNewHomo[i*4+j]);
        }
        printf("\n");
    }
    printf("output pEndowristNewHomo: \n");
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", pEndowristNewHomo[i*4+j]);
        }
        printf("\n");
    }
#endif
}
/*****************************************************************************
 Prototype    : Algorithm.DHfromPostposition2Homo
 Description  : translate DH parameters to Homogeneous matrix from post position
 Input        : double alpha: (unit: rad); skew angle from zi-1 to zi, measured about xi.
                double a: the distance from zi-1 to zi measured along xi. Note that ai is always positive because of the definition of xi.
                double d: the distance from xi-1 to xi, as measured along zi-1. Note that di can be negative.
                double theta: (unit: rad); joint angle from xi-1 to xi, measured about zi-1.
 Output       : double* pHomoDst: the homogeneous matrix translated from DH parameters.
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/9/26
    Author       : lin.lin
    Modification : Created function
 note: reference: chapter 4.3 Denavit-Hartenberg Coordinates, robotics lecture notes.
*****************************************************************************/
template<typename T>
inline void Algorithm::DHfromPostposition2Homo(const T alpha, const T a, const T d, const T theta, T* pHomoDst)
{
    T ct, st, ca, sa;
    ct = cos(theta);
    st = sin(theta);
    sa = sin(alpha);
    ca = cos(alpha);

    pHomoDst[0]  =  ct;
    pHomoDst[1]  = -st*ca;
    pHomoDst[2]  =  st*sa;
    pHomoDst[3]  =  a*ct;

    pHomoDst[4]  =  st;
    pHomoDst[5]  =  ct*ca;
    pHomoDst[6]  = -ct*sa;
    pHomoDst[7]  =  a*st;

    pHomoDst[8]  =  0.0;
    pHomoDst[9]  =  sa;
    pHomoDst[10] =  ca;
    pHomoDst[11] =  d;

    pHomoDst[12] =  0.0;
    pHomoDst[13] =  0.0;
    pHomoDst[14] =  0.0;
    pHomoDst[15] =  1.0;
}
/*****************************************************************************
 Prototype    : Algorithm.DhModified2Homo
 Description  : translate DH parameters to Homogeneous matrix by modified convention (Craig's convention)
 Input        : const T alphaRad : skew angle (unit: rad).
                const T a
                const T d
                const T thetaRad : joint angle (unit: rad).
                T* pHomoDst      : the homogeneous matrix translated from DH parameters.
 Output       : None
 Return Value : inline
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/10/20
    Author       : lin.lin
    Modification : Created function
  Note: reference: fyi: the introduction of Robot, Craig.
*****************************************************************************/
template<typename T>
inline void Algorithm::DhModified2Homo(const T alphaRad, const T a, const T d, const T thetaRad, T* pHomoDst)
{
    T ct, st, ca, sa;
    ct = cos(thetaRad);
    st = sin(thetaRad);
    sa = sin(alphaRad);
    ca = cos(alphaRad);

    pHomoDst[0] = ct;
    pHomoDst[1] = -st;
    pHomoDst[2] = 0.0;
    pHomoDst[3] = a;

    pHomoDst[4] = st * ca;
    pHomoDst[5] = ct * ca;
    pHomoDst[6] = -sa;
    pHomoDst[7] = -sa * d;

    pHomoDst[8]  = st * sa;
    pHomoDst[9]  = ct * sa;
    pHomoDst[10] = ca;
    pHomoDst[11] = ca * d;

    pHomoDst[12] = 0.0;
    pHomoDst[13] = 0.0;
    pHomoDst[14] = 0.0;
    pHomoDst[15] = 1.0;
}


/* BEGIN: Added by lin.lin, 2018/10/18 */
/*****************************************************************************
 Prototype    : tr2rpy
 Description  : Convert a homogeneous transform to roll-pitch-yaw angles.
                Corresponding matlab cmd: tr2rpy(ptTsrc).
 Input        : const T* ptHomo: a homogeneous transform T
                T* ptRpyRad    : the roll-pitch-yaw angles (1 * 3), radian
                nMatrixType    : TR2RPY_MATRIXTYPE_ROTM - Rotation matrix (default);
                                 TR2RPY_MATRIXTYPE_HOMO - homo matrix.
 Output       : None
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/10/12
    Author       : lin.lin
    Modification : Created function
  note:
  1, The 3 angles RPY=[R,P,Y] correspond to sequential rotations about the X, Y and Z axes respectively.
  2, matlab cmd: rotm2eul(rot1) and tr2rpy(rot1, 'zyx') are equivalent.
  3, Options 'zyx' is implemented by function Algorithm::homo2RPY.
*****************************************************************************/
template<typename T>
void Algorithm::tr2rpy(const T* ptTsrc, T* ptRpyRad, int nMatrixType)
{
    T eps   = 2.2204e-16;
    T tarrR[9];
    T* ptSrc = (T*)ptTsrc;
    if ( TR2RPY_MATRIXTYPE_ROTM != nMatrixType)
    {
        t2r(ptTsrc, tarrR);
        ptSrc = tarrR;
    }

    if ( fabs(ptSrc[8])<eps && fabs(ptSrc[5])<eps )
    {
        // singularity
        ptRpyRad[0] = 0; // roll is zero
        ptRpyRad[1] = atan2(ptSrc[2], ptSrc[8]);  // pitch
        ptRpyRad[2] = atan2(ptSrc[3], ptSrc[4]);   // yaw is sum of roll+yaw
    }else
    {
        ptRpyRad[0] = atan2(-ptSrc[5], ptSrc[8]); // roll
        // compute sin/cos of roll angle
        T sr = sin(ptRpyRad[0]);
        T cr = cos(ptRpyRad[0]);
        ptRpyRad[1] = atan2(ptSrc[2], cr * ptSrc[8] - sr * ptSrc[5]);  // pitch
        ptRpyRad[2] = atan2(-ptSrc[1], ptSrc[0]);        // yaw
    }

//    if ( 0 != nMatrixType)
//    {
//        ptTsrc = ptSrcOld;
//    }
}

/*****************************************************************************
 Prototype    : Algorithm.rpy2r
 Description  : Convert Euler angles (by zyx sequences) to rotation matrix (3*3)
 Input        : double tRoll     : Euler angles Roll (unit: rad)
                double tPitch    : Euler angles Pitch (unit: rad)
                double tYaw      : Euler angles Yaw (unit: rad)

 Output       : double* ptHomoDst: homogeneous matrix converted from given Euler angles.
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/10/26
    Author       : lin.lin
    Modification : Created function
  note: for test tr2rpy; fyi: rpy2r, Porke.
    1, The 3 angles RPY=[R,P,Y] correspond to sequential rotations about the X, Y and Z axes respectively.
    2, matlab cmd: rotm2eul(rot1) and tr2rpy(rot1, 'zyx') are equivalent.
    3, Options 'zyx' is  implemented by function: Algorithm::eul2rotm, Algorithm::rpy2Homo(recommanded!).
*****************************************************************************/
template<typename T>
void Algorithm::rpy2r(T tRoll, T tPitch, T tYaw, T* ptHomoDst)
{
    T tarrR1[9], tarrR2[9], tarrR3[9];
    rotateX(tRoll,  tarrR1);
    rotateY(tPitch, tarrR2);
    matrix3x3Multiply(tarrR1, tarrR2, tarrR3);
    rotateZ(tYaw,  tarrR1);
    matrix3x3Multiply(tarrR3, tarrR1, ptHomoDst);
}

/*****************************************************************************
 Prototype    : rpy2jac
 Description  : Jacobian from RPY angle rates to angular velocity
 Input        : const T tRollRad  : roll angle  (radian)
                const T tPitchRad : pitch angle (radian)
                const T tYawRad   : yaw angle   (radian)
                T* ptB [out]      : Jacobian matrix (3 * 3) that maps roll-pitch-yaw angle rates to angular velocity at the operating point RPY=[R,P,Y].
 Output       : None
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/10/10
    Author       : lin.lin
    Modification : Created function
Notes:
    Used in the creation of an analytical Jacobian.
*****************************************************************************/
template<typename T>
void Algorithm::rpy2jac(const T tRollRad, const T tPitchRad, const T tYawRad, T* ptB)
{
    T cp, sp, cr, sr;
    cp = cos(tPitchRad);
    sp = sin(tPitchRad);
    cr = cos(tRollRad);
    sr = sin(tRollRad);

    ptB[0] = 1;
    ptB[1] = 0;
    ptB[2] = sp;

    ptB[3] = 0;
    ptB[4] = cr;
    ptB[5] = -cp*sr;

    ptB[6] = 0;
    ptB[7] = sr;
    ptB[8] = cp*cr;  // 8.1.3 Analytical Jacobian, Robotics, Vision and Control
}
/*****************************************************************************
 Prototype    : Algorithm.t2r
 Description  : get the rotational submatrix
 Input        : const T* ptHomo
                T* ptR
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/10/20
    Author       : lin.lin
    Modification : Created function
  Note: R = t2r(T) is the orthonormal rotation matrix component of homogeneous transformation matrix T.
*****************************************************************************/
template<typename T>
void Algorithm::t2r(const T* ptHomo, T* ptR)
{
    ptR[0] = ptHomo[0];
    ptR[1] = ptHomo[1];
    ptR[2] = ptHomo[2];

    ptR[3] = ptHomo[4];
    ptR[4] = ptHomo[5];
    ptR[5] = ptHomo[6];

    ptR[6] = ptHomo[8];
    ptR[7] = ptHomo[9];
    ptR[8] = ptHomo[10];
}

template<typename T>
void Algorithm::r2t(const T* ptR, T* ptHomo)
{
    ptHomo[0] = ptR[0];
    ptHomo[1] = ptR[1];
    ptHomo[2] = ptR[2];
    ptHomo[3] = 0.0;

    ptHomo[4] = ptR[3];
    ptHomo[5] = ptR[4];
    ptHomo[6] = ptR[5];
    ptHomo[7] = 0.0;

    ptHomo[8]  = ptR[6];
    ptHomo[9]  = ptR[7];
    ptHomo[10] = ptR[8];
    ptHomo[11] = 0.0;

    ptHomo[12] = 0.0;
    ptHomo[13] = 0.0;
    ptHomo[14] = 0.0;
    ptHomo[15] = 1.0;
}

/*****************************************************************************
 Prototype    : Algorithm.jacob0
 Description  : get Jacobian in world coordinates by the method of differential transformation.
     the Jacobian matrix (6*N) for the robot in pose q (1*N), and N is the number of robot joints.
     The manipulator Jacobian matrix maps joint velocity to end-effector spatial velocity V = j0*QD expressed in the world-coordinate frame.
 Input        : const T* ptDH_param    : DH parameters, a, alpha(rad), d, q(rad)
                const int nJointNum    : joint number; (or the row of DH_param)
                const int nDof         : Dof
                const int* pnJointType : joint's type; JOINTTYPE_ROTATION - rotation (default); JOINTTYPE_PRISMATIC - transformation;
                const int nDhConvention: dh conbention; DH_CONVENTION_STD - standard; DH_CONVENTION_CRAIG - Craig's
                T* ptarrJ              : the Jacobian matrix (6*N)
                T* ptLog               : reserve for debug
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/10/10
    Author       : lin.lin
    Modification : Created function

  note: relative to matlab function getJacobian.m
*****************************************************************************/
template<typename T>
void Algorithm::jacob0(const T* ptDH_param, const int nJointNum, const int nDof, const int* pnJointType, const int nDhConvention,
            T* ptarrJ, T* ptLog)
{
    int nTotalDH = nJointNum * 4;
    T a[JOINT_NUM_MAX]    = {0};
    T aRad[JOINT_NUM_MAX] = {0};
    T d[JOINT_NUM_MAX]    = {0};
    T qRad[JOINT_NUM_MAX] = {0};
    T tarrT[JOINT_NUM_MAX][16] = {0.0};
    T tarrR[JOINT_NUM_MAX][9]  = {0.0};
    T tarrTi[JOINT_NUM_MAX][16]= {0.0};
    T tarrZ[JOINT_NUM_MAX][3]  = {0.0};
    T tarrPi[JOINT_NUM_MAX][3] = {0.0};
    T tarrPi0[JOINT_NUM_MAX][3]= {0.0};
    //T tarrJ[JOINT_NUM_MAX][4]  = {0.0};
    T* ptarrT      = NULL;
    T* ptarrR      = NULL;
    T* ptarrPrevR  = NULL;
    T* ptarrTi     = NULL;
    T* ptarrPrevTi = NULL;
    T* ptarrZ      = NULL;
    T* ptarrPi     = NULL;
    T* ptarrPi0    = NULL;
    //T* ptarrJ      = NULL;

    T tarr3x3Temp[9]     = {0.0};
    T tarrvectorCross[3] = {0.0};
    T* ptLogTemp = ptLog;
    int i = 0, j = 0;
    for ( i = 0 ; i < nTotalDH ; ++i )
    {
        a[i]    = ptDH_param[4*i];
        aRad[i] = ptDH_param[4*i+1];
        d[i]    = ptDH_param[4*i+2];
        qRad[i] = ptDH_param[4*i+3];
    }

    // get T, R (passed!)
    for ( i = 0 ; i <nJointNum ; ++i )
    {
        ptarrT     = (T*)tarrT + i*16;
        ptarrPrevR = ptarrR;
        ptarrR     = (T*)tarrR + i*9;
        if (DH_CONVENTION_CRAIG == nDhConvention )
        {
            //T(:,:, i) = getTransformationByCraigEx(a(i), alpha(i), d(i), theta(i));
            DhModified2Homo(aRad[i], a[i], d[i], qRad[i], ptarrT);

        }else
        {
            //T(:,:, i) = getTransformationByStdDH(a(i), alpha(i), d(i), theta(i));
            DHfromPostposition2Homo(aRad[i], a[i], d[i], qRad[i], ptarrT);
        }
        t2r(ptarrT, ptarrR);
        if ( i > 0 )
        {
            matrix3x3Multiply(ptarrPrevR, ptarrR, tarr3x3Temp);
            memcpy(ptarrR, tarr3x3Temp, sizeof(T)*9);
        }
    }
#ifdef UNIT_TEST_MODE
    if ( NULL != ptLogTemp )
    {
        memcpy(ptLogTemp, tarrT, sizeof(T)*16*nJointNum);
        ptLogTemp = ptLogTemp + nJointNum*16;
        memcpy(ptLogTemp, tarrR, sizeof(T)*9*nJointNum);
    }
#endif

    // get transformation matrix from each link to endo (Ti0) (passed!)
    for ( i = 0, j = nJointNum; i <= nJointNum; ++i, --j )
    {
        ptarrT      = (T*)tarrT + j*16;
        ptarrPrevTi = ptarrTi;
        ptarrTi     = (T*)tarrTi + j*16;

        if ( 0 == i )
        {
            ptarrTi[0]  = 1;
            ptarrTi[5]  = 1;
            ptarrTi[10] = 1;
            ptarrTi[15] = 1;
        }else
        {
            // %T65 =T(:,:, 6) * T66;
            matrix4x4Multiply(ptarrT, ptarrPrevTi, ptarrTi);
        }

    }

#ifdef UNIT_TEST_MODE
    if ( NULL != ptLogTemp )
    {
        ptLogTemp += 9*nJointNum;
        memcpy(ptLogTemp, tarrTi, sizeof(T)*16*(nJointNum+1));
    }
#endif

    // get Z, P (passed!)
    if (DH_CONVENTION_CRAIG == nDhConvention )
    {
        for ( i = 0 ; i <nJointNum ; ++i )
        {
            // get Z
            //ptarrPrevR = ptarrR;
            ptarrZ     = (T*)tarrZ   + i*3;
            ptarrPi    = (T*)tarrPi  + i*3;
            ptarrPi0   = (T*)tarrPi0 + i*3;
            ptarrR     = (T*)tarrR   + i*9;
            ptarrTi    = (T*)tarrTi  + (i+1)*16;
            //z(:, i) = R(:, 3, i);
            ptarrZ[0]  = ptarrR[2];
            ptarrZ[1]  = ptarrR[5];
            ptarrZ[2]  = ptarrR[8];

            // get P
            //p6i = T6i(1:3, 4); p61 = T61(1:3, 4);
            ptarrPi[0] = ptarrTi[3];
            ptarrPi[1] = ptarrTi[7];
            ptarrPi[2] = ptarrTi[11];
            //% p6i0 = R(:, :, 1) * p6i; p610 = R(:, :, 1) * p61;
            matrix3x1Multiply(ptarrR, ptarrPi, ptarrPi0);
        }
    }else
    {
        // z(:, 1) = [0, 0, 1].';
        // p61 = T60(1:3, 4);
        // p610 = p61;
        ptarrZ     = (T*)tarrZ;
        ptarrPi     = (T*)tarrPi;
        ptarrPi0     = (T*)tarrPi0;
        ptarrTi = (T*)tarrTi;
        ptarrZ[0] = 0;
        ptarrZ[1] = 0;
        ptarrZ[2] = 1;
        ptarrPi0[0] = ptarrPi[0] = ptarrTi[3];
        ptarrPi0[1] = ptarrPi[1] = ptarrTi[7];
        ptarrPi0[2] = ptarrPi[2] = ptarrTi[11];
        for ( i = 1 ; i < nJointNum ; ++i )
        {
            ptarrZ    = (T*)tarrZ   + i*3;
            ptarrPi   = (T*)tarrPi  + i*3;
            ptarrPi0  = (T*)tarrPi0 + i*3;
            ptarrR    = (T*)tarrR   + (i-1)*9;
            ptarrTi   = (T*)tarrTi  + i*16;
            // get Z
            //z(:, i) = R(:, 3, nIndex);
            ptarrZ[0] = ptarrR[2];
            ptarrZ[1] = ptarrR[5];
            ptarrZ[2] = ptarrR[8];
            // get P
            // p6i = T6,i-1(1:3, 4); p61 = T61(1:3, 4);
            ptarrPi[0] = ptarrTi[3];
            ptarrPi[1] = ptarrTi[7];
            ptarrPi[2] = ptarrTi[11];
            // p6i0 = R(:, :, i-1) * p6i; p610 = R(:, :, 1) * p61;
            matrix3x1Multiply(ptarrR, ptarrPi, ptarrPi0);
        }
    }

#ifdef UNIT_TEST_MODE
    if ( NULL != ptLogTemp )
    {
        ptLogTemp += 16*(nJointNum+1);
        memcpy(ptLogTemp, tarrZ, sizeof(T)*3*nJointNum);
        ptLogTemp += 3*nJointNum;
        memcpy(ptLogTemp, tarrPi0, sizeof(T)*3*nJointNum);
    }
#endif

    // get Jacobian
    //ptarrJ   = (T*)tarrJ;
    for ( i = 0 ; i <nJointNum ; ++i )
    {
        ptarrZ   = (T*)tarrZ   + i*3;
        ptarrPi0 = (T*)tarrPi0 + i*3;
        //pTemp = p6i0;
        if ( JOINTTYPE_PRISMATIC == pnJointType[i] )
        {
            // JacobMatrix(:,i) = [z(:,i); zeros(3, 1)]; % for prismatic joint, fyi (4 - 23)
            ptarrJ[i + 0*nJointNum] = ptarrZ[0];
            ptarrJ[i + 1*nJointNum] = ptarrZ[1];
            ptarrJ[i + 2*nJointNum] = ptarrZ[2];
            ptarrJ[i + 3*nJointNum] = 0;
            ptarrJ[i + 4*nJointNum] = 0;
            ptarrJ[i + 5*nJointNum] = 0;

        }else
        {
            // JacobMatrix(:,i) = [cross(z(:,i), pTemp); z(:,i)]; % for rotation joint, fyi (4 - 24)
            vectorCross(ptarrZ, ptarrPi0, tarrvectorCross);
            ptarrJ[i + 0*nJointNum] = tarrvectorCross[0];
            ptarrJ[i + 1*nJointNum] = tarrvectorCross[1];
            ptarrJ[i + 2*nJointNum] = tarrvectorCross[2];
            ptarrJ[i + 3*nJointNum] = ptarrZ[0];
            ptarrJ[i + 4*nJointNum] = ptarrZ[1];
            ptarrJ[i + 5*nJointNum] = ptarrZ[2];
        }
    }
#ifdef UNIT_TEST_MODE
    if ( NULL != ptLogTemp )
    {
        ptLogTemp += 3*nJointNum;
        memcpy(ptLogTemp, ptarrJ, sizeof(T)*6*nJointNum);
    }
#endif
}

/* END:   Added by lin.lin, 2018/10/18   PN: */
/*****************************************************************************
 Prototype    : Algorithm.FK_DH
 Description  : FK with DH parameters.
 Input        : const double* parOrigin : original homo matrix (4*4)
                const double* pDH_param : DH parameters; the unit of alpha and theta are radius.
                const int nRow          : total rows of DH_param
                const int nFlag         : DH_CONVENTION_STD - std DH; DH_CONVENTION_CRAIG - Craig DH (modified DH)
 Output       : double* parrDest [out]  : destination homo matrix (4*4)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/9/15
    Author       : lin.lin
    Modification : Created function
  2.Date         : 2018/10/20
    Author       : lin.lin
    Modification : add Craig's DH algorithm
note: This function version of FK_DH is Recommended.
*****************************************************************************/
template<typename T>
void Algorithm::FK_DH(const T* ptOrigin, const T* pDH_param, const int nRow, T* ptDestHomo, const int nFlag)
{
    T tarrHomoTemp[HOMO_MATRIX_ELEM_CNT]   = {0};
    T tarrOriginTemp[HOMO_MATRIX_ELEM_CNT] = {0};
    memcpy(tarrOriginTemp, ptOrigin, sizeof(T)*12);
    tarrOriginTemp[15] = 1.0;

    for (int i = 0 ; i < nRow ; ++i )
    {
        if ( DH_CONVENTION_STD == nFlag )
        {
            DHfromPostposition2Homo(pDH_param[i*4], pDH_param[i*4 +1], pDH_param[i*4 +2], pDH_param[i*4 +3], tarrHomoTemp);
        }else  // DH_CONVENTION_CRAIG
        {
            DhModified2Homo(pDH_param[i*4], pDH_param[i*4 +1], pDH_param[i*4 +2], pDH_param[i*4 +3], tarrHomoTemp);
        }

        matrix4x4Multiply(tarrOriginTemp, tarrHomoTemp, ptDestHomo);
        memcpy(tarrOriginTemp, ptDestHomo, sizeof(T)*12);
    }
}

/*****************************************************************************
 Prototype    : Algorithm.IK_DH
 Description  : IK with DH parameters.
 Input        : parrEndHomo: end homo matrix
                pDH_param  : DH parameters; the unit of alpha and theta are radius. ; the address is relative to parrStartHomo
                nRow           : total rows of DH_param
                const int nFlag         : DH_CONVENTION_STD - std DH; DH_CONVENTION_CRAIG - Craig DH (modified DH)
 Output       : parrStartHomo    : destination DH matrix (4*4)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/12/04
    Author       : lin.lin
    Modification : Created function
  note: optimized the original ik algorithm, this version is recommanded!
        run time: about 2 us, 20180103 version
*****************************************************************************/
template<typename T>
void Algorithm::IK_DH(const T* parrEndHomo, const T* pDH_param, const int nRow, T* parrStartHomo, const int nFlag)
{
    T dHomoCur[HOMO_MATRIX_ELEM_CNT]   = {0.0};        // T1*T2*T3*T4
    T dHomoDHInv[HOMO_MATRIX_ELEM_CNT] = {0.0};        // inv(T1234)
    T darrTi[HOMO_MATRIX_ELEM_CNT]     = {0.0};        // store  T1, T2, T3, T4

    // temp homoMatrix that store the middle result while calculating T1*T2*T3*T4
    T arrOriginTemp[HOMO_MATRIX_ELEM_CNT] =
       {1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1};

    // parrStartHomo = parrEndHomo * inv(T1*T2*T3*T4)
    for (int i = 0 ; i < nRow; ++i )
    {
        if ( DH_CONVENTION_STD == nFlag ) {
            DHfromPostposition2Homo(pDH_param[i*4], pDH_param[i*4 +1], pDH_param[i*4 +2], pDH_param[i*4 +3], darrTi);
        } else {
            // DH_CONVENTION_CRAIG
            DhModified2Homo(pDH_param[i*4], pDH_param[i*4 +1], pDH_param[i*4 +2], pDH_param[i*4 +3], darrTi);
        }
        matrix4x4Multiply(arrOriginTemp, darrTi, dHomoCur);
        memcpy(arrOriginTemp, dHomoCur, sizeof(T)*12);
    }
    homogeneous4x4Invers(dHomoCur, dHomoDHInv);
    matrix4x4Multiply(parrEndHomo, dHomoDHInv, parrStartHomo);
}

// 3, CreateAdjustedVectors group
/*****************************************************************************
 Prototype    : Algorithm.CreateAdjustedVectors
 Description  : created pitch joint's neighbourhood points around sphere center; created 14 points on shpere.
 Input        : const double* darrSphereCenterV: center of sphere.3*1
                const double dSphereR:           radius of sphere.
                double** ppdNeighbourVectTrans:  neighbourhood points (15*3)
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/11/29
    Author       : lin.lin
    Modification : Created function
  note: recommend!
note: run time: about 10 ~ 14 us, 20180103 version
*****************************************************************************/
template<typename T>
void Algorithm::CreateAdjustedVectors(const T* darrSphereCenterV, const T dSphereR, T** ppdNeighbourVectTrans)
{
    //cout << "func, line = " << __func__ << "  " << __LINE__ << endl;
    T arrRot[9]  = {0.0};
    T darrVectorZ[3] = {0.0, 0.0, dSphereR};
    int i, j;
    T *pdNeighbourVectTrans = (T*)ppdNeighbourVectTrans;
    pdNeighbourVectTrans[0] = 0.0;
    pdNeighbourVectTrans[1] = 0.0;
    pdNeighbourVectTrans[2] = 0.0;
    pdNeighbourVectTrans = (T*)ppdNeighbourVectTrans+13*3;

    pdNeighbourVectTrans[0] = darrVectorZ[0];
    pdNeighbourVectTrans[1] = darrVectorZ[1];
    pdNeighbourVectTrans[2] = darrVectorZ[2];
    // % A1 B1 C1 D1 6~9
    rotateY((T)DEG2RAD_45, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (T*)ppdNeighbourVectTrans+5*3);
    rotateX(-(T)DEG2RAD_45, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (T*)ppdNeighbourVectTrans+6*3);
    rotateY(-(T)DEG2RAD_45, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (T*)ppdNeighbourVectTrans+7*3);
    rotateX((T)DEG2RAD_45, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (T*)ppdNeighbourVectTrans+8*3);
//    % A0 B0 C0 D0 2~5
    rotateY((T)DEG2RAD_90, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (T*)ppdNeighbourVectTrans+1*3);
    rotateX(-(T)DEG2RAD_90, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (T*)ppdNeighbourVectTrans+2*3);
    rotateY(-(T)DEG2RAD_90, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (T*)ppdNeighbourVectTrans+3*3);
    rotateX((T)DEG2RAD_90, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (T*)ppdNeighbourVectTrans+4*3);
//     % A-1 B-1 C-1 D-1 10~13
    rotateY((T)DEG2RAD_135, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, ((T*)ppdNeighbourVectTrans + 9*3));
    rotateX(-(T)DEG2RAD_135, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, ((T*)ppdNeighbourVectTrans + 10*3));
    rotateY(-(T)DEG2RAD_135, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, ((T*)ppdNeighbourVectTrans + 11*3));
    rotateX((T)DEG2RAD_135, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, ((T*)ppdNeighbourVectTrans + 12*3));

//    neighbourhoodV(:,15) = roty(pi) * darrVectorZ;
    rotateY((T)PI, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, ((T*)ppdNeighbourVectTrans + 14*3));
    for ( i = 0 ; i < 15 ; ++i )
    {
        pdNeighbourVectTrans = (T*)ppdNeighbourVectTrans + i*3;
        pdNeighbourVectTrans[0] +=  darrSphereCenterV[0];
        pdNeighbourVectTrans[1] +=  darrSphereCenterV[1];
        pdNeighbourVectTrans[2] +=  darrSphereCenterV[2];
    }
}
/*****************************************************************************
 Prototype    : Algorithm.CreateAdjustedVectorsEx
 Description  : created pitch joint's neighbourhood points around sphere center
                according transformation from spherical coordinate system to Cartesian coordinate system; created 14 points on shpere.
 Input        : tarrSphereCenterV: center of sphere.3*1
                tSphereR:           radius of sphere.
                pptNeighbourVectTrans:  neighbourhood points (15*3)
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/01/03
    Author       : lin.lin
    Modification : Created function
  note: recommend!
note: run time: about 1 us, 20180103 version
*****************************************************************************/
template<typename T>
void Algorithm::CreateAdjustedVectorsEx(const T* tarrSphereCenterV, const T tSphereR, T** pptNeighbourVectTrans)
{
#ifdef UNIT_TEST_MODE_
    // typedef float localT;
    T tFai[4] = {0,             90*PI_DIV_180, 180*PI_DIV_180, 270*PI_DIV_180};
    T tThi[3] = {90*PI_DIV_180, 45*PI_DIV_180, 135*PI_DIV_180};
    T tx, ty, tz;
    int i, j;
    T *ptNeighbourVectTrans = (T*)pptNeighbourVectTrans;
    ptNeighbourVectTrans[0] = 0.0;
    ptNeighbourVectTrans[1] = 0.0;
    ptNeighbourVectTrans[2] = 0.0;
    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+13*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = tSphereR;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+14*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = -tSphereR;
    for ( i = 0 ; i <3 ; ++i )
    {
        //tFai[i] *= PI_DIV_180;
        for ( j = 0 ; j < 4 ; ++j )
        {
            //tThi[j] *= PI_DIV_180;
            ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+(i*4+j+1)*3;
            ptNeighbourVectTrans[0] = tSphereR * sin(tThi[i]) * cos(tFai[j]);
            ptNeighbourVectTrans[1] = tSphereR * sin(tThi[i]) * sin(tFai[j]);
            ptNeighbourVectTrans[2] = tSphereR * cos(tThi[i]);
        }
    }
#else
    int i;
    T tTempP = 0.70710678 * tSphereR;
    //T tTempM = -tTempP;
    T *ptNeighbourVectTrans = (T*)pptNeighbourVectTrans;
    ptNeighbourVectTrans[0] = 0.0;
    ptNeighbourVectTrans[1] = 0.0;
    ptNeighbourVectTrans[2] = 0.0;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+1*3;
    ptNeighbourVectTrans[0] = tSphereR;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = 0;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+2*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = tSphereR;
    ptNeighbourVectTrans[2] = 0;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+3*3;
    ptNeighbourVectTrans[0] = -tSphereR;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = 0;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+4*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = -tSphereR;
    ptNeighbourVectTrans[2] = 0;

    ///
    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+5*3;
    ptNeighbourVectTrans[0] = tTempP;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = tTempP;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+6*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = tTempP;
    ptNeighbourVectTrans[2] = tTempP;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+7*3;
    ptNeighbourVectTrans[0] = -tTempP;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = tTempP;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+8*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = -tTempP;
    ptNeighbourVectTrans[2] = tTempP;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+9*3;
    ptNeighbourVectTrans[0] = tTempP;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = -tTempP;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+10*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = tTempP;
    ptNeighbourVectTrans[2] = -tTempP;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+11*3;
    ptNeighbourVectTrans[0] = -tTempP;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = -tTempP;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+12*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = -tTempP;
    ptNeighbourVectTrans[2] = -tTempP;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+13*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = tSphereR;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans+14*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = -tSphereR;
#endif

    for ( i = 0 ; i < 15 ; ++i )
    {
        ptNeighbourVectTrans = (T*)pptNeighbourVectTrans + i*3;
        ptNeighbourVectTrans[0] +=  tarrSphereCenterV[0];
        ptNeighbourVectTrans[1] +=  tarrSphereCenterV[1];
        ptNeighbourVectTrans[2] +=  tarrSphereCenterV[2];
    }
}
template<typename T>
void Algorithm::CreateAdjustedVectorsEx(const T* tarrSphereCenterV,  const T tSphereR,
                                        const T  tLongtitudeStepDeg, const T tLatitudeStepDeg,
                                        T**     pptNeighbourVectTrans)
{
    T *ptNeighbourVectTrans = NULL;
#if 1
    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans;
    ptNeighbourVectTrans[0] = tarrSphereCenterV[0];
    ptNeighbourVectTrans[1] = tarrSphereCenterV[1];
    ptNeighbourVectTrans[2] = tarrSphereCenterV[2];

    ptNeighbourVectTrans    = (T*)pptNeighbourVectTrans + 1*3;
    ptNeighbourVectTrans[0] = tarrSphereCenterV[0];
    ptNeighbourVectTrans[1] = tarrSphereCenterV[1];
    ptNeighbourVectTrans[2] = tarrSphereCenterV[2] + tSphereR;

    ptNeighbourVectTrans    = (T*)pptNeighbourVectTrans + 2*3;
    ptNeighbourVectTrans[0] = tarrSphereCenterV[0] + tSphereR;
    ptNeighbourVectTrans[1] = tarrSphereCenterV[1];
    ptNeighbourVectTrans[2] = tarrSphereCenterV[2];

    ptNeighbourVectTrans    = (T*)pptNeighbourVectTrans + 3*3;
    ptNeighbourVectTrans[0] = tarrSphereCenterV[0];
    ptNeighbourVectTrans[1] = tarrSphereCenterV[1] + tSphereR;
    ptNeighbourVectTrans[2] = tarrSphereCenterV[2];

    ptNeighbourVectTrans    = (T*)pptNeighbourVectTrans + 4*3;
    ptNeighbourVectTrans[0] = tarrSphereCenterV[0] - tSphereR;
    ptNeighbourVectTrans[1] = tarrSphereCenterV[1];
    ptNeighbourVectTrans[2] = tarrSphereCenterV[2];

    ptNeighbourVectTrans    = (T*)pptNeighbourVectTrans + 5*3;
    ptNeighbourVectTrans[0] = tarrSphereCenterV[0];
    ptNeighbourVectTrans[1] = tarrSphereCenterV[1] - tSphereR;
    ptNeighbourVectTrans[2] = tarrSphereCenterV[2];

    ptNeighbourVectTrans    = (T*)pptNeighbourVectTrans + 6*3;
    ptNeighbourVectTrans[0] = tarrSphereCenterV[0];
    ptNeighbourVectTrans[1] = tarrSphereCenterV[1];
    ptNeighbourVectTrans[2] = tarrSphereCenterV[2] - tSphereR;

    return;
#endif
    const int tLongtitudeNum = 360 / tLongtitudeStepDeg;
    const int tLatitudeNum   = 180 / tLatitudeStepDeg -1;
#if defined (WIN32) || defined (WIN64)
    T arrtLongtitudeRad[2048] = {0};
    T arrtLatitudeRad[1024] = {0};
#else
    T arrtLongtitudeRad[tLongtitudeNum];
    T arrtLatitudeRad[tLatitudeNum];
#endif
    int nTotalRow = 0;
    int i, j;
    nTotalRow = tLatitudeNum * tLongtitudeNum + 3;
    for ( i = 0 ; i <tLatitudeNum; ++i )
    {
        arrtLatitudeRad[i] = tLatitudeStepDeg * PI_DIV_180 * (i+1);
    }
    for ( j = 0 ; j <tLongtitudeNum; ++j )
    {
        arrtLongtitudeRad[j] = tLongtitudeStepDeg * PI_DIV_180 * j;
    }

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans;
    ptNeighbourVectTrans[0] = 0.0;
    ptNeighbourVectTrans[1] = 0.0;
    ptNeighbourVectTrans[2] = 0.0;

    ptNeighbourVectTrans    = (T*)pptNeighbourVectTrans + 1*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = tSphereR;

    ptNeighbourVectTrans = (T*)pptNeighbourVectTrans + (nTotalRow-1)*3;
    ptNeighbourVectTrans[0] = 0;
    ptNeighbourVectTrans[1] = 0;
    ptNeighbourVectTrans[2] = -tSphereR;

    for ( i = 0 ; i <tLatitudeNum; ++i )
    {
        for ( j = 0 ; j <  tLongtitudeNum; ++j )
        {
            ptNeighbourVectTrans    = (T*)pptNeighbourVectTrans+(i*tLongtitudeNum+j+2)*3;
            ptNeighbourVectTrans[0] = tSphereR * sin(arrtLatitudeRad[i]) * cos(arrtLongtitudeRad[j]);
            ptNeighbourVectTrans[1] = tSphereR * sin(arrtLatitudeRad[i]) * sin(arrtLongtitudeRad[j]);
            ptNeighbourVectTrans[2] = tSphereR * cos(arrtLatitudeRad[i]);
        }
    }

    for ( i = 0 ; i < nTotalRow ; ++i )
    {
        ptNeighbourVectTrans = (T*)pptNeighbourVectTrans + i*3;
        ptNeighbourVectTrans[0] +=  tarrSphereCenterV[0];
        ptNeighbourVectTrans[1] +=  tarrSphereCenterV[1];
        ptNeighbourVectTrans[2] +=  tarrSphereCenterV[2];
    }
#ifdef dbg_CreateAdjustedVectorsEx
    // 1, tLongtitudeNum, tLatitudeNum
    printf("tLongtitudeNum, tLatitudeNum = %d, %d. \n");

    // 2, arrtLongtitude arrtLatitude
    for ( i = 0 ; i <tLatitudeNum; ++i )
    {
        printf("arrtLongtitude[%d] = %8.4f \n", i, arrtLatitudeRad[i] * _180_DIV_PI);
    }
    printf("\n");
    for ( j = 0 ; j <tLongtitudeNum; ++j )
    {
        printf("arrtLongtitude[%d] = %8.4f \n", j, arrtLongtitudeRad[j] * _180_DIV_PI);
    }
#endif
}

// 4, GetWeight
/*****************************************************************************
 Prototype    : Algorithm.GetWeight
 Description  : get the weight of object function
 Input        : pdarrHomoSample: sample set of homo matrices which comes from the neighbour field of the original homo matrix
                pdarrDestHomo:  target clip homo;
                nLen:        total sample number; default is 405
                dMiu1:  the weight of quaternion(q1~q4)
                dMiu2:  the weight of position(x, y, z)
                dMean1: the mean of quaternion functions
                dMean2: the mean of position funcitons
                double* pOutData: used to output some inner variables' value in the debug mode.
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/12/05
    Author       : lin.lin
    Modification : Created function
  note: this version is recommanded!
*****************************************************************************/
template<typename T>
void Algorithm::GetWeight(const T** pptarrHomoSample, const int nAdjustSampleNum, T& tRotMiu, T& tPosMiu, T& tRotMean, T& tPosMean, T* pOutData)
{
    const int &TOTAL_SAMPLE_SIZE = nAdjustSampleNum;
    T sizeTotal = TOTAL_SAMPLE_SIZE;

    T tarrQuatInit[4];
    T tarrHomoSampleR[9] = {0.0};
    T tarrPosInit[3]     = {0.0};  // initial position
    T tarrPosCur[3]      = {0.0};  // current position

    T tarrQuat[TOTAL_SAMPLE_SIZE][4]  = {0.0};
    T tarrVariance[TOTAL_SAMPLE_SIZE] = {0.0};
    T tarrNormPos[TOTAL_SAMPLE_SIZE]  = {0.0};
    // 1, compute darrHomoInit, darrQuatInit
    int i, j, k;
    for (i = 0; i < 3; ++i)
    {
        for (j = 0; j < 3; ++j)
        {
            tarrHomoSampleR[i*3+j] = *((T*)pptarrHomoSample+ i*4 + j );
        }
        tarrPosInit[i] = *((T*)pptarrHomoSample + i*4 + 3);
    }
    rotation2Quat(tarrHomoSampleR, tarrQuatInit);

    // 2, compute darrVariance, darrNormPos
    //  compute quaternion's object function value
    for (k = 0; k < nAdjustSampleNum; ++k)
    {
        for (i = 0; i < 3; ++i)
        {
            for (j = 0; j < 3; ++j)
            {
                tarrHomoSampleR[i*3+j] = *((T*)pptarrHomoSample +k*16 + i*4 + j);
            }
            tarrPosCur[i] = *((T*)pptarrHomoSample +k*16 + i*4 + 3);
        }
        // get the differences of two quaternion
        rotation2Quat(tarrHomoSampleR, tarrQuat[k]);
        tarrVariance[k] = QuatDiff(tarrQuatInit, tarrQuat[k]);
        // compute position's object function values
        tarrNormPos[k] = Distance(tarrPosCur, tarrPosInit);
    }

    // 3, compute mean1, mean2; u1, u2;
    tRotMean = mean(tarrVariance, nAdjustSampleNum);
    tPosMean = mean(tarrNormPos,  nAdjustSampleNum);
    // normalization
    tRotMiu = 1 / tRotMean; // rot miu
    tPosMiu = 1 / tPosMean; // pos miu
}

template<typename T>
void Algorithm::GetPosWeight(const T** pptarrHomoSample, const int nAdjustSampleNum, T& tPosMiu, T* pOutData)
{
    const int &TOTAL_SAMPLE_SIZE = nAdjustSampleNum;
    T sizeTotal = TOTAL_SAMPLE_SIZE;

    T tarrQuatInit[4];
    T tarrHomoSampleR[9] = {0.0};
    T tarrPosInit[3]     = {0.0};  // initial position
    T tarrPosCur[3]      = {0.0};  // current position

    T tarrQuat[TOTAL_SAMPLE_SIZE][4]  = {0.0};
    T tarrVariance[TOTAL_SAMPLE_SIZE] = {0.0};
    T tarrNormPos[TOTAL_SAMPLE_SIZE]  = {0.0};
    T tMean2 = 0.0;

    // 1, compute darrHomoInit, darrQuatInit
    int i, j, k;
    for (i = 0; i < 3; ++i)
    {
        tarrPosInit[i] = *((T*)pptarrHomoSample + i*4 + 3);
    }

    // 2, compute darrNormPos
    for (k = 0; k < nAdjustSampleNum; ++k)
    {
        for (i = 0; i < 3; ++i)
        {
            tarrPosCur[i] = *((T*)pptarrHomoSample +k*16 + i*4 + 3);
        }
        tarrNormPos[k] = Distance(tarrPosCur, tarrPosInit);
    }

    // 3, compute mean1, mean2; u1, u2;
    tMean2 = mean(tarrNormPos, nAdjustSampleNum);

    // normalization
    tPosMiu = 1 / tMean2; // pos miu
}
/*****************************************************************************
 Prototype    : Algorithm.mean
 Description  : Average or mean value of array
 Input        : double* tarrSrc :  pointer to the array of source data
                int nTotalNum   :  the array dimension
 Output       : None
 Return Value : <typename T>
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/14
    Author       : lin.lin
    Modification : Created function
2.Date         : 2018/2/1
  Author       : lin.lin
  Modification : template

*****************************************************************************/
template<typename T>
T Algorithm::mean(T* tarrSrc, int nTotalNum)
{
    int i = 0;
    T tSum = 0.0;
    if (nTotalNum <= 0)
    {
        return 0;
    }
    for (i = 0; i < nTotalNum; ++i)
    {
        tSum += tarrSrc[i];
    }
    return tSum /= nTotalNum;
}
/*****************************************************************************
 Prototype    : Algorithm.std
 Description  : Standard deviation
 Input        : T* tarrSrc      :  pointer to the array of source data
                int nTotalNum   :  the array dimension
 Output       : None
 Return Value : T
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/2/1
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
T Algorithm::StandardDeviation(T* tarrSrc, int nTotalNum)
{
    T tSum = 0.0;
    T  tResidual = 0.0;
    int tMean = mean(tarrSrc, nTotalNum);

    for (int i = 0; i < nTotalNum; ++i)
    {
        tResidual = tarrSrc[i] - tMean;
        tSum += tResidual * tResidual;
    }
    return sqrt(tSum / (nTotalNum - 1));
}
/*****************************************************************************
 Prototype    : Algorithm.getMeanAndStd
 Description  : get the mean value and Standard deviation of the sample data OBJ_FUNC
 Input        : const OBJ_FUNC<T>* pObj_func : sample data (function values)
                const int nTotalNum          : the total number of sample data
                T& tPosFuncMean              : the mean value of position function
                T& tRotFuncMean              : the mean value of rotation function
                T& tPosFuncStd               : the standard deviation value of position function
                T& tRotFuncStd               : the standard deviation value of rotation function
 Output       : None
 Return Value : int: -1 exception; 0 sucess.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/2/7
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
int Algorithm::getMeanAndStd(const OBJ_FUNC<T>* pObj_func, const int nTotalNum,
                                T& tPosFuncMean, T& tRotFuncMean, T& tPosFuncStd, T& tRotFuncStd)
{
    T tPosFuncSum = 0.0;
    T tRotFuncSum = 0.0;
    T tResidual   = 0.0;
    int i;

    // exception
    if (nTotalNum <= 1)
    {
        tPosFuncMean = 0;
        tRotFuncMean = 0;
        tPosFuncStd  = 0;
        tRotFuncStd  = 0;
        return -1;
    }

    // get mean
    for (i = 0; i < nTotalNum; ++i)
    {
        tPosFuncSum += pObj_func[i].tPosEvaluateValue;
        tRotFuncSum += pObj_func[i].tRotEvaluateValue;
    }

    tPosFuncMean = tPosFuncSum / nTotalNum;
    tRotFuncMean = tRotFuncSum / nTotalNum;

    // get std
    tPosFuncSum = 0.0;
    tRotFuncSum = 0.0;
    for (i = 0; i < nTotalNum; ++i)
    {
        tResidual = pObj_func[i].tPosEvaluateValue - tPosFuncMean;
        tPosFuncSum += tResidual * tResidual;
        tResidual = pObj_func[i].tRotEvaluateValue - tRotFuncMean;
        tRotFuncSum += tResidual * tResidual;
    }
    if(nTotalNum > 2){
        tPosFuncStd = sqrt(tPosFuncSum / (nTotalNum - 1));
        tRotFuncStd = sqrt(tRotFuncSum / (nTotalNum - 1));
    }else
    {
        tPosFuncStd = sqrt(tPosFuncSum );
        tRotFuncStd = sqrt(tRotFuncSum );
    }
    return 0;
}

// 5, GetObjFuncValue
template<typename T>
T Algorithm::GetObjFuncValue(const T tFuncPos, const T* darrQuatSrc, const T* darrQuatDest, const T tRotMiu, const T tPosMiu, T& tFuncQuat)
{
    tFuncQuat = QuatDiff(darrQuatSrc, darrQuatDest);                            // compute quaternion's object function value
    return tRotMiu * tFuncQuat + tPosMiu * tFuncPos;                            // compute compositive object function value
}

/*****************************************************************************
 Prototype    : Algorithm.GetObjFuncValue
 Description  : get the value of estimated comprehensive function according the estimated position function value.
 Input        : const double funcPos :  the estimated position function value
                double* quatSrc:   the original quaternion;       note that this data will be corrupted when function be called!
                double* quatDest:  the quaternion to be compared. note that this data will be corrupted when function be called!
                const double u1:        the weight of quaternion's object function value
                const double u2:        the weight of position's object function value
 Output       : None
 Return Value : double: the estimated function value according the estimated position function value.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/9/27
    Author       : lin.lin
    Modification : Created function
note: recommended!
*****************************************************************************/
template<typename T>
T Algorithm::GetObjFuncValue(const T tFuncPos, const T* darrQuatSrc, const T* darrQuatDest, const T tRotMiu, const T tPosMiu)
{
    T tFuncQuat = QuatDiff(darrQuatSrc, darrQuatDest);                            // compute quaternion's object function value
    return tRotMiu * tFuncQuat + tPosMiu * tFuncPos;                            // compute compositive object function value
}
// note: run time: about 1~2 us, 20180103 version
template<typename T>
T Algorithm::GetObjFuncValue(const T* darrQuatSrc, const T* darrPosSrc, const T* darrQuatDest, const T* darrPosDest, const T tRotMiu, const T tPosMiu)
{
    T tFuncPos  = Distance(darrPosSrc, darrPosDest);                             // get position's object function value
    T tFuncQuat = QuatDiff(darrQuatSrc, darrQuatDest);                           // compute quaternion's object function value
    return tRotMiu * tFuncQuat + tPosMiu * tFuncPos;                                  // compute compositive object function value
}

// 6, findNearest
/*****************************************************************************
 Prototype    : Algorithm.findNearest
 Description  : find the optimized postion and gesture for clip (the homogeneous matrix of clip) in the nearest current neighbourhood.
 Input        : darrDestClipHomo  : target clip's frame (4*4 Homogeneous Matrix relative to kuka base coordinate system),
                                    where the original point is on the half clip length, the x axis is along the clip from original point to endowrist point.
                darrTrocarPoint   : trocar point in kuka base coordinate system.
                shaftLength   : the length of shaft. The end of shaft is the pitch joint point.
                clipLength    : the length of clip, from yaw joint point to endowrist point.
                jointLength   : the length between pitch joint and yaw joint
                tAdjustShpereR: the radius of adjusted sphere around pitch joint
 Output       : dSrcYawDeg[in][out]  : input the current(original) yaw,   output the optimal yaw;   unit:  deg
                dSrcPitchDeg[in][out]: input the current(original) pitch, output the optimal pitch; unit:  deg
                darrOptFlangeHomo[in][out]: input the current flange frame and output the optimal flange frame (relative to kuka base coordinate system)
                pOutData:              used for unit test
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/11/22, 2017/11/29
    Author       : lin.lin
    Modification : Created function
note: recommended this version.
note2: sudo ./UnitTest --gtest_filter=ALGORITHM_TEST.findNearest
*****************************************************************************/
template<typename T>
void Algorithm::findNearest(const T* tarrDestClipHomo, const T* tarrTrocarPoint, const T shaftLength,  const T clipLength, const T jointLength,
    const T tMinYaw, const T tMaxYaw, const T tMinPitch, const T tMaxPitch, const T tMinClip, const T tMaxClip, T tAdjustShpereR,
    T &tSrcYawDeg, T &tSrcPitchDeg, T* tarrOptFlangeHomo, T* tarrAdjustPitchPoint, T* pOutData)
{

    // pointer used by log
    T* pCustomField = NULL;
    if ( NULL !=  pOutData)
    {
#ifdef UNIT_TEST_MODE
        pCustomField = &pOutData[UT_LOG_FIND_NEAREST_CUSTOM];
#else

#endif
    }
    // known conditions

    // tune r, p, y with const parameters tDelRollDeg, tDelPitchDeg, tDelYawDeg
    const T tDelRollDeg  = 0.5/1;  // kuka create jitter sonund when 0.5/2
    const T tDelPitchDeg = 0.5/1;
    const T tDelYawDeg   = 0.5/1;
    T tDeltAngleDeg[3]      = {tDelRollDeg, tDelPitchDeg, tDelYawDeg};// fine tuning angle. r, p, y; unit: deg

    T tarrSrcRollDeg[3]     = {0, tDeltAngleDeg[0], -tDeltAngleDeg[0]};         // roll list for angle adjust; unit: deg
    T tarrSrcPitchDeg[3]    = {tSrcPitchDeg, tSrcPitchDeg+tDeltAngleDeg[1], tSrcPitchDeg-tDeltAngleDeg[1]};             // pitch list for angle adjust; unit: deg
    T tarrSrcYawDeg[3]      = {tSrcYawDeg,   tSrcYawDeg  +tDeltAngleDeg[2],   tSrcYawDeg-tDeltAngleDeg[2]};             // yaw list for angle adjust; unit: deg


    T tarrSrcFlangePoint[3] = {tarrOptFlangeHomo[3], tarrOptFlangeHomo[7], tarrOptFlangeHomo[11]};                      // the source (original) flange's position
    T tarrSrcTrocarHomo[16];                                // trocar's old homo-matrix
    T tarrSrcPitchHomo[16];                                 // pitch's old homo-matrix
    T tarrPitchOldV[3];                                     // old point of pitch

    int nRollListId         = 0;
    int nPitchListId        = 0;
    int nYawListId          = 0;

    const int sizeRollList        = 3;                            // number of roll list
    const int sizePitchList       = 3;                            // number of pitch list
    const int sizeYawList         = 3;                            // number of yaw list

    const T tLongtitudeStepDeg    = 90;
    const T tLatitudeStepDeg      = 90;

    const int nLongtitudeNum      = 360 / tLongtitudeStepDeg;
    const int nLatitudeNum        = 180 / tLatitudeStepDeg -1;
    // number of neighbourhood vectors
    const int nNeighbourCnt       = nLongtitudeNum * nLatitudeNum + 3; // 267 or 15;
    const int nSizeTotal         = nNeighbourCnt * sizeRollList * sizePitchList * sizeYawList; // 405 = 15 * 3 * 3 * 3
    const T tarrNeighbourV[nNeighbourCnt][3] = {0.0};       // neighbourhood Vectors
    T tarrAdjustClipHomo[nSizeTotal][16]     = {0};         // adjusted clip's homoMatrix
    T tarrAdjustPitchHomo[nSizeTotal][16]    = {0};         // adjusted pitch's homoMatrix (redundant)
    T tarrAdjustYawHomo[nSizeTotal][16]      = {0};         // adjusted yaw's homoMatrix
    T tarrAdjustFlangeHomo[nSizeTotal][16]   = {0};         // adjusted flange's homoMatrix (not used)

    T tarrAdjustCurClipRot[9]             = {0.0};          // adjusted current clip's rotation matrix
    T tarrAdjustCurClipQuat[4];                             // adjust current clip's quaternion
    T tarrAdjustCurClipPoint[3];                            // adjust current clip's point


    T tarrDestClipRot[9]  = {0.0};                          // destinating clip's initial rotation matrix
    T tarrDestClipPoint[3];                                 // destinating clip's initial point
    T tarrDestClipQuat[4];

    T tSrcRollRad                         = 0.0;            // the roll of current adjust (radius)
    T tTrocarLen;                                           // the initial length between trocar and flange positon
    T dAdjustTrocarLen                    = 0.0;            // the length between adjusted pitch and trocar point

    T tRotFuncMean = 0.0;                                   // the mean of Quat function
    T tRotFuncStd   = 1;
    T tPosFuncMean  = 0.0;                                  // the mean of Position function
    T tPosFuncStd   = 1;

    T   nOptFunc    = 0;                                    // the optimal object's function; for debug
    int nOptFuncIndex   = 0;                                // the optimal index, for debug
    int nOptPitchPointId = 0;

    // function's value
    OBJ_FUNC<T>  obj_tFunc[nSizeTotal] = {0};
    OBJ_FUNC<T>* pObj_tFunc = obj_tFunc;
    const T tPenaltyFunc = 3.3e38;                          // initial's function's value (a penalty number)
    T tFuncInit          = tPenaltyFunc;

    int  nCount     = -1;
    int  nFuncCount = 0;

    int nDiscard = 0;
    // 1, get the initial length between trocar and flange positon, and the DH parameters(tTrocarLen, DH_param_total)
    T dRollDegDH    = TOOL_ROLL_DEGREE_TO_FLANGE;                                 // the mount angle (deg)
    int nRow        = 0;                                    // record the row of current used DH_param matrix
    // get trocar old length
    T tTrocarOldLen[3];                                     // the length between trocar and old flange positon
    tTrocarOldLen[0] = tarrTrocarPoint[0] - tarrSrcFlangePoint[0];
    tTrocarOldLen[1] = tarrTrocarPoint[1] - tarrSrcFlangePoint[1];
    tTrocarOldLen[2] = tarrTrocarPoint[2] - tarrSrcFlangePoint[2];
    tTrocarLen  = SQRT(tTrocarOldLen[0]*tTrocarOldLen[0] + tTrocarOldLen[1]*tTrocarOldLen[1]  + tTrocarOldLen[2]*tTrocarOldLen[2]);
    T DH_param_total[] =
#ifndef NEW_INSTR_DH
    {
        0,  0,            0,                      dRollDegDH,     //unit: deg
        0,  0,            tTrocarLen,             0,
        90, 0,            shaftLength-tTrocarLen, 90,
       -90, jointLength,  0,                      90+tSrcPitchDeg,
        0,  clipLength/2, 0,                      tSrcYawDeg
    };
#else
#ifndef NEW_INSTR_DH_GEAR
    {
        0,    0,               tTrocarLen,                  dRollDegDH, //unit: deg
        0,    0,               shaftLength - tTrocarLen,    0,
        90,   0,               0,                           90 + tSrcPitchDeg,
        90,   jointLength,     0,                           tSrcYawDeg,
        0,    clipLength / 2,  0,                           0
    };
#else
    {
        0,    0,               tTrocarLen,                  90.0 + dRollDegDH, //unit: deg
        0,    0,               shaftLength - tTrocarLen,    0,
        90,   0,               0,                           90 + tSrcPitchDeg,
        90,   jointLength,     0,                           tSrcYawDeg,
        0,    clipLength / 2,  0,                           0
    };
#endif
#endif
    DH_param_total[0]   *= PI_DIV_180;
    DH_param_total[3]   *= PI_DIV_180;

    DH_param_total[4]   *= PI_DIV_180;
    DH_param_total[7]   *= PI_DIV_180;

    DH_param_total[8]   *= PI_DIV_180;
    DH_param_total[11]  *= PI_DIV_180;

    DH_param_total[12]  *= PI_DIV_180;
    DH_param_total[15]  *= PI_DIV_180;

    DH_param_total[16]  *= PI_DIV_180;
    DH_param_total[19]  *= PI_DIV_180;

    // 2, compute trocar's and pitch's old homo-matrix: darrSrcTrocarHomo, darrSrcPitchHomo
#ifndef NEW_INSTR_DH
    nRow = 2 ;                                               //total row number
    FK_DH(tarrOptFlangeHomo, DH_param_total,         nRow, tarrSrcTrocarHomo);
#else
    nRow = 1 ;
    FK_DH(tarrOptFlangeHomo, DH_param_total, nRow, tarrSrcTrocarHomo, DH_CONVENTION_CRAIG);
#endif
    // get old pitch point
    tarrPitchOldV[0] = tarrAdjustPitchPoint[0];
    tarrPitchOldV[1] = tarrAdjustPitchPoint[1];
    tarrPitchOldV[2] = tarrAdjustPitchPoint[2];

    // 3, Create neighbourhood Vectors: tarrNeighbourV
    CreateAdjustedVectorsEx(tarrPitchOldV, tAdjustShpereR, tLongtitudeStepDeg, tLatitudeStepDeg, (T**)tarrNeighbourV/* nNeighbourCnt*3 */);

    // 4, get adjusted clip, pitch yaw and flange's Homo matrix: tarrAdjustClipHomo, darrAdjustPitchHomo, darrAdjustYawHomo, darrAdjustFlangeHomo
    int i, j, k, m, n;
    nRow = sizeof(DH_param_total) / (4 * sizeof(T));


    // 5, get target clip's rotation matrix, position and quaternion: darrDestClipRot, tarrDestClipPoint, tarrDestClipQuat
    for (int i2 = 0; i2 < 3; ++i2)
    {
        for (int j2 = 0; j2 < 3; ++j2)
        {
            tarrDestClipRot[i2*3+j2] = tarrDestClipHomo[i2*4+j2];
        }
        tarrDestClipPoint[i2] = tarrDestClipHomo[i2*4+3];
    }
    rotation2Quat(tarrDestClipRot, tarrDestClipQuat);

    // 6, get the position function value and relative context
    for (i = 0; i < nNeighbourCnt; ++i)
    {
        tarrAdjustPitchPoint[0] = tarrNeighbourV[i][0];
        tarrAdjustPitchPoint[1] = tarrNeighbourV[i][1];
        tarrAdjustPitchPoint[2] = tarrNeighbourV[i][2];
        dAdjustTrocarLen        = Distance(tarrAdjustPitchPoint, tarrTrocarPoint);  // new shaftLength-tTrocarLen
#ifndef NEW_INSTR_DH
        DH_param_total[6]       = shaftLength - dAdjustTrocarLen;  // new tTrocarLen
        DH_param_total[10]      = dAdjustTrocarLen;                // new shaftLength-tTrocarLen
#else
        DH_param_total[2]       = shaftLength - dAdjustTrocarLen;  // new tTrocarLen
        DH_param_total[6]       = dAdjustTrocarLen;                // new shaftLength-tTrocarLen
#endif
        for (j = 0; j < sizeRollList; ++j)
        {
            tSrcRollRad = tarrSrcRollDeg[j] * PI_DIV_180;

            for (k = 0; k < sizePitchList; ++k)
            {
#ifndef NEW_INSTR_DH
                DH_param_total[15] = (90 + tarrSrcPitchDeg[k]) * PI_DIV_180;
#else
              DH_param_total[11]   = (90 + tarrSrcPitchDeg[k]) * PI_DIV_180;
#endif
                for (m = 0; m < sizeYawList; ++m)
                {
                    nCount ++;
                    // renew the DH parameters
#ifndef NEW_INSTR_DH
                    DH_param_total[19] = (tarrSrcYawDeg[m]) * PI_DIV_180;
#else
                    DH_param_total[15] = (tarrSrcYawDeg[m]) * PI_DIV_180;
#endif
                    AdjustEx(tarrSrcTrocarHomo, tarrPitchOldV, tarrAdjustPitchPoint, tSrcRollRad, DH_param_total,
                                        tarrAdjustClipHomo[nCount],
                                        tarrAdjustPitchHomo[nCount],
                                        tarrAdjustYawHomo[nCount],          ///this parameter may be not useful.
                                        tarrAdjustFlangeHomo[nCount]);
                    // judge if out of range
                    if (tarrSrcYawDeg[m]   < tMinYaw    || tarrSrcYawDeg[m] > tMaxYaw ||
                        tarrSrcPitchDeg[k] < tMinPitch  || tarrSrcPitchDeg[k] > tMaxPitch )
                    {
                         nDiscard ++;
                    }else
                    {
                        tarrAdjustCurClipRot[0]   = tarrAdjustClipHomo[nCount][0];
                        tarrAdjustCurClipRot[1]   = tarrAdjustClipHomo[nCount][1];
                        tarrAdjustCurClipRot[2]   = tarrAdjustClipHomo[nCount][2];

                        tarrAdjustCurClipRot[3]   = tarrAdjustClipHomo[nCount][4];
                        tarrAdjustCurClipRot[4]   = tarrAdjustClipHomo[nCount][5];
                        tarrAdjustCurClipRot[5]   = tarrAdjustClipHomo[nCount][6];

                        tarrAdjustCurClipRot[6]   = tarrAdjustClipHomo[nCount][8];
                        tarrAdjustCurClipRot[7]   = tarrAdjustClipHomo[nCount][9];
                        tarrAdjustCurClipRot[8]   = tarrAdjustClipHomo[nCount][10];

                        tarrAdjustCurClipPoint[0] = tarrAdjustClipHomo[nCount][3];
                        tarrAdjustCurClipPoint[1] = tarrAdjustClipHomo[nCount][7];
                        tarrAdjustCurClipPoint[2] = tarrAdjustClipHomo[nCount][11];

                        tarrAdjustPitchPoint[0] = tarrNeighbourV[i][0];
                        tarrAdjustPitchPoint[1] = tarrNeighbourV[i][1];
                        tarrAdjustPitchPoint[2] = tarrNeighbourV[i][2];

                        // 8, get object's function value: tFunc
//                        tFuncPos[nCount] = Distance(darrInitClipPoint, tarrAdjustCurClipPoint);
//                        if ( dFuncPosMean < tFuncPos[nCount] )
//                        {
//                            nDiscard++;
//                            continue; //discard
//                        }else
                        {
                            rotation2Quat(tarrAdjustCurClipRot, tarrAdjustCurClipQuat);
                            obj_tFunc[nFuncCount].tPosEvaluateValue = Distance(tarrDestClipPoint, tarrAdjustCurClipPoint);
                            obj_tFunc[nFuncCount].tRotEvaluateValue = getErrO(tarrDestClipQuat,  tarrAdjustCurClipQuat);

                            obj_tFunc[nFuncCount].nID                 = nCount;
                            obj_tFunc[nFuncCount].nOptPitchPointId    = i;
                            obj_tFunc[nFuncCount].nRollListId         = j;
                            obj_tFunc[nFuncCount].nPitchListId        = k;
                            obj_tFunc[nFuncCount].nYawListId          = m;
                            obj_tFunc[nFuncCount].bIsInRange          = true;//redunt
                            nFuncCount++;
                        }
                    }
                }
            }
        }
    }

    // 7, get the object function's  mean value and standard deviation
    getMeanAndStd(pObj_tFunc, nFuncCount, tPosFuncMean, tRotFuncMean, tPosFuncStd, tRotFuncStd);

    // 8, get the optimal pitch, yaw, index, function value and flange's homoMatrix
    for ( nCount = 0 ; nCount <nFuncCount ; ++nCount )
    {
        // Normalized function value: (x-miu)/sigma
        obj_tFunc[nCount].tPosEvaluateValue = (obj_tFunc[nCount].tPosEvaluateValue - tPosFuncMean) / tPosFuncStd;//zero-mean normalization
        obj_tFunc[nCount].tRotEvaluateValue = (obj_tFunc[nCount].tRotEvaluateValue - tRotFuncMean) / tRotFuncStd;
        obj_tFunc[nCount].tEvaluateValue    = obj_tFunc[nCount].tPosEvaluateValue + obj_tFunc[nCount].tRotEvaluateValue;
        // get the optimal pitch, yaw, index, function value and flange's homoMatrix:
        if ( obj_tFunc[nCount].tEvaluateValue <  tFuncInit)
        {
            // get current pitch, yaw and index
            nOptFuncIndex     = obj_tFunc[nCount].nID;      // get nOptFuncIndex; and then get darrAdjustPitchHomo[nOptFuncIndex] subsequent
            nOptPitchPointId  = obj_tFunc[nCount].nOptPitchPointId;
            nRollListId       = obj_tFunc[nCount].nRollListId;
            nPitchListId      = obj_tFunc[nCount].nPitchListId;
            nYawListId        = obj_tFunc[nCount].nYawListId;
            tFuncInit         = obj_tFunc[nCount].tEvaluateValue;
        }
    }

    // get optimized result
    tarrAdjustPitchPoint[0] = tarrNeighbourV[nOptPitchPointId][0];
    tarrAdjustPitchPoint[1] = tarrNeighbourV[nOptPitchPointId][1];
    tarrAdjustPitchPoint[2] = tarrNeighbourV[nOptPitchPointId][2];
    tSrcPitchDeg            = tarrSrcPitchDeg[nPitchListId];//unit: deg
    tSrcYawDeg              = tarrSrcYawDeg[nYawListId];
    memcpy(tarrOptFlangeHomo, tarrAdjustFlangeHomo[nOptFuncIndex], sizeof(T)*16);
    nOptFunc                = obj_tFunc[nOptFuncIndex].tEvaluateValue;

#ifdef UT_LOG_SWITCH
    // 8, write log
    if ( NULL != pOutData )
    {
#ifdef UNIT_TEST_MODE
        memcpy(pCustomField, tarrAdjustClipHomo[nOptFuncIndex], sizeof(T)*16);//pCustomField[0~15]: tarrAdjustClipHomo
        pCustomField[16] = 0.0;
        pCustomField[17] = 0.0;
        pCustomField[18] = tRotFuncMean;
        pCustomField[19] = tPosFuncMean;
        pCustomField[20] = tarrAdjustPitchPoint[0];    // Adjust Pitch point
        pCustomField[21] = tarrAdjustPitchPoint[1];
        pCustomField[22] = tarrAdjustPitchPoint[2];
        pCustomField[23] = tarrPitchOldV[0];    // reserved!
        pCustomField[24] = tarrPitchOldV[1];
        pCustomField[25] = tarrPitchOldV[2];
        pCustomField[26] = tAdjustShpereR;             // Distance(tarrAdjustPitchPoint, darrTargetPitchPoint);  // adjust R
        pCustomField[27] = obj_tFunc[nOptFuncIndex].tPosEvaluateValue;        // position function value
        pCustomField[28] = obj_tFunc[nOptFuncIndex].tRotEvaluateValue;        // quat function value
        pCustomField[29] = obj_tFunc[nOptFuncIndex].tEvaluateValue;           // function value
        pCustomField[30] = tarrSrcRollDeg[nRollListId];  // adjust roll
#else

#endif
    }
#endif

#define GET_ADJUST_MODE
#ifdef GET_ADJUST_MODE
        memcpy(pOutData, tarrAdjustClipHomo[nOptFuncIndex], sizeof(T)*16);
#endif

#ifdef DBG_ALGORITHM_
    // sudo ./UnitTest --gtest_filter=ALGORITHM_TEST.findNearest
    // 1, get the initial length between trocar and flange positon, and the DH parameters(tTrocarLen, DH_param_total)
    printf("1, get the initial length between trocar and flange positon, and the DH parameters(tTrocarLen, DH_param_total): \n");
    cout << "tTrocarLen = " << tTrocarLen <<endl;
    cout << "darrSrcFlangePoint = " << endl;
    for ( i = 0 ; i <3 ; i++ )
    {
        cout << tarrSrcFlangePoint[i] << "\t";
    }
    cout << endl;
    // 2, get trocar's and pitch's old homo-matrix: darrSrcTrocarHomo, darrSrcPitchHomo
    printf("2, get trocar's and pitch's old homo-matrix: darrSrcTrocarHomo, darrSrcPitchHomo \n");
    printf("output darrSrcTrocarHomo: \n");
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", tarrSrcTrocarHomo[i*4+j]);
        }
        printf("\n");
    }
    printf("output darrDestClipHomo: \n");
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", tarrDestClipHomo[i*4+j]);
        }
        printf("\n");
    }
    printf("output darrSrcPitchHomo: \n");
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", tarrSrcPitchHomo[i*4+j]);
        }
        printf("\n");
    }
    // 3, Create neighbourhood Vectors: tarrNeighbourV
    printf("3, Create neighbourhood Vectors: tarrNeighbourV: \n");
    for ( i = 0 ; i <3 ; ++i )
    {
        for ( j = 0 ; j <15 ; ++j )
        {
            printf("%7.4f\t", tarrNeighbourV[j][i]);
        }
        printf("\n");
    }
    printf("output darrInitClipPoint: \n");
    for ( i = 0 ; i <3; ++i )
    {
        printf("%7.4f\t", tarrInitClipPoint[i]);
    }
    printf("\n");
    // 4, get adjusted clip, pitch yaw and flange's Homo matrix: tarrAdjustClipHomo, darrAdjustPitchHomo, darrAdjustYawHomo, darrAdjustFlangeHomo
    printf("4, get adjusted clip, pitch yaw and flange's Homo matrix: tarrAdjustClipHomo, darrAdjustPitchHomo, darrAdjustYawHomo, darrAdjustFlangeHomo: \n");
    cout << "tarrAdjustClipHomo[nOptFuncIndex] = " << endl;
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", tarrAdjustClipHomo[nOptFuncIndex][i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    cout << "darrAdjustPitchHomo[nOptFuncIndex] = " << endl;
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", tarrAdjustPitchHomo[nOptFuncIndex][i*4+j]);
        }
        printf("\n");
    }
    cout << "darrAdjustYawHomo[nOptFuncIndex] = " << endl;
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", tarrAdjustYawHomo[nOptFuncIndex][i*4+j]);
        }
        printf("\n");
    }
    cout << "darrAdjustFlangeHomo[nOptFuncIndex] = " << endl;
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", tarrAdjustFlangeHomo[nOptFuncIndex][i*4+j]);
        }
        printf("\n");
    }
    // 5, get the object function's weight tRotMiu, tPosMiu, dFuncQuatMean, dFuncPosMean
    printf("5, get the object function's weight tRotMiu, tPosMiu: \n");
    cout << "tRotMiu, tPosMiu, dFuncQuatMean, dFuncPosMean = "
        << tRotMiu << "    " << tPosMiu << "    "
        << tRotFuncMean << "    " << tPosFuncMean << "    "
        << endl; // tEvaluateValue, line, tRotMiu, tPosMiu, dFuncQuatMean, dFuncPosMean = 393.163    0.662673    0.00254348    1.50904

    // 6, get target clip's rotation matrix, position and quaternion: darrDestClipRot, tarrDestClipPoint, tarrDestClipQuat
    printf("6, get target clip's tarrDestClipQuat: \n");
    for ( i = 0 ; i <4 ; ++i )
    {
        printf("%7.4f\t", tarrDestClipQuat[i]);
    }
    printf("\n");
    // 7, renew the DH parameters according current adjusted status: DH_param_total, get adjusted clip and flange's Homo matrix
    printf("7, renew the DH parameters according current adjusted status: DH_param_total, get adjusted clip and flange's Homo matrix: \n");
    cout << "tarrAdjustClipHomo[nOptFuncIndex] = " << endl;
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", tarrAdjustClipHomo[nOptFuncIndex][i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%7.4f\t", tarrAdjustFlangeHomo[nOptFuncIndex][i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    // 8, get object's function value: tEvaluateValue
    printf("8, get object's function value: tEvaluateValue: \n");
    //cout << "nOptFunc, tEvaluateValue[nOptFuncIndex] = " << tEvaluateValue << endl;  // this version: tEvaluateValue = 3.12715
    printf("%8.4f, %8.4f\t", nOptFunc, obj_tFunc[nOptFuncIndex].tEvaluateValue);

    // 9, get the optimal pitch, yaw, index, function value and flange's homoMatrix:
    printf("9, get the optimal pitch, yaw, index, function value and flange's homoMatrix:  dSrcPitchDeg, dSrcYawDeg, nOptFuncIndex, nOptFunc, darrOptFlangeHomo\n");
    cout << "optimal dSrcPitchDeg, dSrcYawDeg, nOptFuncIndex, nOptFunc value = "
         << tSrcPitchDeg << "    " << tSrcYawDeg <<  "    "
         << nOptFuncIndex    << "    " << nOptFunc   <<  "    " << endl; // matlab:  [optYaw, optPitch, optRoll, nOptFunc] = -23.4683  -39.5588  0  0.0193; this version: optimal pitch, yaw, index, function value = -39.5588    -23.4683    397    1.05791
    cout << "optimal darrOptFlangeHomo value = " << endl;
    for ( i = 0 ; i <4 ; ++i )
    {
        for ( j = 0 ; j <4 ; ++j )
        {
            printf("%10.4f\t", tarrOptFlangeHomo[i*4+j]);
        }
        printf("\n");
    }
    cout << "AdjustPitch Point: " << endl;
    printf("%8.4f, %8.4f, %8.4f\n", tarrAdjustPitchHomo[nOptFuncIndex][3], tarrAdjustPitchHomo[nOptFuncIndex][7], tarrAdjustPitchHomo[nOptFuncIndex][11]);
    cout << "the opt nOptPitchPointId, nRollListId,  nPitchListId,  nYawListId: " << endl;
    printf("%d, %d, %d, %d\n", nOptPitchPointId, nRollListId,  nPitchListId,  nYawListId);
    cout << "the opt roll: " << endl;
    printf("%d, %d, %d, %d\n", nOptPitchPointId, nRollListId,  nPitchListId,  nYawListId);
#endif
}

// 7, setFrameAndClip
/*****************************************************************************
 Prototype    : Algorithm.getPitchYawByGeo
 Description  : get pitch and yaw by Geometry.
 Input        : clipHomo:     destination clip's homo-matrix (the fifth coordinate defined by Haoluo.Ning, 4*4 Homogeneous Matrix relative to kuka base coordinate system),
                              where the original point is on the half clip length, the x axis is along the clip from original point to clip point.
                pdTrocarPoint:trocar point in slave world coordinate system
                shaftLength:  shaft length of instrument; The end of shaft is the pitch joint point;
                clipLength:   clip's length;
                jointLength:  palm's length; the length between pitch joint and yaw joint;
                pitch:        the 1st root of pitch; unit:  radian
                yaw:          the 1st root of yaw;   unit:  radian
                pitch2:       the 2nd root of pitch; unit:  radian
                yaw2:         the 2nd root of yaw;   unit:  radian
                pDataOut:     default value: NULL; interface to log; use this pointer to output data to log;
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/12/4
    Author       : lin.lin
    Modification : Created function
note: run time: about 5 us, 20180103 version
*****************************************************************************/
template<typename T>
void Algorithm::getPitchYawByGeo(const T* clipHomo, const T* pdTrocarPoint,
  const T shaftLength, const T clipLength, const T jointLength,
  T& pitch, T& yaw, T& pitch2, T& yaw2, T* pDataOut)
{
    //cout << "func, line = " << __func__ << "  " << __LINE__ << endl;
    T darrClipPoint[3] = {clipHomo[3], clipHomo[7], clipHomo[11]};         //clip point, at the point of 1/2 of the clip.
    T darrClipR[9] = {0.0};                            // store the rotation matrix of clipHomo

    T darrClip2YawPoint[3]    = {0.0};                 // vector from clip to yaw point in the Slave World coordinate system
    T darrYawPoint[3]         = {0.0};                 // yaw point in kuka base coordinate system;
    T darrClipPointInLocal[3] = {-clipLength/2, 0, 0}; // clip point in clip coordinate system (witch original point is in the middle of clip)

    T A = 0.0, B = 0.0, C = 0.0, D = 0.0, t = 0.0;     // the paramters which decided the plane equation formed by two clips: Ax + By + Cz + D = 0;
    T darrTrocarProject[3] = {0.0};                    // trocar projection point
    T darrYaw2TrocarProPoint[3] = {0.0};               // the vector from yaw to trocar projection point

    T dNormOfYaw2TrocarProPoint = 0.0;                 // norm(Yaw2TrocarProjectPoint)
    T darrPitchPointTemp[2][3] = {0.0};                // to store 2 solution of pitch point
    T darrPitchPoint[3] = {0.0};                       // store pitch point

    T darrPitch2YawPoint[3] = {0.0};                   // vector from pitch to yaw point (v1)
    T darrTro2PitchPoint[3] = {0.0};                   // vector from trocar to pitch point (v2)
    T darrYaw2ClipPoint[3]  = {0.0};                   // vector from yaw to clip point (v3)
    T dNormOfdarrPitch2YawPoint = 0.0;                 // norm(v1)
    T dNormOfdarrTro2PitchPoint = 0.0;                 // norm(v2)
    T dNormOfdarrYaw2ClipPoint  = 0.0;                 // norm(v3)
    T dVectorDot = 0.0;                                // v3 . v1; v1: darrPitch2YawPoint; v3: darrYaw2ClipPoint
    T darrYawCross[3] = {0.0};                         // v1 * v3
    T darrzInWorld[3] = {0, 0, 0};                     // unit vector z of the clip coordinate system in the Slave world coordinate system
    T darrzInLocal[3] = {0, 0, 1};                     // unit vector z of the clip coordinate system (witch original point is in the middle of clip)
    T flagRaw = 0.0;                                   // the sign flag of yaw
    T darrRotYaw[9] = {0.0};                           // to store current pitch rotation when clip coordinate system rotate yaw along clip's z
    T darrPitchR[9] = {0.0};                           // temp variable
    T dVectorDot2 = 0.0;                               // v2 . v1; v2: darrTro2PitchPoint
    T darrPitchCross[3] = {0.0};                       // v2 * v1: darrTro2PitchPoint * darrPitch2YawPoint
    T darryInLocal[3] = {0, 1, 0};                     // y axis direction in pitch(wrist 2) coordinate system
    T darryInWorld[3] = {0.0};                         // transform the y axis in pitch(wrist 2) coordinate system to slave world coordinate system
    T dFlagPitch = 0.0;                                // the sign flag of yaw
    T dHomoCur[16]   = {0.0};                          // T1*T2*T3*T4
    T dHomoDHInv[HOMO_MATRIX_ELEM_CNT] = {0.0};        // inv(T1234)
    T arrOriginTemp[HOMO_MATRIX_ELEM_CNT] = {0.0};     // temp homoMatrix that store the middle result while calculating T1*T2*T3*T4
    T darrTi[4][16] = {0.0};                           // store  T1, T2, T3, T4

    int i = 0, j = 0;
    T rTest = 0.0, pTest = 0.0, yTest = 0.0;

  // 1, caculate the point of yaw joint in the Slave World coordinate system
    for ( i = 0 ; i <3 ; ++i )
    {
        for ( j = 0 ; j <3 ; ++j )
        {
            darrClipR[i*3+j] = clipHomo[i*4+j];
        }
    }
    //GeometryUtils::MatrixMultiply(darrClipR, darrClipPointInLocal, 3, 3, 1, darrClip2YawPoint);
    matrix3x1Multiply(darrClipR, darrClipPointInLocal, darrClip2YawPoint);
    darrYawPoint[0] = darrClipPoint[0] + darrClip2YawPoint[0];
    darrYawPoint[1] = darrClipPoint[1] + darrClip2YawPoint[1];
    darrYawPoint[2] = darrClipPoint[2] + darrClip2YawPoint[2];//IK4

  // 2, caculate the projection point of Trocar, and projection direction (trocar project to the plane formed by 2 clips)
    // the normal line of the plane which is formed by two clips in the initial zero pose is axis Z+.
    // the plane equation formed by two clips is: Ax + By + Cz + D = 0; [A;B;C] = endowristR * [0;0;1];
    A = darrClipR[2];
    B = darrClipR[5];
    C = darrClipR[8];// normal direction; along with the z axis in the clip coordinate system;
    D = -(A*darrYawPoint[0] + B*darrYawPoint[1] + C*darrYawPoint[2]);
    // trocar projection: compute trocar projection coordinate in this plane. This problem is equivalent to find the crossover point between a perpendicular through trocar point and the clips' plane.
    // Because it is a projection, the line must be perpendicular to the plane, The direction vector of the perpendicular line is A, B, C. The parametric equation of the perpendicular line is:
    // X = X_trocar + t * A; Y = Y_trocar + t * B; Z = Z_trocar + t * C;   (Point Normal Formula) See the details: page 61 of analytic geometry.
    //get trocar projection on that plane
    t = -(A * pdTrocarPoint[0] + B * pdTrocarPoint[1]+ C * pdTrocarPoint[2] + D) / (A * A + B * B + C * C); // reference Distance formula from point to plane
    darrTrocarProject[0] = pdTrocarPoint[0] + A * t;
    darrTrocarProject[1] = pdTrocarPoint[1] + B * t;
    darrTrocarProject[2] = pdTrocarPoint[2] + C * t;
    // get the point of the end of shaft ; (vector from yawJointPoint to trocar projection point)
    darrYaw2TrocarProPoint[0] = darrTrocarProject[0] - darrYawPoint[0];
    darrYaw2TrocarProPoint[1] = darrTrocarProject[1] - darrYawPoint[1];
    darrYaw2TrocarProPoint[2] = darrTrocarProject[2] - darrYawPoint[2];

  // 3, get pitchJointPoint in kuka base coordinate system (end of shaft)
    // get v3
    dNormOfdarrYaw2ClipPoint  = Distance(darrClipPoint, darrYawPoint);
    darrYaw2ClipPoint[0]  = darrClipPoint[0]  - darrYawPoint[0];
    darrYaw2ClipPoint[1]  = darrClipPoint[1]  - darrYawPoint[1];
    darrYaw2ClipPoint[2]  = darrClipPoint[2]  - darrYawPoint[2];
    darrYaw2ClipPoint[0]  /= dNormOfdarrYaw2ClipPoint;
    darrYaw2ClipPoint[1]  /= dNormOfdarrYaw2ClipPoint;
    darrYaw2ClipPoint[2]  /= dNormOfdarrYaw2ClipPoint;

    dNormOfYaw2TrocarProPoint = Norm(darrYaw2TrocarProPoint);
    if(dNormOfYaw2TrocarProPoint <= ALGORITHM_EPS_TYPENAME)//Two points (trocarProject and YawPoint) coincide
    {
        // two roots are equal
        darrPitchPointTemp[1][0] = darrPitchPointTemp[0][0] = darrYawPoint[0] - jointLength *darrYaw2ClipPoint[0];
        darrPitchPointTemp[1][1] = darrPitchPointTemp[0][1] = darrYawPoint[1] - jointLength *darrYaw2ClipPoint[1];
        darrPitchPointTemp[1][2] = darrPitchPointTemp[0][2] = darrYawPoint[2] - jointLength *darrYaw2ClipPoint[2]; // this solution may be not suitable
    }else
    {
        darrPitchPointTemp[0][0] = darrYawPoint[0] + jointLength * (darrYaw2TrocarProPoint[0])/dNormOfYaw2TrocarProPoint;
        darrPitchPointTemp[0][1] = darrYawPoint[1] + jointLength * (darrYaw2TrocarProPoint[1])/dNormOfYaw2TrocarProPoint;
        darrPitchPointTemp[0][2] = darrYawPoint[2] + jointLength * (darrYaw2TrocarProPoint[2])/dNormOfYaw2TrocarProPoint;

        darrPitchPointTemp[1][0] = darrYawPoint[0] - jointLength * (darrYaw2TrocarProPoint[0])/dNormOfYaw2TrocarProPoint;
        darrPitchPointTemp[1][1] = darrYawPoint[1] - jointLength * (darrYaw2TrocarProPoint[1])/dNormOfYaw2TrocarProPoint;
        darrPitchPointTemp[1][2] = darrYawPoint[2] - jointLength * (darrYaw2TrocarProPoint[2])/dNormOfYaw2TrocarProPoint;
    }
    // 4, get yaw and pitch
    //-------------------- solution 1 of pitch point: -----------------------------------------
    darrPitchPoint[0] = darrPitchPointTemp[0][0];
    darrPitchPoint[1] = darrPitchPointTemp[0][1];
    darrPitchPoint[2] = darrPitchPointTemp[0][2];

    for ( i = 0 ; i <3 ; ++i )
    {
        darrPitch2YawPoint[i] = darrYawPoint[i]   - darrPitchPoint[i];          //v1
        darrTro2PitchPoint[i] = darrPitchPoint[i] - pdTrocarPoint[i];           //v2

    }
    dNormOfdarrPitch2YawPoint = Norm(darrPitch2YawPoint);
    dNormOfdarrTro2PitchPoint = Norm(darrTro2PitchPoint);

    for ( i = 0 ; i <3 ; ++i )
    {
        darrPitch2YawPoint[i] /= dNormOfdarrPitch2YawPoint; //v1
        darrTro2PitchPoint[i] /= dNormOfdarrTro2PitchPoint; //v2

    }

    dVectorDot = vectorDot(darrYaw2ClipPoint, darrPitch2YawPoint);//v3.v1
    if ( fabs(dVectorDot-1.0) < EPS_IK_TYPENAME)
    {
        dVectorDot = 1.0;
    }else if ( fabs(dVectorDot+1.0) < EPS_IK_TYPENAME)
    {
        dVectorDot = -1.0;
    }
    yaw = acos(dVectorDot);     // yaw = [0��PI]
    vectorCross(darrPitch2YawPoint, darrYaw2ClipPoint, darrYawCross); // v1 * v3
    // transform the z axis in clip coordinate system to kuka base coordinate system
    //GeometryUtils::MatrixMultiply(darrClipR, darrzInLocal, 3, 3, 1, darrzInWorld);
    matrix3x1Multiply(darrClipR, darrzInLocal, darrzInWorld);
    flagRaw = vectorDot(darrzInWorld, darrYawCross);
    // yaw direction flag
    if(flagRaw < -ALGORITHM_EPS_TYPENAME)
    {
        yaw = -yaw;
    }

    // (get pitch )
    rotateZ(-yaw, darrRotYaw);
    matrix3x3Multiply(darrClipR, darrRotYaw, darrPitchR);
    dVectorDot2 = vectorDot(darrTro2PitchPoint, darrPitch2YawPoint);// v2 . v1
    if ( dVectorDot2 > 1 && dVectorDot2-1.0 < EPS_IK_TYPENAME)                           // little bigger than 1
    {
        dVectorDot2 = 1.0;
    }else if ( dVectorDot2 < -1 && dVectorDot2+1.0 > -EPS_IK_TYPENAME)                   //little less than -1
    {
        dVectorDot2 = -1.0;
    }
    pitch = acos(dVectorDot2);
    vectorCross(darrTro2PitchPoint, darrPitch2YawPoint, darrPitchCross);//v2 * v1
    //GeometryUtils::MatrixMultiply(darrPitchR, darryInLocal, 3, 3, 1, darryInWorld);
    matrix3x1Multiply(darrPitchR, darryInLocal, darryInWorld);
    dFlagPitch = vectorDot(darryInWorld, darrPitchCross);        // pitch direction flag
    if(dFlagPitch > ALGORITHM_EPS_TYPENAME)
    {
        pitch = -pitch;
    }
#ifdef dbg_IK_ex_new
    homo2RPY(clipHomo, rTest, pTest, yTest);
    cout << "1, calculate the clip and yaw points: " << endl;
    cout << "homo2RPY: " << rTest*180/PI << "\t" << pTest*180/PI << "\t" << yTest*180/PI << endl;
    cout << "clipPoint = " << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%30.20f\t", darrClipPoint[i]);
    }
    printf("\n");
    cout << "yawJointPoint = " << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%30.20f\t", darrYawPoint[i]);
    }
    printf("\n");
    cout << "pdTrocarPoint = " << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%30.20f\t", pdTrocarPoint[i]);
    }
    printf("\n");

    cout << "2, calculate the trocar projection, darrYaw2TrocarProPoint: " << endl;
    cout << "trocar_project = " << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%30.20f\t", darrTrocarProject[i]);
    }
    printf("\n");
    cout << "darrYaw2TrocarProPoint = " << endl;
    //cout << darrYaw2TrocarProPoint[0] << "    "  << darrYaw2TrocarProPoint[1] << "    "  << darrYaw2TrocarProPoint[2] << "    " << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%30.20f\t", darrYaw2TrocarProPoint[i]);
    }
    printf("\n");
    cout << "3, calculate pitchJointPoint: " << endl;
    cout << "pitchJointPoint = " << endl;
    //cout << darrPitchPoint[0] << "    "  << darrPitchPoint[1] << "    "  << darrPitchPoint[2] << "    " << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%30.20f\t", darrPitchPoint[i]);
    }
    printf("\n");

    cout << "4, get yaw and pitch: " << endl;
    cout << yaw << "\t" << pitch << endl;

    cout << "darrPitch2YawPoint, darrTro2PitchPoint, darrYaw2ClipPoint " << endl;
    for ( i = 0 ; i <3 ; i++ )
    {
        printf("%30.20f\t", darrPitch2YawPoint[i]);
    }
    cout << endl;
    for ( i = 0 ; i <3 ; i++ )
    {
        printf("%30.20f\t", darrTro2PitchPoint[i]);
    }
    cout << endl;
    for ( i = 0 ; i <3 ; i++ )
    {
        printf("%30.20f\t", darrYaw2ClipPoint[i]);
    }
    cout << endl;
    cout << "darrYawCross, darrPitchCross " << endl;
    for ( i = 0 ; i <3 ; i++ )
    {
        printf("%30.20f\t", darrYawCross[i]);
    }
    cout << endl;
    for ( i = 0 ; i <3 ; i++ )
    {
        printf("%30.20f\t", darrPitchCross[i]);
    }
    cout << endl;
    cout << "dVectorDot = "  << dVectorDot  << endl;
    cout << "dVectorDot2 = " << dVectorDot2 << endl;
    cout << "//////////////////////////////////////////////" << endl;
#endif

    //-------------------- solution 2 of pitch point : -----------------------------------------
    darrPitchPoint[0] = darrPitchPointTemp[1][0];
    darrPitchPoint[1] = darrPitchPointTemp[1][1];
    darrPitchPoint[2] = darrPitchPointTemp[1][2];

    for ( i = 0 ; i <3 ; ++i )
    {
        darrPitch2YawPoint[i] = darrYawPoint[i]   - darrPitchPoint[i];
        darrTro2PitchPoint[i] = darrPitchPoint[i] - pdTrocarPoint[i];
        darrYaw2ClipPoint[i]  = darrClipPoint[i]  - darrYawPoint[i];
    }
    dNormOfdarrPitch2YawPoint = Norm(darrPitch2YawPoint);
    dNormOfdarrTro2PitchPoint = Norm(darrTro2PitchPoint);
    dNormOfdarrYaw2ClipPoint  = Norm(darrYaw2ClipPoint);
    for ( i = 0 ; i <3 ; ++i )
    {
        darrPitch2YawPoint[i] /= dNormOfdarrPitch2YawPoint;
        darrTro2PitchPoint[i] /= dNormOfdarrTro2PitchPoint;
        darrYaw2ClipPoint[i]  /= dNormOfdarrYaw2ClipPoint;
    }

    // get yaw and pitch
    dVectorDot = vectorDot(darrYaw2ClipPoint, darrPitch2YawPoint);
    if ( fabs(dVectorDot-1.0) < EPS_IK_TYPENAME)
    {
        dVectorDot = 1.0;
    }else if ( fabs(dVectorDot+1.0) < EPS_IK_TYPENAME)
    {
        dVectorDot = -1.0;
    }
    yaw2 = acos(dVectorDot);
    vectorCross(darrPitch2YawPoint, darrYaw2ClipPoint, darrYawCross);
    // transform the z axis in clip coordinate system to kuka base coordinate system
    //GeometryUtils::MatrixMultiply(darrClipR, darrzInLocal, 3, 3, 1, darrzInWorld);
    matrix3x1Multiply(darrClipR, darrzInLocal, darrzInWorld);
    flagRaw = vectorDot(darrzInWorld, darrYawCross);
    // yaw direction flag
    if(flagRaw < -ALGORITHM_EPS_TYPENAME)
    {
        yaw2 = -yaw2;
    }

    // (get pitch )
    rotateZ(-yaw2, darrRotYaw);
    matrix3x3Multiply(darrClipR, darrRotYaw, darrPitchR);
    dVectorDot2 = vectorDot(darrTro2PitchPoint, darrPitch2YawPoint);
    if ( dVectorDot2 > 1 && dVectorDot2-1.0 < EPS_IK_TYPENAME)                           // little bigger than 1
    {
        dVectorDot2 = 1.0;
    }else if ( dVectorDot2 < -1 && dVectorDot2+1.0 > -EPS_IK_TYPENAME)                   //little less than -1
    {
        dVectorDot2 = -1.0;
    }
    pitch2 = acos(dVectorDot2);
    vectorCross(darrTro2PitchPoint, darrPitch2YawPoint, darrPitchCross);
    //GeometryUtils::MatrixMultiply(darrPitchR, darryInLocal, 3, 3, 1, darryInWorld);
    matrix3x1Multiply(darrPitchR, darryInLocal, darryInWorld);
    dFlagPitch = vectorDot(darryInWorld, darrPitchCross);        // pitch direction flag
    if(dFlagPitch > ALGORITHM_EPS_TYPENAME)
    {
        pitch2 = -pitch2;
    }

    // write data to log
    if (NULL != pDataOut)
    {
        for ( i = 0 ; i <3 ; i++ )
        {
            pDataOut[i]      = darrClipPoint[i];          // clip point, at the point of 1/2 of the clip
            pDataOut[3  + i] = darrYawPoint[i];           // yaw point in kuka base coordinate system
            pDataOut[6  + i] = darrTrocarProject[i];      // trocar projection point
            pDataOut[9  + i] = darrPitchPointTemp[0][i];  // solution 1 of pitch point
            pDataOut[12 + i] = darrPitchPointTemp[1][i];  // solution 2 of pitch point
        }
    }
}
template<typename T>
void Algorithm::eye(T* tarrM, int nOrder)
{
    int i, j;
    if  ( nOrder <= 0 )
    {
        nOrder = 0;
        //return;
    }
    for ( i = 0 ; i < nOrder ; ++i )
    {
        for ( j = 0 ; j < nOrder ; ++j )
        {
            if ( i == j )
            {
                tarrM[i*nOrder+j] = 1.0;
            }else
            {
                tarrM[i*nOrder+j] = 0.0;
            }
        }
    }
}

// private:
// from GeometryUtils
template<typename T>
inline T Algorithm::Norm(const T arrd[3])
{
    return SQRT(arrd[0]*arrd[0] + arrd[1]*arrd[1]  + arrd[2]*arrd[2]);
}

/*****************************************************************************
 Prototype    : Algorithm.Distance
 Description  : the distance between 2 points
 Input        : const T a[3]: first point
                const T b[3]: second point
 Output       : None
 Return Value : inline
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/1/3
    Author       : lin.lin
    Modification : Created function
  note: run time: about 860 ns, 20180103 version
*****************************************************************************/
template<typename T>
inline T Algorithm::Distance(const T a[3], const T b[3])
{
    return SQRT((a[0] - b[0]) * (a[0] - b[0]) +
                (a[1] - b[1]) * (a[1] - b[1]) +
                (a[2] - b[2]) * (a[2] - b[2]));
}

template<typename T>
inline T Algorithm::vectorDot(const T arrd1[3], const T arrd2[3])
{
    return (arrd1[0]*arrd2[0] + arrd1[1]*arrd2[1]  + arrd1[2]*arrd2[2]);
}
template<typename T>
inline void Algorithm::vectorCross(const T arrdSrcA[3], const T arrdSrcB[3], T arrdRelsult[3])
{
    arrdRelsult[0] = arrdSrcA[1] * arrdSrcB[2] - arrdSrcA[2] * arrdSrcB[1];
    arrdRelsult[1] = arrdSrcA[2] * arrdSrcB[0] - arrdSrcA[0] * arrdSrcB[2];
    arrdRelsult[2] = arrdSrcA[0] * arrdSrcB[1] - arrdSrcA[1] * arrdSrcB[0];
}
template<typename T>
inline void Algorithm::rotateX(const T angle, T* result)
{
    //cout << "func, line = " << __func__ << "  " << __LINE__ << endl;
    result[0] = 1;
    result[1] = 0;
    result[2] = 0;
    result[3] = 0;
    result[4] = cos(angle);
    result[5] = -sin(angle);
    result[6] = 0;
    result[7] = sin(angle);
    result[8] = cos(angle);
}
template<typename T>
inline void Algorithm::rotateY(const T angle, T* result)
{
    //cout << "func, line = " << __func__ << "  " << __LINE__ << endl;
    result[0] = cos(angle);
    result[1] = 0;
    result[2] = sin(angle);
    result[3] = 0;
    result[4] = 1;
    result[5] = 0;
    result[6] = -sin(angle);
    result[7] = 0;
    result[8] = cos(angle);
}
template<typename T>
inline void Algorithm::rotateZ(const T angle, T* result)
{
    result[0] = cos(angle);
    result[1] = -sin(angle);
    result[2] = 0;
    result[3] = sin(angle);
    result[4] = cos(angle);
    result[5] = 0;
    result[6] = 0;
    result[7] = 0;
    result[8] = 1;
}

template<typename T>
inline void Algorithm::transX2Homo(const T x, T* pResult)
{
    //trans2Homo(x, 0, 0, pResult);
    pResult[0] = 1;
    pResult[1] = 0;
    pResult[2] = 0;
    pResult[3] = x;

    pResult[4] = 0;
    pResult[5] = 1;
    pResult[6] = 0;
    pResult[7] = 0;

    pResult[8]  = 0;
    pResult[9]  = 0;
    pResult[10] = 1;
    pResult[11] = 0;

    pResult[12] = 0;
    pResult[13] = 0;
    pResult[14] = 0;
    pResult[15] = 1;
}

/*****************************************************************************
 Prototype    : Algorithm.QuatMul
 Description  : Calculate the product of two quaternions.
 Input        : const T* pQuatFirst : The quaternion is the multiplicand
                const T* pQuatSecond: The quaternion is the multiplier

 Output       : T* pQuatResult      : The product
 Return Value : inline
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/12/08
    Author       : lin.lin
    Modification : Created function
  note: 1,fyi: CQuat; 2, see doc quatmultiply in matlab
*****************************************************************************/
template<typename T>
inline void Algorithm::QuatMul(const T* pQuatFirst, const T* pQuatSecond, T* pQuatResult)
{
    pQuatResult[0] = pQuatFirst[0] * pQuatSecond[0] - pQuatFirst[1] * pQuatSecond[1]
                   - pQuatFirst[2] * pQuatSecond[2] - pQuatFirst[3] * pQuatSecond[3];

    pQuatResult[1] = pQuatFirst[0] * pQuatSecond[1] + pQuatFirst[1] * pQuatSecond[0]
                   + pQuatFirst[2] * pQuatSecond[3] - pQuatFirst[3] * pQuatSecond[2];

    pQuatResult[2] = pQuatFirst[0] * pQuatSecond[2] + pQuatFirst[2] * pQuatSecond[0]
                   + pQuatFirst[3] * pQuatSecond[1] - pQuatFirst[1] * pQuatSecond[3];

    pQuatResult[3] = pQuatFirst[0] * pQuatSecond[3] + pQuatFirst[3] * pQuatSecond[0]
                   + pQuatFirst[1] * pQuatSecond[2] - pQuatFirst[2] * pQuatSecond[1];
}
/*****************************************************************************
 Prototype    : Algorithm.QuatMul
 Description  : Calculate the product of a 1-by-4 quaternion with itself.
 Input        : const T* pQuat
 Output       : T* pQuatResult      : The product
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/12/4
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
void Algorithm::QuatMul(const T* pQuat, T* pQuatResult)
{
    Algorithm::QuatMul(pQuat, pQuat, pQuatResult);
}

template<typename T>
inline void Algorithm::Conjugate(const T* qSrc, T* qDst)
{
    qDst[0] =  qSrc[0];
    qDst[1] = -qSrc[1];
    qDst[2] = -qSrc[2];
    qDst[3] = -qSrc[3];
}
/*****************************************************************************
 Prototype    : CQuat.rotation2Quat
 Description  : Create quaternion from matrix
 Input        : m: Rotation matrix, should be a member of SO(3).
                q: quaternion, array of 4.
 Output       : None
 Return Value : CQuat
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/16
    Author       : lin.lin
    Modification : Created function
 note: 1, All singularities are handled, provided m belongs to SO(3).
       2, see: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
       3, see rotm2quat.m in Matlab.

       4, run time: about 1 us, 20180103 version
       corresponding matlab cmd: rotm2quat
*****************************************************************************/
template<typename T>
inline void Algorithm::rotation2Quat(const T* m, T* q)
{
    const T tr = m[0] + m[4] + m[8];                        //  trace
    T s = 0;
    if (tr > 0)
    {
      s = 2 * sqrt(1.0 + tr);                               // S=4*qw
      q[0] = s * 0.25;
      q[1] = (m[7] - m[5]) / s;
      q[2] = (m[2] - m[6]) / s;
      q[3] = (m[3] - m[1]) / s;
    } else if (m[0] > m[4] && m[0] > m[8])
    {
      s = 2 * sqrt(1.0 + m[0] - m[4] - m[8]);               // S=4*qx
      q[0] = (m[7] - m[5]) / s;
      q[1] = s * 0.25;
      q[2] = (m[1] + m[3]) / s;
      q[3] = (m[2] + m[6]) / s;
    } else if (m[4] > m[8])
    {
      s = 2 * sqrt(1.0 + m[4] - m[0] - m[8]);               // S=4*qy
      q[0] = (m[2] - m[6]) / s;
      q[1] = (m[1] + m[3]) / s;
      q[2] = s * 0.25;
      q[3] = (m[5] + m[7]) / s;
    } else
    {
      s = 2 * sqrt(1.0 + m[8] - m[0] - m[4]);               // S=4*qz
      q[0] = (m[3] - m[1]) / s;
      q[1] = (m[2] + m[6]) / s;
      q[2] = (m[5] + m[7]) / s;
      q[3] = s * 0.25;
    }
}

/*****************************************************************************
 Prototype    : Algorithm.QuatDiff
 Description  : get the difference between two quaternions
 Input        : qSrc1: First quaternion
                qSrc2: Second quaternion
 Output       :
 Return Value : the difference between two quaternions
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/11/29
    Author       : lin.lin
    Modification : Created function
  note: recommended!
note: run time: about 2 us, 20180103 version
*****************************************************************************/
template<typename T>
T Algorithm::QuatDiff(const T* qSrc1, const T* qSrc2)
{
    T q2inv[4]   = {0};
    T q2invq1[4] = {0};
    T norm_inv;                                             // get inverse
    T dNorm1Sqr, dNorm2Sqr, dNorm2;                         // restore normalized value
    T darrQuatSrc1[4], darrQuatSrc2[4];
    T delt[4] = {1,0,0,0};

    dNorm1Sqr = SQRT(qSrc1[0] * qSrc1[0] + qSrc1[1] * qSrc1[1] + qSrc1[2] * qSrc1[2] + qSrc1[3] * qSrc1[3]);
    dNorm2    = qSrc2[0] * qSrc2[0] + qSrc2[1] * qSrc2[1] + qSrc2[2] * qSrc2[2] + qSrc2[3] * qSrc2[3];
    dNorm2Sqr = SQRT(dNorm2);

    darrQuatSrc1[0] = qSrc1[0] / dNorm1Sqr;
    darrQuatSrc1[1] = qSrc1[1] / dNorm1Sqr;
    darrQuatSrc1[2] = qSrc1[2] / dNorm1Sqr;
    darrQuatSrc1[3] = qSrc1[3] / dNorm1Sqr;
    darrQuatSrc2[0] = qSrc2[0] / dNorm2Sqr;
    darrQuatSrc2[1] = qSrc2[1] / dNorm2Sqr;
    darrQuatSrc2[2] = qSrc2[2] / dNorm2Sqr;
    darrQuatSrc2[3] = qSrc2[3] / dNorm2Sqr;
    norm_inv = 1.0 / dNorm2;
    q2inv[0] =  norm_inv * darrQuatSrc2[0];
    q2inv[1] = -norm_inv * darrQuatSrc2[1];
    q2inv[2] = -norm_inv * darrQuatSrc2[2];
    q2inv[3] = -norm_inv * darrQuatSrc2[3];

    QuatMul(q2inv, darrQuatSrc1, q2invq1);                  ///inv(q2) * q1

    delt[0] = q2invq1[0] - delt[0];
    delt[1] = q2invq1[1] - delt[1];
    delt[2] = q2invq1[2] - delt[2];
    delt[3] = q2invq1[3] - delt[3];                         ///inv(q2) * q1 - (1,0,0,0)
    // get variance
    return (delt[0]*delt[0] + delt[1]*delt[1] + delt[2]*delt[2] + delt[3]*delt[3]);
    //return variance;
}
/*****************************************************************************
 Prototype    : Algorithm.getErrO
 Description  : Get Otientation Error from Unit quaternion form.
 Input        : const T* tarrQd: desired unit quaternion (4*1)
                const T* tarrQe: actual unit quaternion  (4*1)
 Output       : T* ptErrO      : Otientation Error (3*1)
 Return Value : T
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/12/4
    Author       : lin.lin
    Modification : Created function
  note:
  1, fyi: 3.7.3 Orientation Error, Robotics Modelling, Planning and Control
  2, tarrQd, tarrQe are unit quaternions.
*****************************************************************************/
template<typename T>
void Algorithm::getErrO(const T* tarrQd, const T* tarrQe, T* ptErrO)
{
    T tarrQeInv[4]   = {0};
//    T tarrQdQeInv[4] = {0};
    // get tarrQeInv (see doc quatinv in matlab)
    T tNormQe    = 1.0;
    T tNormQeInv = 1.0 / tNormQe;
    tarrQeInv[0] =  tNormQeInv * tarrQe[0];
    tarrQeInv[1] = -tNormQeInv * tarrQe[1];
    tarrQeInv[2] = -tNormQeInv * tarrQe[2];
    tarrQeInv[3] = -tNormQeInv * tarrQe[3];

    // get ErrO
//    QuatMul(tarrQd, tarrQeInv, tarrQdQeInv);
//    ptErrO[0] =  tarrQdQeInv[1];
//    ptErrO[1] =  tarrQdQeInv[2];
//    ptErrO[2] =  tarrQdQeInv[3];
    ptErrO[0]      = tarrQd[0] * tarrQeInv[1] + tarrQd[1] * tarrQeInv[0]
                   + tarrQd[2] * tarrQeInv[3] - tarrQd[3] * tarrQeInv[2];

    ptErrO[1]      = tarrQd[0] * tarrQeInv[2] + tarrQd[2] * tarrQeInv[0]
                   + tarrQd[3] * tarrQeInv[1] - tarrQd[1] * tarrQeInv[3];

    ptErrO[2]      = tarrQd[0] * tarrQeInv[3] + tarrQd[3] * tarrQeInv[0]
                   + tarrQd[1] * tarrQeInv[2] - tarrQd[2] * tarrQeInv[1];  // (3.91)

}

/*****************************************************************************
 Prototype    : Algorithm.getErrO
 Description  : Get Otientation Error from Unit quaternion form.
 Input        : const T* tarrQd: desired unit quaternion (4*1)
                const T* tarrQe: actual unit quaternion  (4*1)
 Output       :

 Return Value : T: the error
 Calls        :
 Called By    :

  History        :
  1.Date         : 2019/8/1
    Author       : lin.lin
    Modification : Created function
note:
1, overload void Algorithm::getErrO(const T* tarrQd, const T* tarrQe, T* ptErrO)
*****************************************************************************/
template<typename T>
T Algorithm::getErrO(const T* tarrQd, const T* tarrQe)
{
    T tarrQeInv[4]   = {0};
    T tarrErrO[3] = { 0 };
    // get tarrQeInv (see doc quatinv in matlab)
    T tNormQe    = 1.0;
    T tNormQeInv = 1.0 / tNormQe;
    tarrQeInv[0] =  tNormQeInv * tarrQe[0];
    tarrQeInv[1] = -tNormQeInv * tarrQe[1];
    tarrQeInv[2] = -tNormQeInv * tarrQe[2];
    tarrQeInv[3] = -tNormQeInv * tarrQe[3];

    // get ErrO
    tarrErrO[0]    = tarrQd[0] * tarrQeInv[1] + tarrQd[1] * tarrQeInv[0]
                   + tarrQd[2] * tarrQeInv[3] - tarrQd[3] * tarrQeInv[2];

    tarrErrO[1]    = tarrQd[0] * tarrQeInv[2] + tarrQd[2] * tarrQeInv[0]
                   + tarrQd[3] * tarrQeInv[1] - tarrQd[1] * tarrQeInv[3];

    tarrErrO[2]    = tarrQd[0] * tarrQeInv[3] + tarrQd[3] * tarrQeInv[0]
                   + tarrQd[1] * tarrQeInv[2] - tarrQd[2] * tarrQeInv[1];  // (3.91)

    return (tarrErrO[0]* tarrErrO[0] + tarrErrO[1]* tarrErrO[1] + tarrErrO[2]* tarrErrO[2]);
}


/*****************************************************************************
 Prototype    : Algorithm.PYToMotors
 Description  : transform clip's gesture to each 3 motor's position (degree).
 Input        : yaw               : the angle of yaw (degree)
                pitch             : the angle of pitch (degree)
                clip              : the angle between 2 clips, always >= 0 (degree)
 Output       :
                motor1: motor 1's position (degree)
                motor2: motor 2's position (degree)
                motor3: motor 3's position (degree)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/4/24
    Author       : lin.lin
    Modification : Created function

note: I use cftool in matlab to fit the test data "PYToMotors_TestResult.csv".
      I choose Polynomial  f(x,y) = p00 + p10*x + p01*y to fit. the fit report is:
Linear model Poly11:
     f(x,y) = p00 + p10*x + p01*y
Coefficients (with 95% confidence bounds):
       p00 =      0.5248  (-0.09315, 1.143)
       p10 =      0.6683  (0.6549, 0.6817)
       p01 =       1.114  (1.103, 1.125)

Goodness of fit:
  SSE: 1956
  R-square: 0.9972
  Adjusted R-square: 0.9972
  RMSE: 3.738
*****************************************************************************/
template<typename T>
void Algorithm::MARYLAND_PYToMotors(const T yaw, const T pitch,  T& clip,
    T& motor1, T& motor2, T& motor3)
{
    T p00 = 0.5248;
    T p10 = 0.6683;
    T p01 = 1.114;


    if ( fabs(yaw) + clip/2 > MARYLAND_PYTOMOTORS_CLIP_MAX )
    {
        clip = (MARYLAND_PYTOMOTORS_CLIP_MAX - fabs(yaw)) * 2;
    }
    // process critical value of clip
    if ( clip < MARYLAND_PYTOMOTORS_CLIP_MIN )
    {
        clip = MARYLAND_PYTOMOTORS_CLIP_MIN;
    }
    if ( clip < MARYLAND_PYTOMOTORS_CLIP_CRI && clip >= MARYLAND_PYTOMOTORS_CLIP_MIN)
    {
        clip = (MARYLAND_PYTOMOTORS_CLIP_CRI - MARYLAND_PYTOMOTORS_CLIP_MIN)/MARYLAND_PYTOMOTORS_CLIP_CRI * clip + MARYLAND_PYTOMOTORS_CLIP_MIN;
    }
    motor1 = pitch;
    motor2 = p00 + p10 * pitch + p01 * yaw + p01 * clip/2;
    motor3 = p00 + p10 * pitch + p01 * yaw - p01 * clip/2;
}
/*****************************************************************************
 Prototype    : Algorithm.MEGA_PYToMotors
 Description  : transform mega's clip's gesture to each 3 motor's position (degree).
 Input        : yaw               : the angle of yaw (degree)
                pitch             : the angle of pitch (degree)
                clip              : the angle between 2 clips, always >= 0 (degree)
 Output       :
                motor1: motor 1's position (degree)
                motor2: motor 2's position (degree)
                motor3: motor 3's position (degree)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/5/18
    Author       : lin.lin
    Modification : Created function
note:
motor1:
Linear model Poly11:
     f(x,y) = p00 + p10*x + p01*y
Coefficients (with 95% confidence bounds):
       p00 =     -0.3055  (-0.3935, -0.2175)
       p10 =      0.9915  (0.9898, 0.9931)
       p01 =   -0.000357  (-0.001668, 0.0009541)

Goodness of fit:
  SSE: 74.55
  R-square: 0.9999
  Adjusted R-square: 0.9999
  RMSE: 0.6231

motor2:
Linear model Poly11:
     f(x,y) = p00 + p10*x + p01*y
Coefficients (with 95% confidence bounds):
       p00 =      -2.203  (-2.604, -1.803)
       p10 =      0.6312  (0.6239, 0.6384)
       p01 =        1.33  (1.324, 1.336)

Goodness of fit:
  SSE: 1542
  R-square: 0.9991
  Adjusted R-square: 0.9991
  RMSE: 2.834

motor3:
Linear model Poly11:
     f(x,y) = p00 + p10*x + p01*y
Coefficients (with 95% confidence bounds):
       p00 =      -1.469  (-1.843, -1.095)
       p10 =      0.6346  (0.6278, 0.6414)
       p01 =       1.325  (1.32, 1.331)

Goodness of fit:
  SSE: 1349
  R-square: 0.9992
  Adjusted R-square: 0.9992
  RMSE: 2.651


  x - pitch; y - yaw; z - motor1 ~ motor3
*****************************************************************************/
template<typename T>
void Algorithm::MEGA_PYToMotors(const T yaw, const T pitch,  T& clip,
    T& motor1, T& motor2, T& motor3)
{
    T p00[3] = {-0.3055,   -2.203,  -1.469 };
    T p10[3] = { 0.9915,    0.6312,  0.6346};
    T p01[3] = {-0.000357,  1.33,    1.325 };

    if ( fabs(yaw) + clip/2 > MEGA_PYTOMOTORS_CLIP_MAX )
    {
        clip = (MEGA_PYTOMOTORS_CLIP_MAX - fabs(yaw)) * 2;
    }
    // process critical value of clip
    if ( clip < MEGA_PYTOMOTORS_CLIP_MIN )
    {
        clip = MEGA_PYTOMOTORS_CLIP_MIN;
    }
    if ( clip < MEGA_PYTOMOTORS_CLIP_CRI && clip >= MEGA_PYTOMOTORS_CLIP_MIN)
    {
        clip = (MEGA_PYTOMOTORS_CLIP_CRI - MEGA_PYTOMOTORS_CLIP_MIN)/MEGA_PYTOMOTORS_CLIP_CRI * clip + MEGA_PYTOMOTORS_CLIP_MIN;
    }
    motor1 = p00[0] + p10[0] * pitch + p01[0] * yaw;
    motor2 = p00[1] + p10[1] * pitch + p01[1] * (yaw + 0.5 * clip);
    motor3 = p00[2] + p10[2] * pitch + p01[2] * (yaw - 0.5 * clip);
}

/* BEGIN: Added by lin.lin, 2017/7/13 */
/*****************************************************************************
 Prototype    : Algorithm.MONO_PYToMotors
 Description  : transform mono's clip's gesture to each 3 motor's position (degree).
 Input        : yaw               : the angle of yaw (degree)
                pitch             : the angle of pitch (degree)
                clip              : the angle between 2 clips, always >= 0 (degree)
 Output       :
                motor1: motor 1's position (degree)
                motor2: motor 2's position (degree)
                motor3: motor 3's position (degree)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/7/13
    Author       : lin.lin
    Modification : Created function
note:
motor1:
Linear model Poly11:
     f(x,y) = p00 + p10*x + p01*y
Coefficients (with 95% confidence bounds):
       p00 =      0.6725  (0.5199, 0.8251)
       p10 =      0.9959  (0.9931, 0.9986)
       p01 =   0.0004068  (-0.001984, 0.002797)

Goodness of fit:
  SSE: 224.1
  R-square: 0.9996
  Adjusted R-square: 0.9996
  RMSE: 1.08

motor2:
Linear model Poly11:
     f(x,y) = p00 + p10*x + p01*y
Coefficients (with 95% confidence bounds):
       p00 =      0.2868  (-0.2466, 0.8203)
       p10 =      0.6102  (0.6005, 0.6199)
       p01 =      0.8288  (0.8205, 0.8372)

Goodness of fit:
  SSE: 2739
  R-square: 0.9964
  Adjusted R-square: 0.9964
  RMSE: 3.777

motor3:
Linear model Poly11:
     f(x,y) = p00 + p10*x + p01*y
Coefficients (with 95% confidence bounds):
       p00 =     -0.1625  (-0.6679, 0.3429)
       p10 =      0.6136  (0.6044, 0.6228)
       p01 =      0.8271  (0.8192, 0.835)

Goodness of fit:
  SSE: 2458
  R-square: 0.9968
  Adjusted R-square: 0.9968
  RMSE: 3.578

  x - pitch; y - yaw; z - motor1 ~ motor3
note: the  wire lines are too loose; so it can be waited new MONO to test the motors' data again. 2017-8-3
*****************************************************************************/
template<typename T>
void Algorithm::MONO_PYToMotors(const T yaw, const T pitch,  T& clip,
    T& motor1, T& motor2, T& motor3)
{

    T p00[3] = {0.6725,    0.2868,  -0.1625 };
    T p10[3] = {0.9959,    0.6102,   0.6136 };
    T p01[3] = {0.0004068, 0.8288,   0.8271 };

    // protect clip not to too big
    if ( clip > MONO_PYTOMOTORS_CLIP_MAX )
    {
        clip = MONO_PYTOMOTORS_CLIP_MAX;
    }
    // protect yaw not to too big
    if ( fabs(yaw) + clip/2 > MONO_PYTOMOTORS_YAW_MAX )
    {
        clip = (MONO_PYTOMOTORS_YAW_MAX - fabs(yaw)) * 2;
    }
    // process critical value of clip
    if ( clip < MONO_PYTOMOTORS_CLIP_MIN )
    {
        clip = MONO_PYTOMOTORS_CLIP_MIN;
    }
//    if ( clip < MONO_PYTOMOTORS_CLIP_CRI && clip >= MONO_PYTOMOTORS_CLIP_MIN)
//    {
//        clip = (MONO_PYTOMOTORS_CLIP_CRI - MONO_PYTOMOTORS_CLIP_MIN)/MONO_PYTOMOTORS_CLIP_CRI * clip + MONO_PYTOMOTORS_CLIP_MIN;
//    }

    T dOffset1 = 0.0;
    T dOffset2 = 1.0;                              //compensating parameters.
    if ( MONO_PYTOMOTORS_CLIP_MIN <= clip && clip < MONO_PYTOMOTORS_CLIP_MIN + 5)
    {
        dOffset1 = 0.0;
        dOffset2 = 1.0;
    }else if ( MONO_PYTOMOTORS_CLIP_MIN +5 <= clip && clip <= MONO_PYTOMOTORS_CLIP_MIN + 10)
    {

        dOffset1 = 15.0 / 2;
        dOffset2 = 0.9 * 2.5 * (5.0 / clip);


    }else if ( MONO_PYTOMOTORS_CLIP_MIN + 10 < clip && clip <= MONO_PYTOMOTORS_CLIP_MAX)
    {
        dOffset1 = 15.0 * clip / 30;
        dOffset2 = 0.9;
    }
    motor1 = p00[0] + p10[0] * pitch + p01[0] * yaw;
    motor2 = p00[1] + p10[1] * pitch + p01[1] * ((yaw - dOffset1) + dOffset2 * clip);
    motor3 = p00[2] + p10[2] * pitch + p01[2] * ((yaw + dOffset1) - dOffset2 * clip);
}
/* END:   Added by lin.lin, 2017/7/13   PN: */
/*****************************************************************************
 Prototype    : Algorithm.Large_PYToMotors
 Description  : transform Large's clip's gesture to each 3 motor's position (degree).
 Input        : yaw               : the angle of yaw (degree)
                pitch             : the angle of pitch (degree)
                clip              : the angle between 2 clips, always >= 0 (degree)
 Output       :
                motor1: motor 1's position (degree)
                motor2: motor 2's position (degree)
                motor3: motor 3's position (degree)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/14
    Author       : lin.lin
    Modification : Created function
note:
motor1:
Linear model Poly11:
     f(x,y) = p00 + p10*x + p01*y
Coefficients (with 95% confidence bounds):
       p00 =    0.007786  (-0.03467, 0.05024)
       p10 =       1.003  (1.002, 1.004)
       p01 =  -0.0002872  (-0.0008603, 0.0002858)

Goodness of fit:
  SSE: 1.703
  R-square: 1
  Adjusted R-square: 1
  RMSE: 0.1685

motor2:
Linear model Poly11:
     f(x,y) = p00 + p10*x + p01*y
Coefficients (with 95% confidence bounds):
       p00 =       2.058  (1.488, 2.628)
       p10 =      0.6346  (0.6244, 0.6449)
       p01 =      0.8271  (0.8194, 0.8348)

Goodness of fit:
  SSE: 307
  R-square: 0.999
  Adjusted R-square: 0.999
  RMSE: 2.262

motor3:
Linear model Poly11:
     f(x,y) = p00 + p10*x + p01*y
Coefficients (with 95% confidence bounds):
       p00 =       1.798  (1.29, 2.306)
       p10 =      0.6341  (0.625, 0.6432)
       p01 =      0.8221  (0.8153, 0.829)

Goodness of fit:
  SSE: 243.9
  R-square: 0.9992
  Adjusted R-square: 0.9992
  RMSE: 2.016
--------------
  x - pitch; y - yaw; z - motor1 ~ motor3
*****************************************************************************/
template<typename T>
void Algorithm::Large_PYToMotors(const T yaw, const T pitch,  T& clip,
    T& motor1, T& motor2, T& motor3)
{
    T p00[3] = {0.007786,    2.058,   1.798  };
    T p10[3] = {1.003,       0.6346,  0.6341 };
    T p01[3] = {-0.0002872,  0.8271,  0.8221 };

    if ( fabs(yaw) + clip/2 > LARGE_PYTOMOTORS_CLIP_MAX )
    {
        clip = (LARGE_PYTOMOTORS_CLIP_MAX - fabs(yaw)) * 2;
    }
    // process critical value of clip
    if ( clip < LARGE_PYTOMOTORS_CLIP_MIN )
    {
        clip = LARGE_PYTOMOTORS_CLIP_MIN;
    }
    if ( clip < LARGE_PYTOMOTORS_CLIP_CRI && clip >= LARGE_PYTOMOTORS_CLIP_MIN)
    {
        clip = (LARGE_PYTOMOTORS_CLIP_CRI - LARGE_PYTOMOTORS_CLIP_MIN)/LARGE_PYTOMOTORS_CLIP_CRI * clip + LARGE_PYTOMOTORS_CLIP_MIN;
    }
    motor1 = p00[0] + p10[0] * pitch + p01[0] * yaw;
    motor2 = p00[1] + p10[1] * pitch + p01[1] * (yaw + 0.5 * clip);
    motor3 = p00[2] + p10[2] * pitch + p01[2] * (yaw - 0.5 * clip);
}

/*****************************************************************************
 Prototype    : Algorithm.MovingAverageFilter
 Description  :
 Input        : const T tSrcData: data to be filter (T can be double, float, Vector)
                const int nWidth: window's width
                T* tarrValueBuf : the buffer with the size of nWidth, store the nWidth's tSrcData
 Output       : None
 Return Value : data filtered
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/12/26
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
T Algorithm::MovingAverageFilter(const T tSrcData, const int nWidth, T* tarrValueBuf)
{
    int i = 0;
    T sum;
    sum = 0.0;
    if  ( nWidth > 0 )
    {
        // new source number to push
        for ( i = 0 ; i < nWidth-1; ++i)
        {
            tarrValueBuf[i] = tarrValueBuf[i+1];
        }
        tarrValueBuf[nWidth-1] = tSrcData;

        for(i=0; i<nWidth; ++i)
        {
            sum += tarrValueBuf[i];
        }
        return (sum / nWidth);
    }else
    {
        sum = 0.0;
        return sum;
    }
}
/* BEGIN: Added by lin.lin, 2018/1/16 */
/*****************************************************************************
 Prototype    : GramSchmidtOrth
 Description  : Orthonormal matrix
 Input        :
 Output       : pDstHomoData: 4*4, input source homo matrix, output Orthonormal homo matrix;
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/1/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
void Algorithm::GramSchmidtOrth(T *pDstHomoData)
{
    // B1 = a1
    T dProdV1V0 = pDstHomoData[0] * pDstHomoData[1] + pDstHomoData[4] * pDstHomoData[5] + pDstHomoData[8] * pDstHomoData[9];    // <B1, a2>
    T dProdV2V0 = pDstHomoData[0] * pDstHomoData[2] + pDstHomoData[4] * pDstHomoData[6] + pDstHomoData[8] * pDstHomoData[10];   // <B1, a3>
    T dLenV0Sqr = pDstHomoData[0] * pDstHomoData[0] + pDstHomoData[4] * pDstHomoData[4] + pDstHomoData[8] * pDstHomoData[8];    // <B1, B1>
    T dSlope0   = dProdV1V0 / dLenV0Sqr;   // <B1, a2> / <B1, B1>
    T dSlope1   = dProdV2V0 / dLenV0Sqr;   // <B1, a3> / <B1, B1>

    // get B2
    pDstHomoData[1] = pDstHomoData[1] - dSlope0 * pDstHomoData[0] ;
    pDstHomoData[5] = pDstHomoData[5] - dSlope0 * pDstHomoData[4] ;
    pDstHomoData[9] = pDstHomoData[9] - dSlope0 * pDstHomoData[8] ;    // B2 = a2 - <B1, a2> / <B1, B1> * B1

    T dProdV1V2 = pDstHomoData[2] * pDstHomoData[1] + pDstHomoData[6] * pDstHomoData[5] + pDstHomoData[10] * pDstHomoData[9]; // <B2, a3>
    T dLenV1Sqr = pDstHomoData[1] * pDstHomoData[1] + pDstHomoData[5] * pDstHomoData[5] + pDstHomoData[9] * pDstHomoData[9]; // <B2, B2>
    T dSlope2   = dProdV1V2 / dLenV1Sqr; // <B2, a3> / <B2, B2>

    // get B3
    pDstHomoData[2]  = pDstHomoData[2]  - dSlope1 * pDstHomoData[0] - dSlope2 * pDstHomoData[1];
    pDstHomoData[6]  = pDstHomoData[6]  - dSlope1 * pDstHomoData[4] - dSlope2 * pDstHomoData[5];
    pDstHomoData[10] = pDstHomoData[10] - dSlope1 * pDstHomoData[8] - dSlope2 * pDstHomoData[9];   // B3 = a3 - <B1, a3> / <B1, B1> * B1 - <B2, a3> / <B2, B2> * B2

    T dLenV2Sqr = pDstHomoData[2] * pDstHomoData[2] + pDstHomoData[6] * pDstHomoData[6] + pDstHomoData[10] * pDstHomoData[10];

    dLenV0Sqr = sqrt(dLenV0Sqr);
    dLenV1Sqr = sqrt(dLenV1Sqr);
    dLenV2Sqr = sqrt(dLenV2Sqr);

    pDstHomoData[0]  = pDstHomoData[0] / dLenV0Sqr ;
    pDstHomoData[1]  = pDstHomoData[1] / dLenV1Sqr ;
    pDstHomoData[2]  = pDstHomoData[2] / dLenV2Sqr ;
    pDstHomoData[4]  = pDstHomoData[4] / dLenV0Sqr ;
    pDstHomoData[5]  = pDstHomoData[5] / dLenV1Sqr ;
    pDstHomoData[6]  = pDstHomoData[6] / dLenV2Sqr ;
    pDstHomoData[8]  = pDstHomoData[8] / dLenV0Sqr ;
    pDstHomoData[9]  = pDstHomoData[9] / dLenV1Sqr ;
    pDstHomoData[10] = pDstHomoData[10] / dLenV2Sqr;
}
/* END:   Added by lin.lin, 2018/1/16   PN: */

/*****************************************************************************
 Prototype    : InvSqrt
 Description  : Fast inverse square-root; Calculate 1/Sqrt(X) instead of sqrt
 Input        : float number: value to be calculate
 Output       : 1/Sqrt(X)
 Return Value : inline
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/11/29
    Author       : lin.lin
    Modification : Created function
  note: this algorithm is not too obvious fast!
*****************************************************************************/
inline float Algorithm::InvSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;                                    // get bits for floating VALUE
    i = 0x5f3759df - (i>>1);                                // gives initial guess y0
    y = *(float*)&i;                                        // convert bits BACK to float
    y = y * (1.5f - (halfx * y * y));                       // Newton step, repeating increases accuracy
    return y;
}
/*****************************************************************************
 Prototype    : Algorithm.IK_rmrc
 Description  :
 Input        : const T *pt_dxd: desire x' (dx)
                const T  *pt_qs: the joint angle measured by angle sensor (unit: rad)
                const T *tarr_K: gain vector
                const int nCase: case to different algorithm of rmrc
                const T *pt_xd : desire X
                const T  *tarrSampleTimeSec: sample time (unit: second)

 Output       :
                T *pt_qe       : The actual joint Angle (unit: rad)
                T  *pt_xe      : The actual position and gesture of end effector (unit: rad)
                T *pt_log      : store log's data
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/10
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
void Algorithm::IK_rmrc(
                        const T *pt_dxd, const T  *pt_qs,
                        const T *tarr_K, const int nCase,
                        const T *pt_xd,  const T  *tarrSampleTimeSec,
                              T *pt_qe,        T  *pt_xe,     T *pt_log)
{
    // log point initial

    // initial variables
    const int nJointNum = 7; // joint number
    const int nRdof     = 1; //
    const int nEndDof   = nJointNum - nRdof; //6; dimension of end effector
    static bool bInitFlag   = false;
    static T tarr_qe[nJointNum]  = {0,0,0,0,0,0,0};
    static T tarr_dqe[nJointNum] = {0,0,0,0,0,0,0};
    static T tarr_xe[nEndDof]    = {0,0,0,0,0,0};
    T tarr_dqeNew[nJointNum]     = {0,0,0,0,0,0,0};

    T tarrV[nEndDof]             = {0,0,0,0,0,0};
    T tarrErr[nEndDof]           = {0,0,0,0,0,0};

    T tarrErrSub[nEndDof]        = {0,0,0,0,0,0};
    T tarrKsub[nEndDof]          = {0,0,0,0,0,0};
    T tarrVsub[nEndDof]          = {0,0,0,0,0,0};

    T tErrxThreshold = IK_RMRC_ERR_POSITION_THRESHOLD; // position error theshold
    T tErroThreshold = IK_RMRC_ERR_ORIENTATION_THRESHOLD; // orien error theshold

    // get err
    T parOrigin[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    static T tarrEndHomo[16]  = {0};
    T tarrEndHomoTrans[16]    = {0};
    T tarrEulRad[3]           = {0,0,0};
    static T tarrEulRadOld[3] = {0,0,0};

    static T tarrDhParam[nJointNum*4] = {0};
    int narrJointType[nJointNum]  = {0};

    T tarrJ[nEndDof*nJointNum]         = {0.0}; // J
    T tarrJa[nEndDof*nJointNum]        = {0.0}; // Ja
    T tarrJaT[nJointNum*nEndDof]       = {0};   // Ja'
    T tarrJapseu[nJointNum*nEndDof]    = {0};   // pseudo Jacobian

    T tarrJapseusubJasub[nJointNum*nJointNum]   = {0}; // J_pseu * J_sub
    T tarrInvJaJaT[nEndDof*nEndDof]    = {0};   // inv(Ja*Ja')

    T tarrJsub[nEndDof*nJointNum]      = {0};
    T tarrJsubTrans[nJointNum*nEndDof]     = {0};

    T tarrLog[100][16]     = {0.0};
    T* ptJ                 = tarrJ;
    T* ptJa                = tarrJa;
    T* ptLog               = (T*)tarrLog;

    T tarrInvB[9]   = {0};
    T tarrB[nEndDof*nEndDof]    = {0};

    int i = 0, j = 0, k = 0;
    T norm_x = 0;
    T norm_o = 0;

    // initial value (case 3)
    T tarrIn[nJointNum*nJointNum] = {0.0};
    T tarr_dq0[nJointNum]         = {0.0};
    T k0 = 2.0;
//    T tarr_qim[nJointNum] = {-170, -120, -170, -120, -170, -120, -175};
//    T tarr_qiM[nJointNum] = { 170,  120,  170,  120,  170,  120,  175};
    T tarr_qim[nJointNum] = {0};
    T tarr_qiM[nJointNum] = {0};

    T DH_param[] = { // alpha, a, d, q
          0*PI_DIV_180,  0, 340.0/1000.0,  pt_qs[0],
        -90*PI_DIV_180,  0,   0.0/1000.0,  pt_qs[1],
         90*PI_DIV_180,  0, 400.0/1000.0,  pt_qs[2],
        -90*PI_DIV_180,  0,   0.0/1000.0,  pt_qs[3],

         90*PI_DIV_180,  0, 400.0/1000.0,  pt_qs[4],
        -90*PI_DIV_180,  0,   0.0/1000.0,  pt_qs[5],
         90*PI_DIV_180,  0, 126.0/1000.0,  pt_qs[6]
    };

    if ( false == bInitFlag )
    {
        for ( i = 0 ; i <nJointNum ; ++i )
        {
            //tarr_qe[i] = pt_qs[i];
        }
        FK_DH(parOrigin, DH_param, nJointNum, tarrEndHomo, DH_CONVENTION_CRAIG);
        tr2rpy(tarrEndHomo, tarrEulRadOld, TR2RPY_MATRIXTYPE_HOMO);
        tarr_xe[0] = tarrEndHomo[3];
        tarr_xe[1] = tarrEndHomo[7];
        tarr_xe[2] = tarrEndHomo[11];

        tarr_xe[3] = tarrEulRadOld[0];
        tarr_xe[4] = tarrEulRadOld[1];
        tarr_xe[5] = tarrEulRadOld[2];
        bInitFlag = true;
    }

    for ( i = 0 ; i <nJointNum ; ++i )
    {
        tarrDhParam[0 + i*4] = DH_param[1 + i*4];
        tarrDhParam[1 + i*4] = DH_param[0 + i*4];
        tarrDhParam[2 + i*4] = DH_param[2 + i*4];
        tarrDhParam[3 + i*4] = DH_param[3 + i*4];
        tarr_qim[i] = pt_qs[i] - 1*PI_DIV_180;
        tarr_qiM[i] = pt_qs[i] + 1*PI_DIV_180;
    }

    FK_DH(parOrigin, DH_param, nJointNum, tarrEndHomo, DH_CONVENTION_CRAIG);
    tr2rpy(tarrEndHomo, tarrEulRad, TR2RPY_MATRIXTYPE_HOMO);

    for ( i = 0 ; i <nEndDof ; ++i )
    {
        tarrErr[i] = pt_xd[i] - tarr_xe[i];
    }
    norm_x = Norm(tarrErr);
    norm_o = Norm(tarrErr+3);

    for ( i = 0 ; i <nJointNum ; ++i )
    {
        tarr_qe[i]         = pt_qs[i];
        DH_param[i*4+3]    = tarr_qe[i];
        tarrDhParam[i*4+3] = tarr_qe[i];
        narrJointType[i]  = JOINTTYPE_ROTATION;
    }

    while ((norm_x >= tErrxThreshold) || (norm_o >= tErroThreshold))
    {
        // get J
        jacob0(tarrDhParam, nJointNum, nEndDof, narrJointType, DH_CONVENTION_CRAIG, tarrJ, ptLog);
        rpy2jac(tarrEulRad[0], tarrEulRad[1], tarrEulRad[2], tarrInvB); // Jacobian from RPY angle rates to angular velocity
        GeometryUtils::MatrixInv(tarrInvB, 3);
        // construct B
        tarrB[0] = tarrB[7] = tarrB[14] = 1.0;
        for ( i = 0 ; i <3 ; ++i )
        {
            for ( j = 0 ; j <3 ; ++j )
            {
                tarrB[(i+3)*nEndDof + j+3] = tarrInvB[i*3+j];
            }
        }
        // get Ja
        GeometryUtils::MatrixMultiply(tarrB, tarrJ, nEndDof, nEndDof, nJointNum, tarrJa); // get Ja
        switch ( nCase )
        {
            case 1 :
                //#
                break;
            case 2 :
                //Jacobian pseudo-inverse
                GeometryUtils::MatrixTranspose(tarrJa, nEndDof, nJointNum, tarrJaT);                       // get JaT
                GeometryUtils::MatrixMultiply(tarrJa, tarrJaT, nEndDof, nJointNum, nEndDof, tarrInvJaJaT); // get JaJaT
                GeometryUtils::MatrixInv(tarrInvJaJaT, nEndDof);                                           // get InvJaJaT
                // get v
                for ( i = 0 ; i <nEndDof ; ++i )
                {
                    tarrV[i] = pt_dxd[i] + tarr_K[i] * tarrErr[i];
                }

                // get dqe by pinv J
                GeometryUtils::MatrixMultiply(tarrJaT, tarrInvJaJaT, nJointNum, nEndDof, nEndDof, tarrJapseu);
                GeometryUtils::MatrixMultiply(tarrJapseu, tarrV, nJointNum, nEndDof, 1, tarr_dqeNew);
                break;
            case 3 :
                // Show the capability of handling the degree of redundancy, considered the 'distance from mechanical joint limits',


                // the distance from mechanical joint limits; fyi (3.57), 126,
                tarr_dq0[0] = ((tarr_qim[0] - tarr_qe[0])/(tarr_qiM[0] - tarr_qim[0]) + 0.5) / nJointNum; // 1/n * ((qim - qi)/range + 1/2)
                tarr_dq0[1] = ((tarr_qim[1] - tarr_qe[1])/(tarr_qiM[1] - tarr_qim[1]) + 0.5) / nJointNum;
                tarr_dq0[2] = ((tarr_qim[2] - tarr_qe[2])/(tarr_qiM[2] - tarr_qim[2]) + 0.5) / nJointNum;

                tarr_dq0[3] = ((tarr_qim[3] - tarr_qe[3])/(tarr_qiM[3] - tarr_qim[3]) + 0.5) / nJointNum;
                tarr_dq0[4] = ((tarr_qim[4] - tarr_qe[4])/(tarr_qiM[4] - tarr_qim[4]) + 0.5) / nJointNum;
                tarr_dq0[5] = ((tarr_qim[5] - tarr_qe[5])/(tarr_qiM[5] - tarr_qim[5]) + 0.5) / nJointNum;
                tarr_dq0[6] = ((tarr_qim[6] - tarr_qe[6])/(tarr_qiM[6] - tarr_qim[6]) + 0.5) / nJointNum;
                for ( i = 0 ; i <nJointNum ; ++i )
                {
                    tarr_dq0[i] *= k0;
                }
                eye(tarrIn, nJointNum);
                for ( i = 0 ; i <nEndDof ; ++i )
                {
                    // get v
                    tarrErrSub[i] = tarrErr[i];                     // (nJointNum-nRdof)*1
                    tarrKsub[i]   = tarr_K[i];                      // (nJointNum-nRdof)*1
                    tarrVsub[i]   = tarrErrSub[i] * tarrKsub[i];    // (nJointNum-nRdof)*1
                    // get dqe by pinv J
                    for ( j = 0 ; j <nJointNum ; ++j )
                    {
                        tarrJsub[i*nJointNum+j] = tarrJa[i*nJointNum+j];
                    }
                }

                GeometryUtils::MatrixTranspose(tarrJsub, nEndDof, nJointNum, tarrJaT);                       // get JasubT
                GeometryUtils::MatrixMultiply(tarrJsub, tarrJaT, nEndDof, nJointNum, nEndDof, tarrInvJaJaT); // get JasubJasubT
                GeometryUtils::MatrixInv(tarrInvJaJaT, nEndDof);                                          // get InvJasubJasubT
                // get J_pseu
                GeometryUtils::MatrixMultiply(tarrJaT, tarrInvJaJaT, nJointNum, nEndDof, nEndDof, tarrJapseu);
                // get tarrJapseusubJasub  (J_pseu * J_sub)
                GeometryUtils::MatrixMultiply(tarrJapseu, tarrJsub, nJointNum, nEndDof, nJointNum, tarrJapseusubJasub);
                // get (In - J_pseu * J_sub)
                for ( i = 0 ; i <nJointNum ; ++i )
                {
                    for ( j = 0 ; j <nJointNum ; ++j )
                    {
                        tarrJapseusubJasub[i*nJointNum+j] = -tarrJapseusubJasub[i*nJointNum+j];
                        if (i == j)
                        {
                            tarrJapseusubJasub[i*nJointNum + j] += 1.0;
                        }
                    }
                }
                // get (In - J_pseu * J_sub) * dq0
                GeometryUtils::MatrixMultiply(tarrJapseusubJasub, tarr_dq0, nJointNum, nJointNum, 1, tarr_dqeNew);
                // get J_pseu * v_sub
                GeometryUtils::MatrixMultiply(tarrJapseu, tarrVsub, nJointNum, nEndDof, 1, tarr_dq0); // reuse dq0 to store: J_pseu * v_sub
                // get tarr_dqeNew
                for ( i = 0 ; i <nJointNum ; ++i )
                {
                    tarr_dqeNew[i] += tarr_dq0[i]; // thetad = J_pseu * v_sub + (In - J_pseu * J_sub) * dq0;
                }
                break;
            case 4 :
                // get v
                for ( i = 0 ; i <nEndDof ; ++i )
                {
                    tarrErrSub[i] = tarrErr[i];                         // (nJointNum-nRdof)*1
                    tarrKsub[i]   = tarr_K[i];                          // (nJointNum-nRdof)*1
                    tarrVsub[i]   = tarrErrSub[i] * tarrKsub[i];        // (nJointNum-nRdof)*1
                    for ( j = 0 ; j <nJointNum ; ++j )
                    {
                        tarrJsub[i*nJointNum+j] = ptJa[i*nJointNum+j];
                    }
                }
                // get dqe by JT
                GeometryUtils::MatrixTranspose(tarrJsub, nEndDof, nJointNum, tarrJsubTrans);
                GeometryUtils::MatrixMultiply(tarrJsubTrans, tarrVsub, nJointNum, nEndDof, 1, tarr_dqeNew); //thetad = J_sub' * v_sub;        // nJointNum*1 % (3.76): dq = JT(q) * K * e  p135
                break;

            default:
                ;
        }

        // integrator (Euler method)
        for ( i = 0 ; i <nJointNum ; ++i )
        {
            tarr_qe[i] = tarr_qe[i] + tarrSampleTimeSec[0] * (tarr_dqeNew[i] + tarr_dqe[i]) / 2.0; // radian
            if ( tarr_qe[i] > PI )
            {
                tarr_qe[i] -= 2*PI;
            }
            if ( tarr_qe[i] < -PI )
            {
                tarr_qe[i] += 2*PI;
            }
        }


        // get xe by fk process
        for ( i = 0 ; i <nJointNum ; ++i )
        {
            DH_param[i*4+3]    = tarr_qe[i];
            tarrDhParam[i*4+3] = tarr_qe[i];
        }

        FK_DH(parOrigin, DH_param, nJointNum, tarrEndHomo, DH_CONVENTION_CRAIG);
        tarr_xe[0] = tarrEndHomo[3];
        tarr_xe[1] = tarrEndHomo[7];
        tarr_xe[2] = tarrEndHomo[11];
        tr2rpy(tarrEndHomo, tarrEulRad, TR2RPY_MATRIXTYPE_HOMO);
        tarr_xe[3] = tarrEulRad[0];
        tarr_xe[4] = tarrEulRad[1];
        tarr_xe[5] = tarrEulRad[2];
        memcpy(tarrEulRadOld, tarrEulRad, sizeof(T)*3);


        // get new err
        for ( i = 0 ; i <nEndDof ; ++i )
        {
            tarrErr[i] = pt_xd[i] - tarr_xe[i];
        }
        norm_x = Norm(tarrErr);
        norm_o = Norm(tarrErr+3);

        // update dq
        for ( i = 0 ; i <nJointNum ; ++i )
        {
            tarr_dqe[i] = tarr_dqeNew[i];

            pt_qe[i]      = tarr_qe[i];
        }

    } // while

    // output
    for ( i = 0 ; i <nJointNum ; ++i )
    {
        pt_qe[i]      = tarr_qe[i];
    }
    for ( i = 0 ; i <nEndDof ; ++i )
    {
        pt_xe[i]    = tarr_xe[i];
    }
    return;
}

/*****************************************************************************
 Prototype    : Algorithm.IK_rmrc
 Description  : Ik process by RMRC
 Input        : const T *pt_xd            : desire X;
                const T *ptDotXdesire     : desire x' (dx)
                const T  *ptQsRad         : the joint angle measured by angle sensor (unit: rad)
                const T  tarrSampleTimeSec: sample time (unit: second)
                const T *tarr_K           : gain vector
                const int nCase           : case to different algorithm of rmrc
                T* DH_param[in, out]      : dh parameters. the 1~3 columns information are useful when called,
                                            and the 4th column (joint angles) will be reset to qs(rad) and finally output qe(rad) within this function.
                T *ptQim                  : lower angle bound of every joint (unit: rad)
                                            default: {-170, -120, -170, -120, -170, -120, -175}
                T *ptQiM                  : higher angle bound of every joint (unit: rad)
                                            default: { 170,  120,  170,  120,  170,  120,  175}
                const int iJointNum       : joint number;     default: 7
                const int iDhConvention   : 0 - std DH; 1 - Craig DH (modified DH)
                const int iErrOrientType  : type of orientation error; RMRC_ERRO_EULER (default), RMRC_ERRO_ANG_AX, RMRC_ERRO_QUAT
 Output       :
                T *ptQeRad                : The actual joint Angle (unit: rad)
                T *tarrXe                 : The actual position and gesture of end effector (unit: m)
                                            if use RMRC_ERRO_QUAT, return tarrXe with size of 7, and the last 4 numbers are orientation errors represent by quaternion.
                T *tarrErr                : positin error; pt_xd - tarrXe;
                T *pt_log                 : store log's data; default: NULL
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/12
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
void Algorithm::IK_rmrc(
                        const T *pt_xd,              const T *ptDotXdesire, const T  *ptQsRad,
                        const T  tarrSampleTimeSec,  const T *tarr_K,       const int nCase,
                        T* DH_param,
                        T *ptQeRad,                        T *tarrXe,              T *tarrErr,
                        const int iDhConvention,
                        T *ptQim,        T *ptQiM,   const int iJointNum,
                        const RMRC_ORIENT_ERR_TYPE iErrOrientType,
                        T *ptParam)
{
    // initial variables
    const int iRdof                 = 1;                    // redundant dof
    const int iJointMaxNum          = RMRC_MAX_JOINT_NUM;
    const int iMaxEndDof            = iJointMaxNum - iRdof;
    int nEndDof                     = iJointNum    - iRdof; // dimension of end effector (6)
    static T tarr_dqe[iJointMaxNum] = {0};
    T tarr_dqeNew[iJointMaxNum]     = {0};

    T tarrV[iMaxEndDof]             = {0};

    T tarrErrSub[iMaxEndDof]        = {0};
    T tarrKsub[iMaxEndDof]          = {0};
    T tarrVsub[iMaxEndDof]          = {0};

    T tErrxThreshold = IK_RMRC_ERR_POSITION_THRESHOLD;      // position error theshold
    T tErroThreshold = IK_RMRC_ERR_ORIENTATION_THRESHOLD;   // orien error theshold

    // get err
    T parOrigin[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    T tarrEndHomo[16]                                = {0};
    T tarrEndHomoTrans[16]                           = {0};
    T tarrEulRad[3]                                  = {0};

    static T tarrDhParam[iJointMaxNum*4]             = {0};
    int narrJointType[iJointMaxNum]                  = {0};

    T tarrJ[iMaxEndDof*iJointMaxNum]                 = {0.0};     // Jacobian: J (6*7)
    T tarrJaT[iJointMaxNum*iMaxEndDof]               = {0};       // transpose of analytical Jacobian: Ja'
    T tarrJapseu[iJointMaxNum*iMaxEndDof]            = {0};       // pseudo analytical Jacobian: Ja+

    T tarrJapseusubJasub[iJointMaxNum*iJointMaxNum]  = {0};       // J_pseu * J_sub
    T tarrInvJaJaT[iMaxEndDof*iMaxEndDof]            = {0};       // inv(Ja*Ja')
    T tarrInvJaJaTTemp[iMaxEndDof*iMaxEndDof]        = {0};
    T tarrJsub[iMaxEndDof*iJointMaxNum]              = {0};//(6*7)
    T tarrJsubTrans[iJointMaxNum*iMaxEndDof]         = {0};//(7*6)
    T tarrDiagW[iJointMaxNum]                        = {0};

    const T tMaxLamda                                = 0.05;      // data from hanlei.wang, p13
    const T tEpsilon                                 = tMaxLamda;
    T tarrLog[100][16]                               = {0.0};
    T* ptLog                                         = (T*)tarrLog;

    T tarrInvB[9]                                    = {0};
    T tarrB[iMaxEndDof*iMaxEndDof]                   = {0}; //(6*6)

    int i = 0, j = 0, k = 0;
    T norm_x = 0;
    T norm_o = 0;

    // initial value (case 3)
    T tarr_dq0[iJointMaxNum]         = {0.0};
    T k0 = 2.0;

    // assign the bound of joint angle
    T tarrJointQdegLowerBound[iJointMaxNum] = {-170, -120, -170, -120, -170, -120, -175};
    T tarrJointQdegUpperBound[iJointMaxNum] = { 170,  120,  170,  120,  170,  120,  175};

    if ( NULL == ptQim )
    {
        ptQim = tarrJointQdegLowerBound;
        for ( i = 0 ; i <iJointNum ; ++i )
        {
            ptQim[i] *= PI_DIV_180;
        }
    }
    if ( NULL == ptQiM )
    {
        ptQiM = tarrJointQdegUpperBound;
        for ( i = 0 ; i <iJointNum ; ++i )
        {
            ptQiM[i] *= PI_DIV_180;
        }
    }

    for ( i = 0 ; i <iJointNum ; ++i )
    {
        tarrDhParam[0 + i*4] = DH_param[1 + i*4];
        tarrDhParam[1 + i*4] = DH_param[0 + i*4];
        tarrDhParam[2 + i*4] = DH_param[2 + i*4];
        tarrDhParam[3 + i*4] = DH_param[3 + i*4];
    }

    // get tarrEulRad
    for ( i = 0 ; i <iJointNum ; ++i )
    {
        DH_param[i*4+3]    = ptQsRad[i];
        tarrDhParam[i*4+3] = ptQsRad[i];
        ptQeRad[i]         = ptQsRad[i];
        narrJointType[i]   = JOINTTYPE_ROTATION;
    }
    if ( NULL == ptParam )
    {
        for ( i = 0 ; i <iJointNum ; ++i )
        {
            tarrDiagW[i] = 1.0;
        }

    }else
    {
        for ( i = 0 ; i <iJointNum ; ++i )
        {
            tarrDiagW[i] = ptParam[i];
        }
    }

    T tarrEndRot[9] = {0};

    // get J (6*7)
    jacob0(tarrDhParam, iJointNum, nEndDof, narrJointType, iDhConvention, tarrJ, ptLog);
    // get Xe, err
    FK_DH(parOrigin,    DH_param,   iJointNum, tarrEndHomo, iDhConvention);
    tarrXe[0] = tarrEndHomo[3];
    tarrXe[1] = tarrEndHomo[7];
    tarrXe[2] = tarrEndHomo[11];
    for ( i = 0 ; i <nEndDof-3 ; ++i )
    {
        tarrErr[i] = pt_xd[i] - tarrXe[i];
    }
    if ( RMRC_ORIENT_ERR_TYPE::RMRC_ERRO_EULER ==  iErrOrientType) // use Ja
    {
        tr2rpy(tarrEndHomo, tarrEulRad, TR2RPY_MATRIXTYPE_HOMO);
        tarrXe[3] = tarrEulRad[0];
        tarrXe[4] = tarrEulRad[1];
        tarrXe[5] = tarrEulRad[2];
        // get erro
        tarrErr[3] = pt_xd[3] - tarrXe[3];
        tarrErr[4] = pt_xd[4] - tarrXe[4];
        tarrErr[5] = pt_xd[5] - tarrXe[5];

        // --------------get Ja------------------------
        // get B (6*6)
        rpy2jac(tarrEulRad[0], tarrEulRad[1], tarrEulRad[2], tarrInvB); // Jacobian from RPY angle rates to angular velocity
        GeometryUtils::MatrixInv(tarrInvB, 3);
        tarrB[0] = tarrB[7] = tarrB[14] = 1.0;
        for ( i = 0 ; i <3 ; ++i )
        {
            for ( j = 0 ; j <3 ; ++j )
            {
                tarrB[(i+3)*nEndDof + j+3] = tarrInvB[i*3+j];
            }
        }
        // get Ja (6*7)
        GeometryUtils::MatrixMultiply(tarrB, tarrJ, nEndDof, nEndDof, iJointNum, tarrJsub); // get Ja
    }else // use J
    {
        memcpy(tarrJsub, tarrJ, sizeof(T)*nEndDof*iJointNum);  // get J
        Algorithm::t2r(tarrEndHomo, tarrEndRot);
        Algorithm::rotation2Quat(tarrEndRot, tarrXe+3);        // get quaternion from tarrXe+3 ~ tarrXe+6
        // get erro
        Algorithm::getErrO(pt_xd+3, tarrXe+3, tarrErr+3); // get last 3 numbers of err
    }
    switch ( nCase )
    {
        case 1 :
            //#
            break;
        case 2 :
            //Jacobian pseudo-inverse
            GeometryUtils::MatrixTranspose(tarrJsub, nEndDof, iJointNum, tarrJaT);                                        // get JaT
            GeometryUtils::MatrixMultiply(tarrJsub, tarrJaT, nEndDof, iJointNum, nEndDof, tarrInvJaJaT);                  // get Ja*JaT
            GeometryUtils::MatrixInv(tarrInvJaJaT, nEndDof);                                                            // get Inv(Ja*JaT)
            // get v
            for ( i = 0 ; i <nEndDof ; ++i )
            {
                tarrV[i] = ptDotXdesire[i] + tarr_K[i] * tarrErr[i];            // (dxd +Ke)
            }

            // get dqe by pinv J
            GeometryUtils::MatrixMultiply(tarrJaT, tarrInvJaJaT, iJointNum, nEndDof, nEndDof, tarrJapseu);              // get Ja+ = JaT * Inv(Ja*JaT)
            GeometryUtils::MatrixMultiply(tarrJapseu, tarrV, iJointNum, nEndDof, 1, tarr_dqeNew);                       // get Ja+ * (dxd + Ke) (3.72)
            break;
        case 3 :
            // Show the capability of handling the degree of redundancy, considered the 'distance from mechanical joint limits',
            // the distance from mechanical joint limits; fyi (3.57), 126,
            tarr_dq0[0] = ((ptQim[0] - ptQeRad[0]) / (ptQiM[0] - ptQim[0]) + 0.5)  /  iJointNum;                        // 1/n * ((qim - qi)/range + 1/2)
            tarr_dq0[1] = ((ptQim[1] - ptQeRad[1]) / (ptQiM[1] - ptQim[1]) + 0.5)  /  iJointNum;
            tarr_dq0[2] = ((ptQim[2] - ptQeRad[2]) / (ptQiM[2] - ptQim[2]) + 0.5)  /  iJointNum;

            tarr_dq0[3] = ((ptQim[3] - ptQeRad[3]) / (ptQiM[3] - ptQim[3]) + 0.5)  /  iJointNum;
            tarr_dq0[4] = ((ptQim[4] - ptQeRad[4]) / (ptQiM[4] - ptQim[4]) + 0.5)  /  iJointNum;
            tarr_dq0[5] = ((ptQim[5] - ptQeRad[5]) / (ptQiM[5] - ptQim[5]) + 0.5)  /  iJointNum;
            tarr_dq0[6] = ((ptQim[6] - ptQeRad[6]) / (ptQiM[6] - ptQim[6]) + 0.5)  /  iJointNum;
            for ( i = 0 ; i <iJointNum ; ++i )
            {
                tarr_dq0[i] *= k0;
            }
            for ( i = 0 ; i <nEndDof ; ++i )
            {
                // get v
                tarrErrSub[i] = tarrErr[i];
                tarrKsub[i]   = tarr_K[i];
                tarrVsub[i]   = ptDotXdesire[i] + tarrKsub[i] * tarrErrSub[i];      // v = dxd + K * e (3.70, 3.72)
            }

            // input tarrJsub (6*7)
            // output tarrJapseu (7*6)
            Algorithm::getWeightRightPinv(tarrJsub, nEndDof, iJointNum, tarrJapseu, tarrDiagW);
            // get tarrJapseusubJasub  (J+ * J)
            GeometryUtils::MatrixMultiply(tarrJapseu, tarrJsub, iJointNum, nEndDof, iJointNum, tarrJapseusubJasub);
            // get (In - J+ * J)
            for ( i = 0 ; i <iJointNum ; ++i )
            {
                for ( j = 0 ; j <iJointNum ; ++j )
                {
                    tarrJapseusubJasub[i*iJointNum+j] = -tarrJapseusubJasub[i*iJointNum+j];
                    if (i == j)
                    {
                        tarrJapseusubJasub[i*iJointNum + j] += 1.0;
                    }
                }
            }
            // get (In - J+ * J) * dq0
            GeometryUtils::MatrixMultiply(tarrJapseusubJasub, tarr_dq0, iJointNum, iJointNum, 1, tarr_dqeNew);
            // get J+ * (dxd +Ke)
            GeometryUtils::MatrixMultiply(tarrJapseu, tarrVsub, iJointNum, nEndDof, 1, tarr_dq0);                       // reuse dq0 to store: J_pseu * v_sub
            // get tarr_dqeNew
            for ( i = 0 ; i <iJointNum ; ++i )
            {
                tarr_dqeNew[i] += tarr_dq0[i]; // thetad = J+ * (dxd +Ke) + (In - J+ * J) * dq0; (3.54) (3.72)
            }
            break;

        case 4 :
            // get v
            for ( i = 0 ; i <nEndDof ; ++i )
            {
                tarrErrSub[i] = tarrErr[i];                 // (nJointNum-nRdof)*1
                tarrKsub[i]   = tarr_K[i];                  // (nJointNum-nRdof)*1
                tarrVsub[i]   = tarrErrSub[i] * tarrKsub[i];// (nJointNum-nRdof)*1
            }
            // get dqe by JT
            GeometryUtils::MatrixTranspose(tarrJsub, nEndDof, iJointNum, tarrJsubTrans);
            GeometryUtils::MatrixMultiply(tarrJsubTrans, tarrVsub, iJointNum, nEndDof, 1, tarr_dqeNew); //thetad = J_sub' * v_sub;        // nJointNum*1 % (3.76): dq = JT(q) * K * e  p135
            break;

        default:
            ;
    }

    // integrator (Euler method)
    for ( i = 0 ; i <iJointNum ; ++i )
    {
        ptQeRad[i] += tarrSampleTimeSec * (tarr_dqeNew[i] + tarr_dqe[i]) / 2.0; // radian
        while ( ptQeRad[i] > PI )
        {
            ptQeRad[i] -= 2*PI;
        }
        while ( ptQeRad[i] < -PI )
        {
            ptQeRad[i] += 2*PI;
        }
    }

    // get xe by fk process
    for ( i = 0 ; i <iJointNum ; ++i )
    {
        DH_param[i*4+3]    = ptQeRad[i];
        tarrDhParam[i*4+3] = ptQeRad[i];
        tarr_dqe[i] = tarr_dqeNew[i];                       // update tarr_dqe
    }
    FK_DH(parOrigin, DH_param, iJointNum, tarrEndHomo, iDhConvention);

    // output tarrXe
    tarrXe[0] = tarrEndHomo[3];
    tarrXe[1] = tarrEndHomo[7];
    tarrXe[2] = tarrEndHomo[11];
    if ( RMRC_ORIENT_ERR_TYPE::RMRC_ERRO_EULER ==  iErrOrientType) // use Ja
    {
        tr2rpy(tarrEndHomo, tarrEulRad, TR2RPY_MATRIXTYPE_HOMO);
        tarrXe[3] = tarrEulRad[0];
        tarrXe[4] = tarrEulRad[1];
        tarrXe[5] = tarrEulRad[2];
    }else
    {
        Algorithm::t2r(tarrEndHomo, tarrEndRot);
        Algorithm::rotation2Quat(tarrEndRot,    tarrXe+3); // get quaternion from tarrXe+3 ~ tarrXe+6
    }
    return;
}
/*****************************************************************************
 Prototype    : Algorithm.vex
 Description  : Convert skew-symmetric matrix to vector
 Input        : const T *ptS: point to the skew-symmetric matrix(3*3)
                                | 0 -vz vy|
                                | vz 0 -vx|
                                |-vy vx 0 |
 Output       : T* ptV      : point to the vector(3*1)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/29
    Author       : lin.lin
    Modification : Created function

  note:
  1, This is the inverse of the function skew (not implemented yet).
  2, No checking is done to ensure that the matrix is actually skew-symmetric.
  3, The function takes the mean of the two elements that correspond to each unique element of the matrix, ie. vx = 0.5*(s(3,2)-s(2,3))
*****************************************************************************/
template<typename T>
void Algorithm::vex(const T *ptS, T* ptV)
{
    ptV[0] = (ptS[7] - ptS[5]) * 0.5;
    ptV[1] = (ptS[2] - ptS[6]) * 0.5;
    ptV[2] = (ptS[3] - ptS[1]) * 0.5;
}
/*****************************************************************************
 Prototype    : Algorithm.skew
 Description  : Create skew-symmetric matrix
 Input        : const T* ptV  : point to the vector(iDim*1); iDim = 2, 3(default)
                const int iDim: the dimension of V; 3(default) - create so(3); 2 - so(2)
 Output       : T* ptS      : s = skew(v) is a skew-symmetric matrix formed from v (3 * 1).
                                 | 0 -vz vy|
                                 | vz 0 -vx|
                                 |-vy vx 0 |
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/12/4
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
void Algorithm::skew(const T* ptV, T* ptS, const int iDim)
{
    if ( 3 == iDim )
    {
        // SO(3) case
        ptS[0] =  0.0;     ptS[1] = -ptV[2];  ptS[2] =   ptV[1];
        ptS[3] =  ptV[2];  ptS[4] =  0.0;     ptS[5] =  -ptV[0];
        ptS[6] = -ptV[1];  ptS[7] =  ptV[0];  ptS[8] =   0.0;

    }else if(1 == iDim)
    {
        // SO(2) case
        skew(*ptV, ptS);
    }else
    {
        printf("error('argument must be a 1- or 3-vector')\n");
    }

}
/*****************************************************************************
 Prototype    : Algorithm.skew
 Description  : Create skew-symmetric matrix of SO(2)
 Input        : const T tq: the number (eg. rotation angle in planar) to input

 Output       : T* ptS    : s = skew(v) is a skew-symmetric matrix formed from tq.
                               | 0   -vz|
                               | vz   0 |
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/12/4
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
void Algorithm::skew(const T tq, T* ptS)
{
    ptS[0] = 0.0;     ptS[1] = -tq;
    ptS[2] = tq;      ptS[3] = 0.0;
}

/*****************************************************************************
 Prototype    : Algorithm.tr2delta
 Description  : Convert homogeneous transform to differential motions
 Input        : const T *ptT0: the infinitessimal motion from pose ptT0 to ptT1
                const T* ptT1:
 Output       :
                T* ptDelta   : the differential motion(6*1) corresponding to infinitessimal motion from pose T0 to T1
                               which are homogeneous transformations(4*4)
                               d=(dx, dy, dz, dRx, dRy, dRz) and is an approximation to the average spatial velocity multiplied by time.
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/29
    Author       : lin.lin
    Modification : Created function
  note: d is only an approximation to the motion T, and assumes that T0.=T1 or T.=eye(4,4)
*****************************************************************************/
template<typename T>
void Algorithm::tr2delta(const T *ptT0, const T* ptT1, T* ptDelta)
{
    T R0[9]  = {0};
    T R1[9]  = {0};
    T R0T[9] = {0};
    T Rtemp[9] = {0};
    Algorithm::t2r(ptT0, R0);
    Algorithm::t2r(ptT1, R1);
    GeometryUtils::MatrixTranspose(R0, 3, 3, R0T);
    GeometryUtils::MatrixMultiply(R1, R0T, 3, 3, 3, Rtemp);
    Rtemp[0] -= 1; Rtemp[4] -= 1; Rtemp[8] -= 1;   // R1*R0' - eye(3,3)

    ptDelta[0] = ptT1[3]  - ptT0[3] ;
    ptDelta[1] = ptT1[7]  - ptT0[7] ;
    ptDelta[2] = ptT1[11] - ptT0[11];

    vex(Rtemp, ptDelta+3);
}
/*****************************************************************************
 Prototype    : Algorithm.transl
 Description  : Create a translational transformation matrix
 Input        : const T *ptT0: a pure translation of x, y and z
 Output       : T* ptTdst    : an SE(3) homogeneous transform (4*4) representing a pure translation of x, y and z.
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/29
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
void Algorithm::transl(const T *ptT0, T* ptTdst)
{
    int i;
    for  ( i = 0 ; i <16 ; i++ )
    {
        ptTdst[i] = 0.0;
    }
    ptTdst[0]  = 1.0;
    ptTdst[5]  = 1.0;
    ptTdst[10] = 1.0;
    ptTdst[15] = 1.0;

    ptTdst[3]  = ptT0[0];
    ptTdst[7]  = ptT0[1];
    ptTdst[11] = ptT0[2];
}
/*****************************************************************************
 Prototype    : Algorithm.eul2rotm
 Description  :
 Input        : T* pEuler
                T* ptRotm
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/3
    Author       : lin.lin
    Modification : Created function
  note: Algorithm::rpy2Homo is equivalent to Algorithm::eul2rotm. Recommend the former.
*****************************************************************************/
template<typename T>
void Algorithm::eul2rotm(T* pEuler, T* ptRotm)
{
    T tarrResTemp[9] = {0};
    T tarrRz[9], tarrRy[9], tarrRx[9];

    rotateZ(pEuler[0],  tarrRz);
    rotateY(pEuler[1],  tarrRy);
    rotateX(pEuler[2],  tarrRx);

    matrix3x3Multiply(tarrRz, tarrRy, tarrResTemp);
    matrix3x3Multiply(tarrResTemp, tarrRx, ptRotm);
}
/*****************************************************************************
 Prototype    : Algorithm.normrnd
 Description  : Generate a normal distribution matrix
 Input        : T aver    : Mean
                T sigma   : standard deviation
                int row   : line number (default: 1)
                int column: column number (default: 1)
                T* p
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/3
    Author       : lin.lin
    Modification : Created function
  note: https://blog.csdn.net/luanpeng825485697/article/details/77870256
*****************************************************************************/
template<typename T>
void Algorithm::normrnd(T aver, T sigma, int row, int column, T* p)
{
    T x, dScope, y, fengzhi;
    //int RAND_MAX = 65535;
    int i, j;

    //p = (T *)malloc(row * column * sizeof(T));
    srand((unsigned)time(0));
    for (i=0; i<row; i++)
    {
        for (j=0; j<column; j++)
        {
            do
            {
                x = ((T)rand() / RAND_MAX)*6*sigma + aver - 3*sigma;  //
                y = 1.0 / (std::sqrt(2*PI)*sigma) * std::exp(-1*(x-aver)*(x-aver)/(2*sigma*sigma));
                fengzhi = 1.0 / (std::sqrt(2*PI) * sigma);
                dScope  = ((T)rand()/RAND_MAX) * fengzhi;
            }while(dScope > y);

            //p(i,j) = x;
            p[i*column + j] = x;
        }
    }
}

/*****************************************************************************
 Prototype    : Algorithm.jcbi
 Description  : the Jacobian algorithm for computing eigenvalues and eigenvectors of real symmetric matrices
 Input        : T* a   : store n order real symmetric matrix, n eigenvalues are stored on the diagonal when returned. (n*n)
                int n  : the order of matrix A.
                T eps  : Control accuracy requirement
                int jt : Control the maximum number of iterations
 Output       : T* v   : Return the eigenvectors. Where the j-th column is the eigenvector corresponding to the j-th eigenvalue. (n*n)

 Return Value : int    : Return flag value.
                         If the flag value returned is less than 0, the program failed.
                         If the flag value returned is greater than 0, the program returns normally.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/26
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
int Algorithm::jcbi(T* a, const int n, T* v, const T eps, const int jt)
{
    int i, j, p, q, u, w, t, s, l;
    T fm, cn, sn, omega, x, y, d;
    l = 1;
    for (i=0; i<=n-1; i++)
    {
        v[i*n+i] = 1.0;
        for (j=0; j<=n-1; j++)
        {
            if (i != j)
            {
                v[i*n+j] = 0.0;
            }
        }
    }
    while (1 == 1)
    {
        fm = 0.0;
        for (i=1; i<=n-1; i++)
        {
            for (j=0; j<=i-1; j++)
            {
                d = fabs(a[i*n+j]);
                if ((i!=j) && (d>fm))
                {
                    fm = d;
                    p = i;
                    q = j;
                }
            }
        }
        if (fm < eps)
        {
            return (1);
        }
        if (l > jt)
        {
            return (-1);
        }
        l = l + 1;
        u = p * n + q;
        w = p * n + p;
        t = q * n + p;
        s = q * n + q;
        x = -a[u];
        y = (a[s] - a[w]) / 2.0;
        omega = x / sqrt(x*x + y*y);
        if (y < 0.0)
        {
            omega =- omega;
        }
        sn = 1.0 + sqrt(1.0 - omega * omega);
        sn = omega / sqrt(2.0 * sn);
        cn = sqrt(1.0 - sn * sn);
        fm = a[w];
        a[w] = fm * cn * cn + a[s] * sn * sn + a[u] * omega;
        a[s] = fm * sn * sn + a[s] * cn * cn - a[u] * omega;
        a[u] = 0.0;
        a[t] = 0.0;
        for (j=0; j<=n-1; j++)
        {
            if ((j!=p) && (j!=q))
            {
                u = p * n + j;
                w = q * n + j;
                fm = a[u];
                a[u] =  fm * cn + a[w] * sn;
                a[w] =- fm * sn + a[w] * cn;
            }
        }
        for (i=0; i<=n-1; i++)
        {
            if ((i!=p) && (i!=q))
            {
                u = i * n + p;
                w = i * n + q;
                fm = a[u];
                a[u] =  fm * cn + a[w] * sn;
                a[w] =- fm * sn + a[w] * cn;
            }
        }
        for (i=0; i<=n-1; i++)
        {
            u = i * n + p;
            w = i * n + q;
            fm= v[u];
            v[u] =  fm * cn + v[w] * sn;
            v[w] =- fm * sn + v[w] * cn;
        }
    }
    return 1;
}
/*****************************************************************************
 Prototype    : Algorithm.jcbj
 Description  : the jacobian threshold method of computing the eigenvalues and eigenvectors of real symmetric matrices
 Input        : T* a        :store n order real symmetric matrix, n eigenvalues are stored on the diagonal when returned. (n*n)
                const int n :the order of matrix A.
                const T eps : Control accuracy requirement
 Output       :
                T* v        :Return the eigenvectors. Where the j-th column is the eigenvector corresponding to the j-th eigenvalue. (n*n)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/26
    Author       : lin.lin
    Modification : Created function
  note: The jacobian threshold method is better than jcbi (see commit:4ae230)
*****************************************************************************/
template<typename T>
void Algorithm::jcbj(T* a, const int n, T* v, const T eps)
{
    int i, j, p, q, u, w, t, s;
    T ff, fm, cn, sn, omega, x, y, d;
    for (i=0; i<=n-1; i++)
    {
        v[i*n+i] = 1.0;
        for (j=0; j<=n-1; j++)
        {
            if (i != j)
            {
                v[i*n+j] = 0.0;
            }
        }
    }
    ff = 0.0;
    for (i=1; i<=n-1; i++)
    {
        for (j=0; j<=i-1; j++)
        {
            d  = a[i*n+j];
            ff = ff + d * d;
        }
    }
    ff = sqrt(2.0 * ff);
loop0:
    ff = ff / (1.0 * n);
loop1:
    for (i=1; i<=n-1; i++)
    {
        for (j=0; j<=i-1; j++)
        {
            d = fabs(a[i*n+j]);
            if (d > ff)
            {
                p = i;
                q = j;
                goto loop;
            }
        }
    }
    if (ff < eps)
    {
        return;
    }
    goto loop0;
loop:
    u = p * n + q;
    w = p * n + p;
    t = q * n + p;
    s = q * n + q;
    x =- a[u];
    y = (a[s] - a[w]) / 2.0;
    omega = x / sqrt(x * x + y * y);
    if (y < 0.0)
    {
        omega =- omega;
    }
    sn = 1.0 + sqrt(1.0 - omega * omega);
    sn = omega / sqrt(2.0 * sn);
    cn = sqrt(1.0 - sn * sn);
    fm = a[w];
    a[w] = fm * cn * cn + a[s] * sn * sn + a[u] * omega;
    a[s] = fm * sn * sn + a[s] * cn * cn - a[u] * omega;
    a[u] = 0.0;
    a[t] = 0.0;
    for (j=0; j<=n-1; j++)
    {
        if ((j!=p) && (j!=q))
        {
            u = p * n + j;
            w = q * n + j;
            fm = a[u];
            a[u] =  fm * cn + a[w] * sn;
            a[w] =- fm * sn + a[w] * cn;
        }
    }
    for (i=0; i<=n-1; i++)
    {
        if ((i!=p)&&(i!=q))
        {
            u  = i * n + p;
            w  = i * n + q;
            fm = a[u];
            a[u] =  fm * cn + a[w] * sn;
            a[w] =- fm * sn + a[w] * cn;
        }
    }
    for (i=0; i<=n-1; i++)
    {
        u  = i * n + p;
        w  = i * n + q;
        fm = v[u];
        v[u] =  fm * cn + v[w] * sn;
        v[w] =- fm * sn + v[w] * cn;
    }
    goto loop1;
}
/*****************************************************************************
 Prototype    : Algorithm.min
 Description  : get the Smallest elements in array
 Input        : const double* pdSrcData: pointer to the array of source data
                const int nSize        : the array dimension
 Output       : int* pIndex            : the index corresponding to the Largest element.
                                         If the maximum value occurs more than once, then max returns the index corresponding to the first occurrence.
 Return Value : T
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/24
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
T Algorithm::min(const T* pdSrcData, const int nSize, int* pIndex)
{
    T dTempCur = 0;
    T dTempMin = pdSrcData[0];
    int iIndex = 0;
    for (int i = 1; i < nSize; ++i)
    {
        dTempCur = pdSrcData[i];
        if (dTempCur < dTempMin)
        {
            dTempMin = dTempCur;
            iIndex   = i;
        }
    }

    if ( NULL != pIndex )
    {
        *pIndex = iIndex;
    }
    return dTempMin;
}
/*****************************************************************************
 Prototype    : Algorithm.max
 Description  : get the Largest elements in array
 Input        : const T* pdSrcData: pointer to the array of source data
                const int nSize   : the array dimension

 Output       : int* pIndex       : the index corresponding to the Largest element.
                                    If the maximum value occurs more than once, then max returns the index corresponding to the first occurrence.
 Return Value : T
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/26
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
T Algorithm::max(const T* pdSrcData, const int nSize, int* pIndex)
{
    T dTempCur = 0;
    T dTempMax = pdSrcData[0];
    int iIndex = 0;
    for (int i = 1; i < nSize; ++i)
    {
        dTempCur = pdSrcData[i];
        if (dTempCur > dTempMax)
        {
            dTempMax = dTempCur;
            iIndex   = i;
        }
    }

    if ( NULL != pIndex )
    {
        *pIndex = iIndex;
    }
    return dTempMax;
}

template<typename T>
void Algorithm::swap(T *a, T *b)
{
    T c;
    c  = *a;
    *a = *b;
    *b =  c;
}
/*****************************************************************************
 Prototype    : Algorithm.getDlsFactors
 Description  : get the damped factors in damped least squres
 Input        : const T tMaxLamda : the maximum damping factor
                const T tEpsilon  : the minimal singular value
                const T *tarrSigma: the singular values of specified matrix.
                const int iSigNum : the numbers of of singular values.

 Output       : T *tarrLamda      : the dynamic damping factor we get.
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/12/12
    Author       : lin.lin
    Modification : Created function
  note: fyi: Repeatable kinematic control of redundant manipulators_ implementation issues, (24)
*****************************************************************************/
template<typename T>
void Algorithm::getDlsFactors(const T tMaxLamda, const T tEpsilon, const T *tarrSigma, const int iSigNum, T *tarrLamda)
{
    for ( int i = 0 ; i < iSigNum ; i++ )
    {
        if ( tarrSigma[i] < tEpsilon )
        {
            tarrLamda[i] = tMaxLamda * (1 +cos(PI*tarrSigma[i]/tEpsilon)) / 2;
            // tarrLamda[i] = tMaxLamda * tMaxLamda * (1 - tarrSigma[i]/tEpsilon); // simplify computation
        }else
        {
            tarrLamda[i] = 0.0;
        }
    }
}
template<typename T>
void Algorithm::getMatrixWithDamp(T* tarrSymMatrix, const int iMatrixDim, const T tMaxLamda, const T tEpsilon)
{
    // initial jcbj
    const T eps           = 0.000001;
    T* ptarrAcpy = NULL;

    ptarrAcpy = (T*)malloc(sizeof(T) * iMatrixDim*iMatrixDim);
    int iMatrixMaxDim = RMRC_MAX_JOINT_NUM;
    T tarrEigenVectorAct[RMRC_MAX_JOINT_NUM*RMRC_MAX_JOINT_NUM] = {0};
    T tarrEigenValueAct[RMRC_MAX_JOINT_NUM]  = {0};
    // initial getDlsFactors
    T tarrLamda[RMRC_MAX_JOINT_NUM] = { 0 };

    int i, j;
    memcpy(ptarrAcpy, tarrSymMatrix, sizeof(T)*iMatrixDim*iMatrixDim);
    jcbj(ptarrAcpy, iMatrixDim, tarrEigenVectorAct, eps);

    // get EigenValue
    for ( j = 0 ; j <iMatrixDim ; ++j )
    {
        tarrEigenValueAct[j] = ptarrAcpy[j*iMatrixDim+j];
    }

    getDlsFactors(tMaxLamda, tEpsilon, tarrEigenValueAct, iMatrixDim, tarrLamda);
    for ( i = 0 ; i <iMatrixDim ; i++ )
    {
        tarrSymMatrix[i*iMatrixDim+i] += tarrLamda[i]*tarrLamda[i];
    }
    free(ptarrAcpy);
}

/*****************************************************************************
 Prototype    : gmiv
 Description  : Generalized inverse method for solving linear least squares problems
 Input        : double* a : Store the coefficient matrix A of the overdetermined system of equation.
                int m     : the row number of A
                int n     : the column number of A
                double* b : The constant vector on the right side of the overdetermined system; (m*1)
                double eps: Control accuracy requirements in singular value decomposition
                int ka    : max(m, n) + 1
 Output       :
                double* x : Returns the least squares solution of the overdetermined system; (n*1)
                double* aa: Returns the generalized inverse A+ of the coefficient matrix A; (n*m)
                double* u : Return the left singular vector U when make SVD on the coefficient matrix A; (m*m)
                double* v : Return the right singular vector VT when make SVD on the coefficient matrix A; (n*n)
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/22
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
template<typename T>
int Algorithm::gmiv(T* a, int m, int n, T* b, T* x, T* aa, T eps, T* u, T* v, int ka)
{
    int i, j;
    i = Algorithm::pinv(a, m, n, aa, eps, u, v, ka);
    if (i < 0)
        return(-1);
    for (i=0; i<=n-1; i++)
    {
        x[i]=0.0;
        for (j=0; j<=m-1; j++)
        {
            x[i] = x[i] + aa[i*m+j] * b[j]; // X = A+ * B
        }
    }
    return(1);
}
/*****************************************************************************
 Prototype    : Algorithm.MatrixRank
 Description  : get the Rank of matrix
 Input        : T* tarrMatrix: store the elements of the matrix A, and it will be destroyed when returned.
                int m        : the row number of matrix
                int n        : the column number of matrix
 Output       : None
 Return Value : int          : the rank of matrix
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/5
    Author       : lin.lin
    Modification : Created function
  Note: uses the method based on the gauss elimination method for all selected principal elements.
*****************************************************************************/
template<typename T>
int Algorithm::MatrixRank(T* tarrMatrix, int m, int n)
{
    int i, j, k, nn, is, js, l, ll, u, v;
    T q, d;
    nn = m;
    if (m >= n) nn = n;
    k = 0;
    for (l=0; l<=nn-1; l++)
    {
        q=0.0;
        for (i=l; i<=m-1; i++)
            for (j=l; j<=n-1; j++)
            {
                ll = i * n + j;
                d = fabs(tarrMatrix[ll]);
                if (d > q)
                {
                    q  = d;
                    is = i;
                    js = j;
                }
            }
        if (q+1.0==1.0)
        {
            return(k);
        }
        k = k + 1;
        if (is!=l)
        {
            for (j=l; j<=n-1; j++)
            {
                u = l  * n + j;
                v = is * n + j;
                d = tarrMatrix[u];
                tarrMatrix[u] = tarrMatrix[v];
                tarrMatrix[v] = d;
            }
        }
        if (js != l)
        {
            for (i=l; i<=m-1; i++)
            {
                u = i * n + js;
                v = i * n + l;
                d = tarrMatrix[u];
                tarrMatrix[u] = tarrMatrix[v];
                tarrMatrix[v] = d;
            }
        }
        ll = l * n + l;
        //for (i=l+1; i<=n-1; i++)
        for (i=l+1; i<=m-1; i++)
        {
            d = tarrMatrix[i*n+l] / tarrMatrix[ll];
            for (j=l+1; j<=n-1; j++)
            {
                u = i * n + j;
                tarrMatrix[u] = tarrMatrix[u] - d * tarrMatrix[l*n+j];
            }
        }
    }
    return(k);
}
/*****************************************************************************
 Prototype    : Algorithm.diag
 Description  : Create a square diagonal matrix with the elements of vector v on the main diagonal.
 Input        : const T* tarrVecW: the elements of vector v
                const int iDim   : the dimension of the square diagonal matrix

 Output       : T* tarrW         : returns a square diagonal matrix
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/12/25
    Author       : lin.lin
    Modification : Created function
  note: fyi: matlab command: diag
*****************************************************************************/
template<typename T>
void Algorithm::diag(const T* tarrVecW, const int iDim, T* tarrW)
{
    int i, j;
    for ( i = 0 ; i < iDim ; ++i )
    {
        for ( j = 0 ; j < iDim ; ++j )
        {
            if ( i == j )
            {
                tarrW[i*iDim+j] = tarrVecW[i];
            }else
            {
                tarrW[i*iDim+j] = 0.0;
            }
        }
    }
}
/*****************************************************************************
 Prototype    : Algorithm.getWeightRightPinv
 Description  : get weighted right pseudo-inverse matrix
 Input        : const T* tarrM: the source matrix (iRow*iCol)
                const int iRow: the row number of matrix tarrM
                const int iCol: the column number of matrix tarrM
                T* tarrVecW   : weight vector (iCol*1) to create positive definite weight matrix (iCol*iCol)
 Output       : T* tarrPinvM  : return weighted right pseudo-inverse matrix

 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/12/25
    Author       : lin.lin
    Modification : Created function
  note: fyi: A.7 Pseudo-inverse, Bruno Siciliano
*****************************************************************************/
template<typename T>
void Algorithm::getWeightRightPinv(const T* tarrM, const int iRow, const int iCol, T* tarrPinvM, T* tarrVecW)
{
    // initial value
    const T tMaxLamda                          = 0.05;      // data from hanlei.wang, p13
    const T tEpsilon                           = tMaxLamda;
    const int iRowMaxNum                       = 16;
    const int iColMaxNum                       = 16;
    T tarrMT[iColMaxNum*iRowMaxNum]            = {0};       // transpose of tarrM: tarrM'
    T tarrMMT[iRowMaxNum*iRowMaxNum]           = {0};       // transpose of tarrM: tarrM'
    T* ptInvMMT                                = NULL;
    T tarrW[iColMaxNum*iColMaxNum]             = {0};
    T tarrMW[iRowMaxNum*iColMaxNum]            = {0};
    T tarrMWT[iColMaxNum*iRowMaxNum]           = {0};
    int i = 0;
    if ( NULL == tarrVecW )
    {
        eye(tarrW, iCol);
    }else
    {
        // get inv(W)
        for ( i = 0 ; i <iCol ; ++i )
        {
            tarrVecW[i] = 1.0 / tarrVecW[i];
        }
        diag(tarrVecW, iCol, tarrW);
    }
    // get tarrMT
    GeometryUtils::MatrixMultiply(tarrM,   tarrW, iRow, iCol, iCol, tarrMW); // A * invW
    GeometryUtils::MatrixTranspose(tarrMW, iRow,  iCol, tarrMWT); //  invW * A'
    GeometryUtils::MatrixTranspose(tarrM,  iRow,  iCol, tarrMT);  // A'

    // get tarrMW*tarrMT
    GeometryUtils::MatrixMultiply(tarrMW, tarrMT, iRow, iCol, iRow, tarrMMT);      // A * invW * A'
#ifdef RMRC_KINEMATIC_SINGULARITIES
    // kinematic singularities process; (3.59), Bruno Siciliano
    if ( 0.0 < tEpsilon )
    {
        getMatrixWithDamp(tarrMMT, iRow, tMaxLamda, tEpsilon);
    }
#endif
    // get Inv(J*JT)
    ptInvMMT = tarrMMT;
    GeometryUtils::MatrixInv(ptInvMMT, iRow); // inv(A * invW * A')
    // get J_pseu (J+ = JT * inv(J*JT))
    GeometryUtils::MatrixMultiply(tarrMWT, ptInvMMT, iCol, iRow, iRow, tarrPinvM); // invW * A' * inv(A * invW * A')
}
template<typename T>
void Algorithm::getWeightLeftPinv(const T* tarrM, const int iRow, const int iCol, T* tarrPinvM, T* tarrVecW)
{
    // initial value
    const T tMaxLamda                          = 0.05;      // data from hanlei.wang, p13
    const T tEpsilon                           = tMaxLamda;
    const int iRowMaxNum                       = 16;
    const int iColMaxNum                       = 16;
    T tarrMT[iColMaxNum*iRowMaxNum]            = {0};       // transpose of tarrM: tarrM'
    T tarrMMT[iRowMaxNum*iRowMaxNum]           = {0};       // transpose of tarrM: tarrM'
    T tarrW[iRowMaxNum*iRowMaxNum]             = {0};
    T tarrMTWM[iColMaxNum*iColMaxNum]          = {0};
    T tarrMTW[iColMaxNum*iRowMaxNum]           = {0};
    int i = 0;
    if ( NULL == tarrVecW )
    {
        eye(tarrW, iRow);
    }else
    {
        // get W
        diag(tarrVecW, iRow, tarrW);
    }
    // get tarrMT
    GeometryUtils::MatrixTranspose(tarrM,  iRow,  iCol, tarrMT);
    GeometryUtils::MatrixMultiply(tarrMT,  tarrW, iCol, iRow, iRow, tarrMTW); // A' * W
    GeometryUtils::MatrixMultiply(tarrMTW, tarrM, iCol, iRow, iCol, tarrMTWM);//A' * W * A
#ifdef RMRC_KINEMATIC_SINGULARITIES
        // kinematic singularities process; (3.59), Bruno Siciliano
        if ( 0.0 < tEpsilon )
        {
            getMatrixWithDamp(tarrMTWM, iCol, tMaxLamda, tEpsilon);
        }
#endif
    GeometryUtils::MatrixInv(tarrMTWM, iCol); // inv(A' * W * A)
    GeometryUtils::MatrixMultiply(tarrMTWM, tarrMTW, iCol, iCol, iRow, tarrPinvM);  // inv(A' * W * A) * A' * W; tarrPinvM: iCol*iRow
}

/*****************************************************************************
 Prototype    : Algorithm.MatrixInv
 Description  : get the inverse matrix
 Input        : const T* tarrA   : matrix to be inversed
                const int row    : the row number of the matrix
                const int column : the column number of the matrix

 Output       : T* p             : pointed to the memory stored the inverse matrix
 Return Value : int              : 0 - failed; 1- success.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/5
    Author       : lin.lin
    Modification : Created function
  note: blog.csdn.net/luanpeng825485697/article/details/77619372
*****************************************************************************/
template<typename T>
int Algorithm::MatrixInv(const T* tarrA, const int row, const int column, T* p)
{
    if (row != column)
    {
        printf("The inverse matrix is not square!\n");
        return NULL;
    }
    int *is, *js, i, j, k;
    int n = row;
    T temp, fmax;
    //Matrix p(a.row,a.column);
    for(i=0; i<n; i++)
    {
        for(j=0; j<n; j++)
            p[i*n+j] = tarrA[i*n+j];
    }
    is = new int[n];
    js = new int[n];
    for(k=0; k<n; k++)
    {
        fmax = 0.0;
        for(i=k; i<n; i++)
            for(j=k; j<n; j++)
            {
                temp = std::fabs(p[i*n+j]);// find max value
                if(temp > fmax)
                {
                    fmax = temp;
                    is[k] = i; js[k] = j;
                }
            }
            if((fmax + 1.0) == 1.0)
            {
                delete[] is;
                delete[] js;
                //throw_logic_error("There is no inverse matrix!");
                printf("There is no inverse matrix!\n");
                return NULL;
            }
            if((i=is[k]) != k)
                for(j=0; j<n; j++)
                    swap(p+k*n+j, p+i*n+j);// swap pointer
            if((j=js[k]) != k)
                for(i=0; i<n; i++)
                    swap(p+i*n+k, p+i*n+j);  // swap pointer
            p[k*n+k] = 1.0 / p[k*n+k];
            for(j=0; j<n; j++)
                if(j != k)
                    p[k*n+j] *= p[k*n+k];
            for(i=0; i<n; i++)
                if(i != k)
                    for(j=0; j<n; j++)
                        if(j != k)
                            p[i*n+j] = p[i*n+j] - p[i*n+k] * p[k*n+j];
            for(i=0; i<n; i++)
                if(i != k)
                    p[i*n+k] *= -p[k*n+k];
    }
    for(k=n-1; k>=0; k--)
    {
        if((j=js[k]) != k)
            for(i=0; i<n; i++)
                swap((p+j*n+i),(p+k*n+i));
        if((i=is[k]) != k)
            for(j=0; j<n; j++)
                swap((p+j*n+i),(p+j*n+k));
    }
    delete[] is;
    delete[] js;
    return 1;
}
#endif

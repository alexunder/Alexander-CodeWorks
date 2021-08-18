#include <iostream>
#include "Algorithm.h"
#include "Utils/KyComDef.h"

#include "Utils/UtilFunc.h"

// inner called;
static void framePosTrans(const double* pdSrcPos, const double* pdTransHomogeneous, double* pdDstPos)
{
    pdDstPos[0] = pdTransHomogeneous[0] * pdSrcPos[0] + pdTransHomogeneous[1] * pdSrcPos[1] + pdTransHomogeneous[2]  * pdSrcPos[2] + pdTransHomogeneous[3] ;
    pdDstPos[1] = pdTransHomogeneous[4] * pdSrcPos[0] + pdTransHomogeneous[5] * pdSrcPos[1] + pdTransHomogeneous[6]  * pdSrcPos[2] + pdTransHomogeneous[7] ;
    pdDstPos[2] = pdTransHomogeneous[8] * pdSrcPos[0] + pdTransHomogeneous[9] * pdSrcPos[1] + pdTransHomogeneous[10] * pdSrcPos[2] + pdTransHomogeneous[11] ;
}

//Orientation is 3x3
//homogeneous is 4x4
static void frameOrientationTrans(const double* pdSrcOrientation, const double* pdTransHomogeneous, double* pdDstOrientation)
{
    pdDstOrientation[0] = pdTransHomogeneous[0] * pdSrcOrientation[0] + pdTransHomogeneous[1] * pdSrcOrientation[3] + pdTransHomogeneous[2]  * pdSrcOrientation[6] ;
    pdDstOrientation[1] = pdTransHomogeneous[0] * pdSrcOrientation[1] + pdTransHomogeneous[1] * pdSrcOrientation[4] + pdTransHomogeneous[2]  * pdSrcOrientation[7] ;
    pdDstOrientation[2] = pdTransHomogeneous[0] * pdSrcOrientation[2] + pdTransHomogeneous[1] * pdSrcOrientation[5] + pdTransHomogeneous[2]  * pdSrcOrientation[8] ;
    pdDstOrientation[3] = pdTransHomogeneous[4] * pdSrcOrientation[0] + pdTransHomogeneous[5] * pdSrcOrientation[3] + pdTransHomogeneous[6]  * pdSrcOrientation[6] ;
    pdDstOrientation[4] = pdTransHomogeneous[4] * pdSrcOrientation[1] + pdTransHomogeneous[5] * pdSrcOrientation[4] + pdTransHomogeneous[6]  * pdSrcOrientation[7] ;
    pdDstOrientation[5] = pdTransHomogeneous[4] * pdSrcOrientation[2] + pdTransHomogeneous[5] * pdSrcOrientation[5] + pdTransHomogeneous[6]  * pdSrcOrientation[8] ;
    pdDstOrientation[6] = pdTransHomogeneous[8] * pdSrcOrientation[0] + pdTransHomogeneous[9] * pdSrcOrientation[3] + pdTransHomogeneous[10] * pdSrcOrientation[6] ;
    pdDstOrientation[7] = pdTransHomogeneous[8] * pdSrcOrientation[1] + pdTransHomogeneous[9] * pdSrcOrientation[4] + pdTransHomogeneous[10] * pdSrcOrientation[7] ;
    pdDstOrientation[8] = pdTransHomogeneous[8] * pdSrcOrientation[2] + pdTransHomogeneous[9] * pdSrcOrientation[5] + pdTransHomogeneous[10] * pdSrcOrientation[8] ;
}

static void frameOrt2HomoTrans(const double* pdSrcOrt, const double* pdTransHomogeneous, double* pdDstOrtInHomo)
{
    pdDstOrtInHomo[0]  = pdTransHomogeneous[0] * pdSrcOrt[0] + pdTransHomogeneous[1] * pdSrcOrt[3] + pdTransHomogeneous[2] * pdSrcOrt[6] ;
    pdDstOrtInHomo[1]  = pdTransHomogeneous[0] * pdSrcOrt[1] + pdTransHomogeneous[1] * pdSrcOrt[4] + pdTransHomogeneous[2] * pdSrcOrt[7] ;
    pdDstOrtInHomo[2]  = pdTransHomogeneous[0] * pdSrcOrt[2] + pdTransHomogeneous[1] * pdSrcOrt[5] + pdTransHomogeneous[2] * pdSrcOrt[8] ;
    pdDstOrtInHomo[4]  = pdTransHomogeneous[4] * pdSrcOrt[0] + pdTransHomogeneous[5] * pdSrcOrt[3] + pdTransHomogeneous[6] * pdSrcOrt[6] ;
    pdDstOrtInHomo[5]  = pdTransHomogeneous[4] * pdSrcOrt[1] + pdTransHomogeneous[5] * pdSrcOrt[4] + pdTransHomogeneous[6] * pdSrcOrt[7] ;
    pdDstOrtInHomo[6]  = pdTransHomogeneous[4] * pdSrcOrt[2] + pdTransHomogeneous[5] * pdSrcOrt[5] + pdTransHomogeneous[6] * pdSrcOrt[8] ;
    pdDstOrtInHomo[8]  = pdTransHomogeneous[8] * pdSrcOrt[0] + pdTransHomogeneous[9] * pdSrcOrt[3] + pdTransHomogeneous[10] * pdSrcOrt[6] ;
    pdDstOrtInHomo[9]  = pdTransHomogeneous[8] * pdSrcOrt[1] + pdTransHomogeneous[9] * pdSrcOrt[4] + pdTransHomogeneous[10] * pdSrcOrt[7] ;
    pdDstOrtInHomo[10] = pdTransHomogeneous[8] * pdSrcOrt[2] + pdTransHomogeneous[9] * pdSrcOrt[5] + pdTransHomogeneous[10] * pdSrcOrt[8] ;
}

void Algorithm::KukaAndWorldTransmatrix(const Point& one,const Point& two,const Point& three,double* trans)
{
    Vector worldXInKuka = GeometryUtils::pointToVector(one,two);
    Vector worldYInKuka = GeometryUtils::pointToVector(one,three);
    Vector worldZInKuka = GeometryUtils::getNormal(worldXInKuka,worldYInKuka);

    trans[0] = cos(GeometryUtils::getVectorAngle(worldXInKuka,Vector(1,0,0)));
    trans[1] = cos(GeometryUtils::getVectorAngle(worldYInKuka,Vector(1,0,0)));
    trans[2] = cos(GeometryUtils::getVectorAngle(worldZInKuka,Vector(1,0,0)));
    trans[3] = one.x();
    trans[4] = cos(GeometryUtils::getVectorAngle(worldXInKuka,Vector(0,1,0)));
    trans[5] = cos(GeometryUtils::getVectorAngle(worldYInKuka,Vector(0,1,0)));
    trans[6] = cos(GeometryUtils::getVectorAngle(worldZInKuka,Vector(0,1,0)));
    trans[7] = one.y();
    trans[8] = cos(GeometryUtils::getVectorAngle(worldXInKuka,Vector(0,0,1)));
    trans[9] = cos(GeometryUtils::getVectorAngle(worldYInKuka,Vector(0,0,1)));
    trans[10] = cos(GeometryUtils::getVectorAngle(worldZInKuka,Vector(0,0,1)));
    trans[11] = one.z();
    trans[12] = 0;
    trans[13] = 0;
    trans[14] = 0;
    trans[15] = 1;
}
/*****************************************************************************
 Prototype    : Algorithm.GetInstrumentFrame
 Description  : Get instrument frame according by 3 motors position, 3 motors home positon
                and frange frame, output the instrument frame and clip angle (degree).
 Input        : [in] motor1Position     : the 1st motor's position
                [in] motor2Position     : the 2nd motor's position
                [in] motor3Position     : the 3rd motor's position
                [in] motor1HomePosition : the 1st motor's home position
                [in] motor2HomePosition : the 2nd motor's home position
                [in] motor3HomePosition : the 3rd motor's home position
                [in] flange             : the flange's frame
 Output       :
                [out] instrumentFrame   : the instrument's frame  (clip's frame)
                [out] double& clip      : clip's angle (degree)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         :
    Author       :
    Modification : Created function

*****************************************************************************/
void Algorithm::GetInstrumentFrame(
    const double& motor1Position,
    const double& motor2Position,
    const double& motor3Position,
    const double& motor1HomePosition,
    const double& motor2HomePosition,
    const double& motor3HomePosition,
    const Frame& flange,
    Frame& instrumentFrame, double& clip)
{
    double pitchFactor = 1.0;
    double leftClipFactor = 1.0;
    double rightClipFactor = 1.0;
    double pitchInfluenceFactor = 1.0;
    //TODO
    //confirm
    //which is X axis in flange frame
    double roll = flange.mOrientation.mData[6];

    double pitch = pitchFactor * (motor1Position - motor1HomePosition);
    double yawStart = pitchInfluenceFactor * pitch;
    double leftAngle = (motor2Position - motor2HomePosition - yawStart) * leftClipFactor;
    double rightAngle = (motor3Position - motor3HomePosition - yawStart) * rightClipFactor;

    double yaw = (leftAngle + rightAngle) / 2;

    double* rotate = GeometryUtils::MatrixMultiply(GeometryUtils::rotateY(yaw),3,3,
                                                    GeometryUtils::rotateX(pitch),3,3);

    Vector x(1,0,0);
    Vector y(0,1,0);
    Vector z(0,0,1);

    Vector newX(GeometryUtils::MatrixMultiply(rotate,3,3,x.mData,3,1));
    Vector newY(GeometryUtils::MatrixMultiply(rotate,3,3,y.mData,3,1));
    Vector newZ(GeometryUtils::MatrixMultiply(rotate,3,3,z.mData,3,1));

    Frame f=Frame::identity();
    f.mOrientation.mData[0] = cos(GeometryUtils::getVectorAngle(newX,Vector(1,0,0)));
    f.mOrientation.mData[1] = cos(GeometryUtils::getVectorAngle(newY,Vector(1,0,0)));
    f.mOrientation.mData[2] = cos(GeometryUtils::getVectorAngle(newZ,Vector(1,0,0)));
    f.mOrientation.mData[3] = cos(GeometryUtils::getVectorAngle(newX,Vector(0,1,0)));
    f.mOrientation.mData[4] = cos(GeometryUtils::getVectorAngle(newY,Vector(0,1,0)));
    f.mOrientation.mData[5] = cos(GeometryUtils::getVectorAngle(newZ,Vector(0,1,0)));
    f.mOrientation.mData[6] = cos(GeometryUtils::getVectorAngle(newX,Vector(0,0,1)));
    f.mOrientation.mData[7] = cos(GeometryUtils::getVectorAngle(newY,Vector(0,0,1)));
    f.mOrientation.mData[8] = cos(GeometryUtils::getVectorAngle(newZ,Vector(0,0,1)));

    instrumentFrame = f;
    clip = rightAngle - leftAngle;
    /* BEGIN: Added by lin.lin, 2017/4/1 */
    delete[] rotate;
    /* END:   Added by lin.lin, 2017/4/1   PN: */
}

Frame Algorithm::LaparoToEyeMapping(const Frame& instrumentFrameInSlave,const double& factor)
{

    Frame frame = Frame::identity();
    frame.mOrigine.mData[0] = instrumentFrameInSlave.mOrigine.mData[0] * factor;
    frame.mOrigine.mData[1] = instrumentFrameInSlave.mOrigine.mData[1] * factor;
    frame.mOrigine.mData[2] = instrumentFrameInSlave.mOrigine.mData[2] * factor;

    frame.mOrientation = instrumentFrameInSlave.mOrientation;
    return frame;
}

Frame Algorithm::EyeToLaparoMapping(const Frame& clipFrameInMaster,const double& factor)
{
    Frame frame = Frame::identity();
    frame.mOrigine.mData[0] = clipFrameInMaster.mOrigine.mData[0] / factor;
    frame.mOrigine.mData[1] = clipFrameInMaster.mOrigine.mData[1] / factor;
    frame.mOrigine.mData[2] = clipFrameInMaster.mOrigine.mData[2] / factor;

    frame.mOrientation = clipFrameInMaster.mOrientation;
    return frame;
}
void Algorithm::SpaceConvert(const Frame& inputLeft,const Frame& inputRight,Frame& outputLeft,Frame& outputRight)
{
    //unit mm
    double det = 3.0;
    memcpy(outputLeft.mOrientation.mData,inputLeft.mOrientation.mData,sizeof(double)*3);
    memcpy(outputRight.mOrientation.mData,inputRight.mOrientation.mData,sizeof(double)*3);

    outputLeft.mOrigine.mData[0] = inputLeft.mOrigine.mData[0] + det/2;
    outputLeft.mOrigine.mData[1] = inputLeft.mOrigine.mData[1];
    outputLeft.mOrigine.mData[2] = inputLeft.mOrigine.mData[2];

    outputRight.mOrigine.mData[0] = inputRight.mOrigine.mData[0] + det/2;
    outputRight.mOrigine.mData[1] = inputRight.mOrigine.mData[1];
    outputRight.mOrigine.mData[2] = inputRight.mOrigine.mData[2];
}


void Algorithm::SpaceReconvert(const Frame& inputLeft,const Frame& inputRight,Frame& outputLeft,Frame& outputRight)
{
    //unit mm
    double det = 3.0;
    memcpy(outputLeft.mOrientation.mData,inputLeft.mOrientation.mData,sizeof(double)*3);
    memcpy(outputRight.mOrientation.mData,inputRight.mOrientation.mData,sizeof(double)*3);

    outputLeft.mOrigine.mData[0] = inputLeft.mOrigine.mData[0] - det/2;
    outputLeft.mOrigine.mData[1] = inputLeft.mOrigine.mData[1];
    outputLeft.mOrigine.mData[2] = inputLeft.mOrigine.mData[2];

    outputRight.mOrigine.mData[0] = inputRight.mOrigine.mData[0] - det/2;
    outputRight.mOrigine.mData[1] = inputRight.mOrigine.mData[1];
    outputRight.mOrigine.mData[2] = inputRight.mOrigine.mData[2];
}
/*****************************************************************************
 Prototype    : Algorithm.KinematicMapping
 Description  : Kinematic Mapping
 Input        : clipFrame      : clip frame (relative to kuka base coordinate system),
                                 where the original point is clip joint (target position), the z axis is along the shaft from begin point to end point.
                trocarPosition : the position(type: Vector) of trocar;
                shalftLength   : the length of shaft
                jointLength    : the length between pitch joint and yaw joint(clip joint)
 Output       :
                yaw            :(radian measure)
                pitch          :(radian measure)
                roll           :(radian measure)
                flangeFrame    : flange frame (relative to kuka base coordinate system)

 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/3/30
    Author       :
    Modification : Created function
 notes:
    1, work out clip's gesture (yaw, pitch, roll) according clip's frame;
    2, work out flange' frame, including:
       1) work out the original point of flange's frame according trocarPosition, target Position(original position of clip's Frame), shalftLength, jointLength, pitch;
       2) work out the rotation matrix of flange's frame according roll.
    3, yaw has nothing to do with the flange's frame.
*****************************************************************************/
void Algorithm::KinematicMapping(
        const Frame& clipFrame,      const Vector& trocarPosition,
        const double& shalftLength,  const double& jointLength,
        double& yaw,double& pitch,   double& roll,Frame& flangeFrame
    )
{
    //work out clip's gesture (yaw, pitch, roll) according clip's frame
    GeometryUtils::getRYPFromFrame(clipFrame, roll, yaw, pitch);
    //work out the point of end of shalft (endofJoint)
    Vector targetPosition = clipFrame.mOrigine;
    double *endOfJointData = GeometryUtils::MatrixMultiply(GeometryUtils::rotateX(pitch),3,3, Vector(0,0,-jointLength).mData,3,1);
    Point endofJoint(endOfJointData[0]+targetPosition.mData[0],endOfJointData[1]+targetPosition.mData[1],endOfJointData[2]+targetPosition.mData[2]);
    //work out new frange frame after rotating roll angle.
    double *matrix = GeometryUtils::MatrixMultiply(Rotation::RotZ(roll).mData,3,3, flangeFrame.mOrientation.mData,3,3);
    memcpy(flangeFrame.mOrientation.mData, matrix, sizeof(double)*9);
    delete[] endOfJointData;
    delete[] matrix;
    //the shaft length inside trocar point.
    double insideLength = GeometryUtils::Distance(trocarPosition, GeometryUtils::pointToVector(Point(0,0,0),endofJoint));
    //the shaft length outside trocar point.
    double leftLength = shalftLength - insideLength;

    double flangeX = (shalftLength * trocarPosition.mData[0] - leftLength * endofJoint.mData[0]) / insideLength;
    double flangeY = (shalftLength * trocarPosition.mData[1] - leftLength * endofJoint.mData[1]) / insideLength;
    double flangeZ = (shalftLength * trocarPosition.mData[2] - leftLength * endofJoint.mData[2]) / insideLength;

    flangeFrame.mOrigine.mData[0] = flangeX;
    flangeFrame.mOrigine.mData[1] = flangeY;
    flangeFrame.mOrigine.mData[2] = flangeZ;
}
/*****************************************************************************
 Prototype    : Algorithm.RPYToMotors
 Description  : transform clip's gesture to each 3 motor's position (degree).
 Input        : yaw               : the angle of yaw (degree)
                pitch             : the angle of pitch (degree)
                clip              : the angle between 2 clips (degree)
                motor1HomePosition: motor 1's home position (degree)
                motor2HomePosition: motor 2's home position (degree)
                motor3HomePosition: motor 3's home position (degree)

 Output       :
                motor1: motor 1's position (degree)
                motor2: motor 2's position (degree)
                motor3: motor 3's position (degree)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/4/6
    Author       : lin.lin
    Modification : Created function
*****************************************************************************/
//the Algorithm from Verona
void Algorithm::RPYToMotors(
    const double& yaw, const double& pitch, const double& clip,
    const double& motor1HomePosition,
    const double& motor2HomePosition,
    const double& motor3HomePosition,
    double& motor1, double& motor2, double& motor3)
{
#if 0
    double pitchFactorForMotor1 = 1.01857984;
    double yawFactorForMotor1 = -0.83063427;
    double yawFactorForMotor2 = 0.60886298;
    double yawFactorForMotor3 = 0.60886298;
    double gripFactorForMotor2 = -1.21772597;
    double gripFactorForMotor3 = 1.21772597;
#endif
    double pitchFactorForMotor1 = 1.01857984;
    double pitchFactorForMotor2 = 1;
    double pitchFactorForMotor3 = 1;
    double yawFactorForMotor1 = 0.83063427;
    double yawFactorForMotor2 = -0.60886298;
    double yawFactorForMotor3 = 0.60886298;
    double gripFactorForMotor2 = 1.21772597;
    double gripFactorForMotor3 = -1.21772597;

    double detaMotor1 = pitch * pitchFactorForMotor1 + yaw * yawFactorForMotor1;
    double detaMotor2 = yaw * yawFactorForMotor2 + pitch * pitchFactorForMotor2 + clip * gripFactorForMotor2;
    double detaMotor3 = yaw * yawFactorForMotor3 + pitch * pitchFactorForMotor3 + clip * gripFactorForMotor3;

    motor1 = (motor1HomePosition + detaMotor1);
    motor2 = motor2HomePosition + detaMotor2;
    motor3 = motor3HomePosition + detaMotor3;
}

void Algorithm::Large_Motors2PY(double dMotor1, double dMotor2, double dMotor3,
    double& dYaw, double& dPitch,  double& dClip)
{
    /* BEGIN: Added by lin.lin, 2017/12/25 */
    // range protect
    if ( dMotor1 > LARGE_MOTORS2PY_MOTOR1_MAX )
    {
        dMotor1 = LARGE_MOTORS2PY_MOTOR1_MAX;
    }
    if ( dMotor1 < LARGE_MOTORS2PY_MOTOR1_MIN )
    {
        dMotor1 = LARGE_MOTORS2PY_MOTOR1_MIN;
    }
    if ( dMotor2 > LARGE_MOTORS2PY_MOTOR2_MAX )
    {
        dMotor2 = LARGE_MOTORS2PY_MOTOR2_MAX;
    }
    if ( dMotor2 < LARGE_MOTORS2PY_MOTOR2_MIN )
    {
        dMotor2 = LARGE_MOTORS2PY_MOTOR2_MIN;
    }
    if ( dMotor3 > LARGE_MOTORS2PY_MOTOR3_MAX )
    {
        dMotor3 = LARGE_MOTORS2PY_MOTOR3_MAX;
    }
    if ( dMotor3 < LARGE_MOTORS2PY_MOTOR3_MIN )
    {
        dMotor3 = LARGE_MOTORS2PY_MOTOR3_MIN;
    }
    /* END:   Added by lin.lin, 2017/12/25   PN: */

    double p00[3] = {0.007786,    2.058,   1.798  };
    double p10[3] = {1.003,       0.6346,  0.6341 };
    double p01[3] = {-0.0002872,  0.8271,  0.8221 };

    double pclip[3] = {0.0, p01[1]/2, -p01[2]/2};

    double Pmatrix[9] = {
            p10[0],p01[0],pclip[0],
            p10[1],p01[1],pclip[1],
            p10[2],p01[2],pclip[2]
    };
    double Pm_inv[9] = {0};
    double dArrMotorSubP00[3] = {0};
    dArrMotorSubP00[0] = dMotor1 - p00[0];
    dArrMotorSubP00[1] = dMotor2 - p00[1];
    dArrMotorSubP00[2] = dMotor3 - p00[2];
    memcpy(Pm_inv, Pmatrix, sizeof(double)*9);
    int nRes = GeometryUtils::MatrixInv(Pm_inv, 3);
    if(0 == nRes){
        cout << "error log(func, line, info): " << __func__ << __LINE__ << endl;
        cout << "Pmatrix is singular: " << endl;
        cout << "|" << Pmatrix[0]  << "\t" << Pmatrix[1]  << "\t" << Pmatrix[2]  << "|\n"
             << "|" << Pmatrix[3]  << "\t" << Pmatrix[4]  << "\t" << Pmatrix[5]  << "|\n"
             << "|" << Pmatrix[6]  << "\t" << Pmatrix[7]  << "\t" << Pmatrix[8]  << "|\n" << endl;
        return;
    }

    double arrdEuler[3] = {0};
    matrix3x1Multiply(Pm_inv, dArrMotorSubP00, arrdEuler);
    dPitch = arrdEuler[0];
    dYaw   = arrdEuler[1];
    dClip  = arrdEuler[2];

//#define dbg_M2PY
#ifdef dbg_M2PY
    cout << __func__ << "\t" << "Pmatrix = " << endl;
    cout << "|" << Pmatrix[0]  << "\t" << Pmatrix[1]  << "\t" << Pmatrix[2]  << "|\n"
         << "|" << Pmatrix[3]  << "\t" << Pmatrix[4]  << "\t" << Pmatrix[5]  << "|\n"
         << "|" << Pmatrix[6]  << "\t" << Pmatrix[7]  << "\t" << Pmatrix[8]  << "|\n" << endl;

    cout << __func__ << "\t" << "Pm_inv = " << endl;
    cout << "|" << Pm_inv[0]  << "\t" << Pm_inv[1]  << "\t" << Pm_inv[2]  << "|\n"
         << "|" << Pm_inv[3]  << "\t" << Pm_inv[4]  << "\t" << Pm_inv[5]  << "|\n"
         << "|" << Pm_inv[6]  << "\t" << Pm_inv[7]  << "\t" << Pm_inv[8]  << "|\n" << endl;

    cout << __func__ << "\t" << "(dPitch, dYaw, dClip) = " << endl;
    cout << dPitch << "\t" << dYaw << "\t" << dClip << endl;
#endif
}
void Algorithm::MEGA_MotorsToPY(const double motor1, const double motor2, const double motor3,
    double& yaw, double& pitch,  double& clip)
{
    //#define dbg_M2PY
    double p00[3] = {-0.3055, -2.203, -1.469};
    double p10[3] = { 0.9915,  0.6312, 0.6346};
    double p01[3] = { -0.000357,       1.33,   1.325};
    double pclip[3] = {0.0, p01[1]/2, -p01[2]/2};

    double Pmatrix[3][3] = {
            p10[0],p01[0],pclip[0],
            p10[1],p01[1],pclip[1],
            p10[2],p01[2],pclip[2]
    };

    double dArrMotorSubP00[3] = {0};
    dArrMotorSubP00[0] = motor1 - p00[0];
    dArrMotorSubP00[1] = motor2 - p00[1];
    dArrMotorSubP00[2] = motor3 - p00[2];
    double* Pm_inv = GeometryUtils::MatrixInv((double*)Pmatrix, 3, 3);

    pitch = Pm_inv[0]*dArrMotorSubP00[0] + Pm_inv[1]*dArrMotorSubP00[1] + Pm_inv[2]*dArrMotorSubP00[2];
    yaw   = Pm_inv[3]*dArrMotorSubP00[0] + Pm_inv[4]*dArrMotorSubP00[1] + Pm_inv[5]*dArrMotorSubP00[2];
    clip  = Pm_inv[6]*dArrMotorSubP00[0] + Pm_inv[7]*dArrMotorSubP00[1] + Pm_inv[8]*dArrMotorSubP00[2];

#ifdef dbg_M2PY
    /*
    cout << __func__ << "\t" << "Pmatrix = " << endl;
    cout << "|" << Pmatrix[0][0]  << "\t" << Pmatrix[0][1]  << "\t" << Pmatrix[0][2]  << "|\n"
         << "|" << Pmatrix[1][0]  << "\t" << Pmatrix[1][1]  << "\t" << Pmatrix[1][2]  << "|\n"
         << "|" << Pmatrix[2][0]  << "\t" << Pmatrix[2][1]  << "\t" << Pmatrix[2][2]  << "|\n" << endl;
    cout << __func__ << "\t" << "Pm_inv = " << endl;
    cout << "|" << Pm_inv[0]  << "\t" << Pm_inv[1]  << "\t" << Pm_inv[2]  << "|\n"
         << "|" << Pm_inv[3]  << "\t" << Pm_inv[4]  << "\t" << Pm_inv[5]  << "|\n"
         << "|" << Pm_inv[6]  << "\t" << Pm_inv[7]  << "\t" << Pm_inv[8]  << "|\n" << endl;

    cout << __func__ << "\t" << "(pitch, yaw, clip) = " << endl;
    cout << pitch << "\t" << yaw << "\t" << clip << endl;
    */
#endif
    delete[] Pm_inv;
}

/* BEGIN: Added by Linl, 2017/3/17 */

void Algorithm::MARYLAND_MotorsToPY(const double motor1, const double motor2, const double motor3,
    double& yaw, double& pitch,  double& clip)
{
    //#define dbg_M2PY
    double p00 = 0.5248;
    double p10 = 0.6683;
    double p01 = 1.114;

    double Pmatrix[3][3] = {
            1, 0, 0,
            p10, p01, p01/2,
            p10, p01, -p01/2
    };

    double dArrMotorSubP00[3] = {0};
    dArrMotorSubP00[0] = motor1;
    dArrMotorSubP00[1] = motor2 - p00;
    dArrMotorSubP00[2] = motor3 - p00;
    double* Pm_inv = GeometryUtils::MatrixInv((double*)Pmatrix, 3, 3);

    pitch = Pm_inv[0]*dArrMotorSubP00[0] + Pm_inv[1]*dArrMotorSubP00[1] + Pm_inv[2]*dArrMotorSubP00[2];
    yaw   = Pm_inv[3]*dArrMotorSubP00[0] + Pm_inv[4]*dArrMotorSubP00[1] + Pm_inv[5]*dArrMotorSubP00[2];
    clip  = Pm_inv[6]*dArrMotorSubP00[0] + Pm_inv[7]*dArrMotorSubP00[1] + Pm_inv[8]*dArrMotorSubP00[2];

    delete[] Pm_inv;
}
/* END:   Added by Linl, 2017/3/17 */

void Algorithm::NewSurgnova_PYToMotors(const double yaw, const double pitch, const double clip,
    double& motor1, double& motor2, double& motor3, const double gripDegree)
{
    const double PitchDrivenDiameter  = 5.35;
    const double PitchDrivingDiameter = 5.55;
    const double Clip1DrivenDiameter  = 7.47;
    const double Clip1DrivingDiameter = 5.55;
    const double Clip2DrivenDiameter  = 7.47;
    const double Clip2DrivingDiameter = 5.55;
    const double PulleyDiameter = 3.45;

    // Construct transform matrix using above measured parameters
    double TransformMatrix[3][3];

    TransformMatrix[0][0] = - PitchDrivenDiameter / PitchDrivingDiameter;
    TransformMatrix[0][1] = 0.0;
    TransformMatrix[0][2] = 0.0;

    TransformMatrix[1][0] = - PulleyDiameter / Clip1DrivenDiameter;
    TransformMatrix[1][1] =   Clip1DrivenDiameter / Clip1DrivingDiameter;
    TransformMatrix[1][2] =   Clip1DrivenDiameter / (2*Clip1DrivingDiameter);

    TransformMatrix[2][0] =   PulleyDiameter / Clip2DrivenDiameter;
    TransformMatrix[2][1] = - Clip2DrivenDiameter / Clip2DrivingDiameter;
    TransformMatrix[2][2] =   Clip2DrivenDiameter / (2*Clip2DrivingDiameter);

    //Debug print
    /*
    int i, j;
    cout <<"Surgnova_PYToMotors matrix:" <<endl;
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            cout << TransformMatrix[i][j] << ",";
        }
        cout << endl;
    }
    */
    double new_clip = clip;
    /*
    if (fabs(yaw) + new_clip / 2 > SURGNOVA_PYTOMOTORS_CLIP_MAX)
    {
        new_clip = (SURGNOVA_PYTOMOTORS_CLIP_MAX - fabs(yaw)) * 2;
    }

    if ( new_clip < - gripDegree)
    {
        new_clip = - gripDegree;
    }
    if ( new_clip < gripDegree && new_clip >= - gripDegree)
    {
        new_clip = new_clip * 2.0 - gripDegree;
    }
    */
    //DBG
    //cout << ", updated clip=" << new_clip << endl;
    //Matrix dot
    motor1 = - TransformMatrix[0][0]*pitch + TransformMatrix[0][1]*yaw + TransformMatrix[0][2]*new_clip;
    motor2 = - TransformMatrix[1][0]*pitch - TransformMatrix[1][1]*yaw + TransformMatrix[1][2]*new_clip;
    motor3 = - TransformMatrix[2][0]*pitch - TransformMatrix[2][1]*yaw + TransformMatrix[2][2]*new_clip;
}

void Algorithm::NewSurgnova_PYToMotors_velocity(const double yaw, const double pitch, const double clip,
    double& motor1, double& motor2, double& motor3)
{
    const double PitchDrivenDiameter  = 5.35;
    const double PitchDrivingDiameter = 5.55;
    const double Clip1DrivenDiameter  = 7.47;
    const double Clip1DrivingDiameter = 5.55;
    const double Clip2DrivenDiameter  = 7.47;
    const double Clip2DrivingDiameter = 5.55;
    const double PulleyDiameter = 3.45;

    // Construct transform matrix using above measured parameters
    double TransformMatrix[3][3];

    TransformMatrix[0][0] = - PitchDrivenDiameter / PitchDrivingDiameter;
    TransformMatrix[0][1] = 0.0;
    TransformMatrix[0][2] = 0.0;

    TransformMatrix[1][0] = - PulleyDiameter / Clip1DrivenDiameter;
    TransformMatrix[1][1] =   Clip1DrivenDiameter / Clip1DrivingDiameter;
    TransformMatrix[1][2] =   Clip1DrivenDiameter / (2*Clip1DrivingDiameter);

    TransformMatrix[2][0] =   PulleyDiameter / Clip2DrivenDiameter;
    TransformMatrix[2][1] = - Clip2DrivenDiameter / Clip2DrivingDiameter;
    TransformMatrix[2][2] =   Clip2DrivenDiameter / (2*Clip2DrivingDiameter);

    //Matrix dot
    motor1 = - TransformMatrix[0][0]*pitch + TransformMatrix[0][1]*yaw + TransformMatrix[0][2]*clip;
    motor2 = - TransformMatrix[1][0]*pitch - TransformMatrix[1][1]*yaw + TransformMatrix[1][2]*clip;
    motor3 = - TransformMatrix[2][0]*pitch - TransformMatrix[2][1]*yaw + TransformMatrix[2][2]*clip;
}

void Algorithm::Surgnova_PYToMotors(const double yaw, const double pitch, const double clip,
    double& motor1, double& motor2, double& motor3, const double gripDegree)
{
    const double PitchDrivenDiameter  = 5.35;
    const double PitchDrivingDiameter = 7.10;
    const double Clip1DrivenDiameter  = 7.47;
    const double Clip1DrivingDiameter = 7.10;
    const double Clip2DrivenDiameter  = 7.47;
    const double Clip2DrivingDiameter = 7.10;
    const double PulleyDiameter = 3.45;

    // Construct transform matrix using above measured parameters
    double TransformMatrix[3][3];

    TransformMatrix[0][0] = 0.0;
    TransformMatrix[0][1] = - PitchDrivenDiameter / PitchDrivingDiameter;
    TransformMatrix[0][2] = 0.0;

    TransformMatrix[1][0] = Clip1DrivenDiameter / Clip1DrivingDiameter;
    TransformMatrix[1][1] = - PulleyDiameter / Clip1DrivenDiameter;
    TransformMatrix[1][2] = Clip1DrivenDiameter / (2*Clip1DrivingDiameter);

    TransformMatrix[2][0] = - Clip2DrivenDiameter / Clip2DrivingDiameter;
    TransformMatrix[2][1] = PulleyDiameter / Clip2DrivenDiameter;
    TransformMatrix[2][2] = Clip2DrivenDiameter / (2*Clip2DrivingDiameter);

    //Debug print
    /*
    int i, j;
    cout <<"Surgnova_PYToMotors matrix:" <<endl;
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            cout << TransformMatrix[i][j] << ",";
        }
        cout << endl;
    }
    */

    double new_clip = clip;
    if (fabs(yaw) + new_clip / 2 > SURGNOVA_PYTOMOTORS_CLIP_MAX)
    {
        new_clip = (SURGNOVA_PYTOMOTORS_CLIP_MAX - fabs(yaw)) * 2;
    }

    if ( new_clip < - gripDegree)
    {
        new_clip = - gripDegree;
    }
    if ( new_clip < gripDegree && new_clip >= - gripDegree)
    {
        //new_clip = (((double)SURGNOVA_PYTOMOTORS_CLIP_CRI - (double)SURGNOVA_PYTOMOTORS_CLIP_MIN) * new_clip) / \
        //         (double)SURGNOVA_PYTOMOTORS_CLIP_CRI + (double)SURGNOVA_PYTOMOTORS_CLIP_MIN;
        new_clip = new_clip * 2.0 - gripDegree;
    }
    //DBG
    //cout << ", updated clip=" << new_clip << endl;
    //Matrix dot
    motor1 = TransformMatrix[0][0]*yaw + TransformMatrix[0][1]*pitch + TransformMatrix[0][2]*new_clip;
    motor2 = TransformMatrix[1][0]*yaw + TransformMatrix[1][1]*pitch + TransformMatrix[1][2]*new_clip;
    motor3 = TransformMatrix[2][0]*yaw + TransformMatrix[2][1]*pitch + TransformMatrix[2][2]*new_clip;
}

void Algorithm::Surgnova_MotorsToPY(const double motor1, const double motor2, const double motor3,
    double& yaw, double& pitch, double& clip)
{

    const double PitchDrivenDiameter  = 5.35;
    const double PitchDrivingDiameter = 7.10;
    const double Clip1DrivenDiameter  = 7.47;
    const double Clip1DrivingDiameter = 7.10;
    const double Clip2DrivenDiameter  = 7.47;
    const double Clip2DrivingDiameter = 7.10;
    const double PulleyDiameter = 3.45;

    // Construct transform matrix using above measured parameters
    double TransformMatrix[3][3];

    TransformMatrix[0][0] = - (PulleyDiameter / Clip2DrivenDiameter)*(PitchDrivingDiameter / \
                             PitchDrivenDiameter)*(Clip1DrivingDiameter / Clip1DrivenDiameter);
    TransformMatrix[0][1] = Clip1DrivingDiameter / (2*Clip1DrivenDiameter);
    TransformMatrix[0][2] = - Clip1DrivingDiameter / (2*Clip1DrivenDiameter);

    TransformMatrix[1][0] = - PitchDrivingDiameter / PitchDrivenDiameter;
    TransformMatrix[1][1] = 0.0;
    TransformMatrix[1][2] = 0.0;

    TransformMatrix[2][0] = 0.0;
    TransformMatrix[2][1] = Clip1DrivingDiameter / Clip1DrivenDiameter;
    TransformMatrix[2][2] = Clip1DrivingDiameter / Clip1DrivenDiameter;

    //Debug print
    /*
    int i, j;
    cout <<"Surgnova_MotorsToPY inv matrix:" <<endl;
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            cout << TransformMatrix[i][j] << ",";
        }
        cout << endl;
    }
    */

    yaw   = TransformMatrix[0][0]*motor1 + TransformMatrix[0][1]*motor2 + TransformMatrix[0][2]*motor3;
    pitch = TransformMatrix[1][0]*motor1 + TransformMatrix[1][1]*motor2 + TransformMatrix[1][2]*motor3;
    clip  = TransformMatrix[2][0]*motor1 + TransformMatrix[2][1]*motor2 + TransformMatrix[2][2]*motor3;
}

void Algorithm::Surgnova_PYToMotors_velocity(const double yaw, const double pitch, const double clip,
        double& motor1, double& motor2, double& motor3)
{
    const double PitchDrivenDiameter  = 5.35;
    const double PitchDrivingDiameter = 7.10;
    const double Clip1DrivenDiameter  = 7.47;
    const double Clip1DrivingDiameter = 7.10;
    const double Clip2DrivenDiameter  = 7.47;
    const double Clip2DrivingDiameter = 7.10;
    const double PulleyDiameter = 3.45;

    // Construct transform matrix using above measured parameters
    double TransformMatrix[3][3];

    TransformMatrix[0][0] = 0.0;
    TransformMatrix[0][1] = - PitchDrivenDiameter / PitchDrivingDiameter;
    TransformMatrix[0][2] = 0.0;

    TransformMatrix[1][0] = Clip1DrivenDiameter / Clip1DrivingDiameter;
    TransformMatrix[1][1] = - PulleyDiameter / Clip1DrivenDiameter;
    TransformMatrix[1][2] = Clip1DrivenDiameter / (2*Clip1DrivingDiameter);

    TransformMatrix[2][0] = - Clip2DrivenDiameter / Clip2DrivingDiameter;
    TransformMatrix[2][1] = PulleyDiameter / Clip2DrivenDiameter;
    TransformMatrix[2][2] = Clip2DrivenDiameter / (2*Clip2DrivingDiameter);

    motor1 = TransformMatrix[0][0]*yaw + TransformMatrix[0][1]*pitch + TransformMatrix[0][2]*clip;
    motor2 = TransformMatrix[1][0]*yaw + TransformMatrix[1][1]*pitch + TransformMatrix[1][2]*clip;
    motor3 = TransformMatrix[2][0]*yaw + TransformMatrix[2][1]*pitch + TransformMatrix[2][2]*clip;
}

void Algorithm::NewSurgnova_MotorsToPY(const double motor1, const double motor2, const double motor3,
    double& yaw, double& pitch, double& clip)
{

    const double PitchDrivenDiameter  = 5.35;
    const double PitchDrivingDiameter = 5.55;
    const double Clip1DrivenDiameter  = 7.47;
    const double Clip1DrivingDiameter = 5.55;
    const double Clip2DrivenDiameter  = 7.47;
    const double Clip2DrivingDiameter = 5.55;
    const double PulleyDiameter = 3.45;

    // Construct transform matrix using above measured parameters
    double TransformMatrix[3][3];

    TransformMatrix[0][0] = - PitchDrivingDiameter / PitchDrivenDiameter;
    TransformMatrix[0][1] = 0.0;
    TransformMatrix[0][2] = 0.0;

    TransformMatrix[1][0] = - (PulleyDiameter / Clip2DrivenDiameter)*(PitchDrivingDiameter / \
                             PitchDrivenDiameter)*(Clip1DrivingDiameter / Clip1DrivenDiameter);
    TransformMatrix[1][1] =   Clip1DrivingDiameter / (2*Clip1DrivenDiameter);
    TransformMatrix[1][2] = - Clip1DrivingDiameter / (2*Clip1DrivenDiameter);


    TransformMatrix[2][0] = 0.0;
    TransformMatrix[2][1] =  Clip1DrivingDiameter / Clip1DrivenDiameter;
    TransformMatrix[2][2] =  Clip1DrivingDiameter / Clip1DrivenDiameter;

    //Debug print
    /*
    int i, j;
    cout <<"Surgnova_MotorsToPY inv matrix:" <<endl;
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            cout << TransformMatrix[i][j] << ",";
        }
        cout << endl;
    }
    */

    pitch = - TransformMatrix[0][0]*motor1 + TransformMatrix[0][1]*motor2 + TransformMatrix[0][2]*motor3;
    yaw   = - TransformMatrix[1][0]*motor1 - TransformMatrix[1][1]*motor2 - TransformMatrix[1][2]*motor3;
    clip  =   TransformMatrix[2][0]*motor1 + TransformMatrix[2][1]*motor2 + TransformMatrix[2][2]*motor3;
}

void Algorithm::CartesianVelocityToJointVelocity(Eigen::Matrix4d m1, Eigen::Matrix4d m2, double JointVelocity[])
{
    Eigen::Vector3d v(m2.coeff(0, 3) - m1.coeff(0, 3),
                      m2.coeff(1, 3) - m1.coeff(1, 3),
                      m2.coeff(2, 3) - m1.coeff(2, 3));

    Eigen::Matrix3d R0 = m1.block<3, 3>(0, 0);
    Eigen::Matrix3d R1 = m2.block<3, 3>(0, 0);
    Eigen::Matrix3d S = R1 * R0.transpose() - Eigen::Matrix3d::Identity();

    Eigen::Vector3d w(S.coeff(2, 1) - S.coeff(1, 2),
                      S.coeff(0, 2) - S.coeff(2, 0),
                      S.coeff(1, 0) - S.coeff(0, 1));

    JointVelocity[0] = v(0);
    JointVelocity[1] = v(1);
    JointVelocity[2] = v(2);
    JointVelocity[3] = w(0) * 0.5;
    JointVelocity[4] = w(1) * 0.5;
    JointVelocity[5] = w(2) * 0.5;
}
/* BEGIN: Added by Linl, 2017/3~4 */
/*****************************************************************************
 Prototype    : Algorithm::traj
 Description  : Trajectory Planning
 Input        : double t :sample time. (unit: ms)
                double ua:starting position(x,y or z)(unit: degree)
                double ub:terminal postion  (unit: degree)
                double va:starting velocity(x',y' or z')
                double vb:terminal velocity
                double ta:starting time  (unit: ms)
                double tb:terminal time  (unit: ms)
 Output       : None
 Return Value : the angle of next sample time.(unit: degree)
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/3/17
    Author       : lin.lin
    Modification : Created function
 note: the algorithm gives independent trajectory planning seperately.
*****************************************************************************/
double Algorithm::traj(double t, double ua, double ub, double va, double vb, double ta, double tb)
{
    double a0, a1, a2, a3;
    a0 = ua;
    a1 = va;
    a2 = 3.0 * (ub - ua) / pow((tb - ta), 2) - 2.0 * va / (tb - ta) - vb / (tb - ta);
    a3 = 2.0 * (ua - ub) / pow((tb - ta), 3) + (va + vb) / pow((tb - ta), 2);
    return a0 + a1 * t + a2 * t*t + a3 * t*t*t;
}

double Algorithm::traj_cubic_a0(double ua, double ub, double va, double vb, double ta, double tb)
{
  return ua;
}

double Algorithm::traj_cubic_a1(double ua, double ub, double va, double vb, double ta, double tb)
{
  return va;
}

double Algorithm::traj_cubic_a2(double ua, double ub, double va, double vb, double ta, double tb)
{
  double a2 = 3.0 * (ub - ua) / pow((tb - ta), 2) - 2.0 * va / (tb - ta) - vb / (tb - ta);
  return a2;
}

double Algorithm::traj_cubic_a3(double ua, double ub, double va, double vb, double ta, double tb)
{
  double a3 = 2.0 * (ua - ub) / pow((tb - ta), 3) + (va + vb) / pow((tb - ta), 2);
  return a3;
}

double Algorithm::traj_cubic_u(double t, double a0, double a1, double a2, double a3)
{
  return a0 + a1 * t + a2 * t*t + a3 * t*t*t;
}

double Algorithm::traj_cubic_v(double t, double a1, double a2, double a3)
{
  return a1 + 2 * a2 * t + 3 * a3 * t*t;
}

Eigen::Vector3d Algorithm::traj_cubic_a0(Eigen::Vector3d ua, Eigen::Vector3d ub, Eigen::Vector3d va, Eigen::Vector3d vb, double ta, double tb)
{
  return ua;
}

Eigen::Vector3d Algorithm::traj_cubic_a1(Eigen::Vector3d ua, Eigen::Vector3d ub, Eigen::Vector3d va, Eigen::Vector3d vb, double ta, double tb)
{
  return va;
}

Eigen::Vector3d Algorithm::traj_cubic_a2(Eigen::Vector3d ua, Eigen::Vector3d ub, Eigen::Vector3d va, Eigen::Vector3d vb, double ta, double tb)
{
  Eigen::Vector3d a2 = 3.0 * (ub - ua) / pow((tb - ta), 2) - 2.0 * va / (tb - ta) - vb / (tb - ta);
  return a2;
}

Eigen::Vector3d Algorithm::traj_cubic_a3(Eigen::Vector3d ua, Eigen::Vector3d ub, Eigen::Vector3d va, Eigen::Vector3d vb, double ta, double tb)
{
  Eigen::Vector3d a3 = 2.0 * (ua - ub) / pow((tb - ta), 3) + (va + vb) / pow((tb - ta), 2);
  return a3;
}

Eigen::Vector3d Algorithm::traj_cubic_u(double t, Eigen::Vector3d a0, Eigen::Vector3d a1, Eigen::Vector3d a2, Eigen::Vector3d a3)
{
  return a0 + a1 * t + a2 * t*t + a3 * t*t*t;
}

Eigen::Vector3d Algorithm::traj_cubic_v(double t, Eigen::Vector3d a1, Eigen::Vector3d a2, Eigen::Vector3d a3)
{
  return a1 + 2 * a2 * t + 3 * a3 * t*t;
}

Eigen::Vector3d Algorithm::traj_cubic_a(double t, Eigen::Vector3d a2, Eigen::Vector3d a3)
{
  return 2 * a2 + 6 * a3 * t;
}

Eigen::Matrix<double, 7, 1> Algorithm::traj_cubic_a0(Eigen::Matrix<double, 7, 1> ua, Eigen::Matrix<double, 7, 1> ub, Eigen::Matrix<double, 7, 1> va, Eigen::Matrix<double, 7, 1> vb, double ta, double tb)
{
  return ua;
}

Eigen::Matrix<double, 7, 1> Algorithm::traj_cubic_a1(Eigen::Matrix<double, 7, 1> ua, Eigen::Matrix<double, 7, 1> ub, Eigen::Matrix<double, 7, 1> va, Eigen::Matrix<double, 7, 1> vb, double ta, double tb)
{
  return va;
}

Eigen::Matrix<double, 7, 1> Algorithm::traj_cubic_a2(Eigen::Matrix<double, 7, 1> ua, Eigen::Matrix<double, 7, 1> ub, Eigen::Matrix<double, 7, 1> va, Eigen::Matrix<double, 7, 1> vb, double ta, double tb)
{
  Eigen::Matrix<double, 7, 1> a2 = 3.0 * (ub - ua) / pow((tb - ta), 2) - 2.0 * va / (tb - ta) - vb / (tb - ta);
  return a2;
}

Eigen::Matrix<double, 7, 1> Algorithm::traj_cubic_a3(Eigen::Matrix<double, 7, 1> ua, Eigen::Matrix<double, 7, 1> ub, Eigen::Matrix<double, 7, 1> va, Eigen::Matrix<double, 7, 1> vb, double ta, double tb)
{
  Eigen::Matrix<double, 7, 1> a3 = 2.0 * (ua - ub) / pow((tb - ta), 3) + (va + vb) / pow((tb - ta), 2);
  return a3;
}

Eigen::Matrix<double, 7, 1> Algorithm::traj_cubic_u(double t, Eigen::Matrix<double, 7, 1> a0, Eigen::Matrix<double, 7, 1> a1, Eigen::Matrix<double, 7, 1> a2, Eigen::Matrix<double, 7, 1> a3)
{
  return a0 + a1 * t + a2 * t*t + a3 * t*t*t;
}

Eigen::Matrix<double, 7, 1> Algorithm::traj_cubic_v(double t, Eigen::Matrix<double, 7, 1> a1, Eigen::Matrix<double, 7, 1> a2, Eigen::Matrix<double, 7, 1> a3)
{
  return a1 + 2 * a2 * t + 3 * a3 * t*t;
}

Eigen::Matrix<double, 7, 1> Algorithm::traj_cubic_a(double t, Eigen::Matrix<double, 7, 1> a2, Eigen::Matrix<double, 7, 1> a3)
{
  return 2 * a2 + 6 * a3 * t;
}

Eigen::Matrix<double, 10, 1> Algorithm::traj_cubic_a0(Eigen::Matrix<double, 10, 1> ua, Eigen::Matrix<double, 10, 1> ub, Eigen::Matrix<double, 10, 1> va, Eigen::Matrix<double, 10, 1> vb, double ta, double tb)
{
  return ua;
}

Eigen::Matrix<double, 10, 1> Algorithm::traj_cubic_a1(Eigen::Matrix<double, 10, 1> ua, Eigen::Matrix<double, 10, 1> ub, Eigen::Matrix<double, 10, 1> va, Eigen::Matrix<double, 10, 1> vb, double ta, double tb)
{
  return va;
}

Eigen::Matrix<double, 10, 1> Algorithm::traj_cubic_a2(Eigen::Matrix<double, 10, 1> ua, Eigen::Matrix<double, 10, 1> ub, Eigen::Matrix<double, 10, 1> va, Eigen::Matrix<double, 10, 1> vb, double ta, double tb)
{
  Eigen::Matrix<double, 10, 1> a2 = 3.0 * (ub - ua) / pow((tb - ta), 2) - 2.0 * va / (tb - ta) - vb / (tb - ta);
  return a2;
}

Eigen::Matrix<double, 10, 1> Algorithm::traj_cubic_a3(Eigen::Matrix<double, 10, 1> ua, Eigen::Matrix<double, 10, 1> ub, Eigen::Matrix<double, 10, 1> va, Eigen::Matrix<double, 10, 1> vb, double ta, double tb)
{
  Eigen::Matrix<double, 10, 1> a3 = 2.0 * (ua - ub) / pow((tb - ta), 3) + (va + vb) / pow((tb - ta), 2);
  return a3;
}

Eigen::Matrix<double, 10, 1> Algorithm::traj_cubic_u(double t, Eigen::Matrix<double, 10, 1> a0, Eigen::Matrix<double, 10, 1> a1, Eigen::Matrix<double, 10, 1> a2, Eigen::Matrix<double, 10, 1> a3)
{
  return a0 + a1 * t + a2 * t*t + a3 * t*t*t;
}

Eigen::Matrix<double, 10, 1> Algorithm::traj_cubic_v(double t, Eigen::Matrix<double, 10, 1> a1, Eigen::Matrix<double, 10, 1> a2, Eigen::Matrix<double, 10, 1> a3)
{
  return a1 + 2 * a2 * t + 3 * a3 * t*t;
}

Eigen::Matrix<double, 10, 1> Algorithm::traj_cubic_a(double t, Eigen::Matrix<double, 10, 1> a2, Eigen::Matrix<double, 10, 1> a3)
{
  return 2 * a2 + 6 * a3 * t;
}

Eigen::Matrix<double, 7, 6> Algorithm::pinv(Eigen::Matrix<double, 6, 7> mtrxSrc)
{
  Eigen::Matrix<double, 7, 6> mtrxSrcT;
  mtrxSrcT = mtrxSrc.transpose();
  return mtrxSrcT * (mtrxSrc * mtrxSrcT).inverse();
}
/*****************************************************************************
 Prototype    : Algorithm.pinv
 Description  : get right pseudo-inverse matrix
 Input        : const double* tarrM: the source matrix (iRow*iCol)
                const int iRow: the row number of matrix tarrM
                const int iCol: the column number of matrix tarrM
 Output       : double* tarrPinvM  : return right pseudo-inverse matrix

 Return Value : int: 0 - failed; 1- success.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2019/12/06
    Author       : lin.lin
    Modification : Created function
  note: 1, A.7 Pseudo-inverse, Bruno Siciliano;
        2, see Algorithm::getWeightRightPinv
*****************************************************************************/
int Algorithm::pinv(const double* tarrM, const int iRow, const int iCol, double* tarrPinvM)
{
    // initial value
    typedef double T;
    const int iRowMaxNum                       = 16;
    const int iColMaxNum                       = 16;
    T tarrMT[iColMaxNum*iRowMaxNum]            = {0};       // transpose of tarrM: tarrM'
    T tarrMMT[iRowMaxNum*iRowMaxNum]           = {0};       // M*M'

    T tarrMW[iRowMaxNum*iColMaxNum]            = {0};
    T tarrMWT[iColMaxNum*iRowMaxNum]           = {0};
    int iRet = 0;

    // get Transpose M'
    GeometryUtils::MatrixTranspose(tarrM, iRow,  iCol, tarrMT);
    // get M*M'
    GeometryUtils::MatrixMultiply(tarrM, tarrMT, iRow, iCol, iRow, tarrMMT);
    // get inv(M * M')
    iRet = GeometryUtils::MatrixInv(tarrMMT, iRow);
    if (0 == iRet)
      return iRet;
    // get pinv (M' * inv(M * M'))
    GeometryUtils::MatrixMultiply(tarrMT, tarrMMT, iCol, iRow, iRow, tarrPinvM);
    return iRet;
}
/*****************************************************************************
 Prototype    : Algorithm.trajEx
 Description  : Trajectory Planning
 Input        : double ts            : sample time. (unit: second)
                int dimension        : dimension of position vector.
                double theta_start[] : initial position (unit: degree)
                double theta_end[]   : final position (unit: degree)
                double thetadot_mid[]: ramp up to the given specified constant velocity.
                double tf            : the ending time (unit: second), the final or total time of the movement
                double theta[]       : output param, the position (unit: degree) on the current sample time ts.
                double omega[]       : output param, the velocity on the current sample time ts.
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/3/24
    Author       : lin.lin
    Modification : Created function

note: 1��see reference Chapter 6, Trajectory Planning of "robotics lecture notes.pdf"
      2��reference ../UnitTest/MatlabScript/traj_plan.m.
*****************************************************************************/
void Algorithm::trajEx(
    double t,
    int dimension,
    double theta_start[],
    double theta_end[],
    double thetadot_mid[],
    double tf,
    double theta[],
    double omega[]
    )
{
    //a transition time, with which we ramp up to the given specified constant velocity
    double tb[2];
    //accelaration
    double a[2];

    int nStep = 0;
    //double t = ts; //Sample time
    int index = 0;
    //int theta_start_len = dimension;
    // initialization of theta trajectory

    double velocitymin_flag, velocitymax_flag;

    //assertion.
    for (index=0; index<dimension; index++)
    {
        velocitymin_flag = thetadot_mid[index] / (  (theta_end[index] - theta_start[index]) / tf);
        velocitymax_flag = thetadot_mid[index] / (2*(theta_end[index] - theta_start[index]) / tf);
        if(velocitymin_flag<1)
        {
            printf("Error! midpoint velocity too small\n");
            return;
        }else if(velocitymax_flag>1)
        {
            printf("Error! midpoint velocity too large\n");
            return;
        }
    }
    for (index=0; index<dimension; index++)
    {
        //tb = (theta_start - theta_end + thetadot_mid*tf)./thetadot_mid;
        //a = thetadot_mid./tb;
        tb[index] = (theta_start[index] - theta_end[index] + thetadot_mid[index]*tf)/thetadot_mid[index];
        a[index] = thetadot_mid[index]/tb[index];

        //iteration
//        for(nStep=0; nStep<nsteps; nStep++)
//        {
            //t = nStep * tf / nsteps;
        if(t<=tb[index])
        {
            theta[index] = theta_start[index] + 0.5 * a[index] * t*t;
            omega[index] = a[index] * t;
        }else if ((t > tb[index]) && (t < (tf - tb[index])))
        {
            theta[index] = (theta_start[index] + theta_end[index] - thetadot_mid[index]*tf)/2 + thetadot_mid[index]*t;
            omega[index] = thetadot_mid[index];
        }else
        {
            theta[index] = theta_end[index] - 0.5 * a[index] * tf*tf + a[index]*tf*t - 0.5*a[index]*t*t;
            omega[index] = a[index]*tf - a[index]*t;
        }
    }
}

/*****************************************************************************
 Prototype    : axis2rot
 Description  : axis2rot computes the rotation matrix corresponding to a rotation
                about axis k by angle theta
 Input        : theata     : angle of rotation about axis k
                k[]        : a column vector

 Output       : rotMatrix[]: output 3*3 rotation matrix
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/4/10
    Author       : lin.lin
    Modification : Created function
note:
function R = axis2rot( theta, k )

v = 1 - cos(theta);
c = cos(theta);
s = sin(theta);

R = [k(1)^2 * v+c           k(1)*k(2)*v-k(3)*s      k(1)*k(3)*v+k(2)*s;
     k(1)*k(2)*v+k(3)*s     k(2)^2 * v+c            k(2)*k(3)*v-k(1)*s;
     k(1)*k(3)*v-k(2)*s     k(2)*k(3)*v+k(1)*s      k(3)^2 * v+c];
end
*****************************************************************************/
void Algorithm::axis2rot(const double theta, const double pK[], double pRotMatrix[])
{
    double v = 1.0 - cos(theta);
    double c = cos(theta);
    double s = sin(theta);

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

/*****************************************************************************
 Prototype    : rot2axis
 Description  : extracts the angle theta and axis k from a given matrix R.
 Input        : rotMatrix[]  : input 3x3 rotation matrix
 Output       :
                theata       : angle of rotation
                k[]          : axis of rotation
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/4/10
    Author       : lin.lin
    Modification : Created function

note:
function [theta,k] = rot2axis(R)

dR     = [R(3,2)-R(2,3);
          R(1,3)-R(3,1);
          R(2,1)-R(1,2)];

ctheta = (trace(R) - 1.0) / 2.0;

% Choose positive root for sin(theta).
stheta = 0.5 * sqrt(dR(1)^2 + dR(2)^2 + dR(3)^2);

theta  = atan2(stheta, ctheta);

if abs(stheta)>eps
    k = dR / (2 * stheta);
else
    k = zeros(3,1);
end
*****************************************************************************/
void Algorithm::rot2axis(double rotMatrix[], double& theta, double k[])
{
    const double AXIS_EPS =  2.2204e-16;
    double dR[3];
    double traceR = rotMatrix[0] + rotMatrix[4] + rotMatrix[8];

    dR[0] = rotMatrix[7] - rotMatrix[5];
    dR[1] = rotMatrix[2] - rotMatrix[6];
    dR[2] = rotMatrix[3] - rotMatrix[1];

    double ctheta = (traceR - 1.0) / 2.0;
    //Choose positive root for sin(theta).
    double stheta = 0.5 * sqrt(dR[0]*dR[0] + dR[1]*dR[1] + dR[2]*dR[2]);
    theta = atan2(stheta, ctheta);

    if (fabs(stheta) > AXIS_EPS)
    {
        k[0] = 0.5 * dR[0] / stheta ;
        k[1] = 0.5 * dR[1] / stheta ;
        k[2] = 0.5 * dR[2] / stheta ;
    }
    else
    {
        k[0] = 0.0 ;
        k[1] = 0.0 ;
        k[2] = 0.0 ;
    }
}

/*****************************************************************************
 Prototype    : trajplan_cartesian
 Description  : trajectory plan
 Input        : T_start[]  :4*4, starting homogenous transformation of tool frame, relative to world frame
                T_end[]    :4*4, ending homogenous transformation of tool frame, relative to world frame
                total_time :the final or total time of the movement
                t          : unit: the same with total_time
 Output       : R_t_tool   :3*3
                Pos_t_tool :3*1
 Return Value :
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/4/10
    Author       : lin.lin
    Modification : Created function

note:
function [ R_t_tool, Pos_t_tool ] = trajplan_cartesian( T_start, T_end, total_time, t )
tool_startpos = [T_start(1,4); T_start(2,4); T_start(3,4);];
tool_endpos = [T_end(1,4); T_end(2,4); T_end(3,4);];
tool_startorient = T_start(1:3, 1:3);
tool_endorient = T_end(1:3, 1:3);

Pos_t_tool = tool_startpos + t/total_time * (tool_endpos - tool_startpos);

R_totalaction =  tool_startorient' * tool_endorient;

[theta, k] = rot2axis(R_totalaction);

theta_t = theta * t / total_time;
R_t_action = axis2rot(theta_t, k);

R_t_tool = tool_startorient * R_t_action;

end
*****************************************************************************/
void Algorithm::trajplan_cartesian(double T_start[], double T_end[], double total_time, double t,
                        double R_t_tool[], double Pos_t_tool[])
{
    double tool_startpos[3] = {0};                          //start position (3*1)
    double tool_endpos[3] = {0};                            //end position (3*1)
    double tool_startorient[3][3] = {0};                    //start orientation (3*3)
    double tool_startorientT[3][3] = {0};                   //inverse (3*3)
    double tool_endorient[3][3] = {0};                      //end orientation (3*3)
    //double Pos_t_tool[3] = {0};                           //position on time t
    double R_totalaction[3][3];                             // (3*3)
    //compute Pos_t_tool
    //initial startpos and endpos
    tool_startpos[0] = T_start[3];
    tool_startpos[1] = T_start[7];
    tool_startpos[2] = T_start[11];
    tool_endpos[0] = T_end[3];
    tool_endpos[1] = T_end[7];
    tool_endpos[2] = T_end[11];
    for ( int i = 0 ; i <3 ; i++ )
    {
        for ( int j = 0 ; j <3 ; j++ )
        {
            tool_startorient[i][j] = T_start[i*4+j];
            tool_endorient[i][j] = T_end[i*4+j];
        }
        Pos_t_tool[i] = tool_startpos[i] + t /total_time * (tool_endpos[i] - tool_startpos[i]);
    }
    //compute R_t_tool
    GeometryUtils::MatrixTranspose((double*)tool_startorient, 3, 3, (double*)tool_startorientT);
    GeometryUtils::MatrixMultiply((double*)tool_startorientT, (double*)tool_endorient, 3, 3, 3, (double*)R_totalaction);

    double theta = 0.0;
    double k[3] = {0};
    rot2axis((double*)R_totalaction, theta, k);

    double theta_t = theta * t / total_time;
    //axis2rot(double theta, double k[], double rotMatrix[])
    double R_t_action[3][3] = {0};
    axis2rot(theta_t, k, (double *)R_t_action);

    GeometryUtils::MatrixMultiply((double *)tool_startorient, (double *)R_t_action, 3, 3, 3, (double *)R_t_tool);
}

void Algorithm::trajplan_cartesian(double T_start[], double T_end[], double total_time, double t,
                        Frame &frame)
{
//    double R_t_tool[3][3]   = {0};
//    double Pos_t_tool[3][1] = {0};

    double tool_startpos[3] = {0};                          //start position (3*1)
    double tool_endpos[3] = {0};                            //end position (3*1)
    double tool_startorient[3][3] = {0};                    //start orientation (3*3)
    double tool_startorientT[3][3] = {0};                   //inverse (3*3)
    double tool_endorient[3][3] = {0};                      //end orientation (3*3)
    //double Pos_t_tool[3] = {0};                           //position on time t
    double R_totalaction[3][3];                             // (3*3)
    //compute Pos_t_tool
    //initial startpos and endpos
    tool_startpos[0] = T_start[3];
    tool_startpos[1] = T_start[7];
    tool_startpos[2] = T_start[11];
    tool_endpos[0] = T_end[3];
    tool_endpos[1] = T_end[7];
    tool_endpos[2] = T_end[11];
    for ( int i = 0 ; i <3 ; i++ )
    {
        for ( int j = 0 ; j <3 ; j++ )
        {
            tool_startorient[i][j] = T_start[i*4+j];
            tool_endorient[i][j] = T_end[i*4+j];
        }
//        Pos_t_tool[i]         = tool_startpos[i] + t /total_time * (tool_endpos[i] - tool_startpos[i]);
        frame.mOrigine.mData[i] = tool_startpos[i] + t /total_time * (tool_endpos[i] - tool_startpos[i]);
    }
    //compute R_t_tool
    GeometryUtils::MatrixTranspose((double*)tool_startorient, 3, 3, (double*)tool_startorientT);
    GeometryUtils::MatrixMultiply((double*)tool_startorientT, (double*)tool_endorient, 3, 3, 3, (double*)R_totalaction);

    double theta = 0.0;
    double k[3] = {0};
    rot2axis((double*)R_totalaction, theta, k);

    double theta_t = theta * t / total_time;
    //axis2rot(double theta, double k[], double rotMatrix[])
    double R_t_action[3][3] = {0};
    axis2rot(theta_t, k, (double *)R_t_action);

    GeometryUtils::MatrixMultiply((double *)tool_startorient, (double *)R_t_action, 3, 3, 3, (double *)frame.mOrientation.mData);

//   //construct frame
//    memcpy(frame.mOrientation.mData, R_t_tool, sizeof(double)*9);
//    memcpy(frame.mOrigine.mData, Pos_t_tool, sizeof(double)*3);
}

/* END:   Added by lin.lin, 2017/4/12   PN: */
void Algorithm::IK(const Frame& clipFrame, const Vector& trocarPosition,
                   const double& shaftLength, const double& clipLength,const double& jointLength,
                    double& pitch, double& yaw, Frame& flangeFrame
                  )
{
    Vector clipV = clipFrame.mOrigine;
    Rotation clipR = clipFrame.mOrientation;

    //get clipCross position
    double* clipRMultVector = GeometryUtils::MatrixMultiply(clipR.mData,3,3,Vector(-clipLength/2,0,0).mData,3,1);
    Vector clipPoint = clipV + Vector(clipRMultVector[0],clipRMultVector[1],clipRMultVector[2]);

    //the clip is in xy surface
    //get the surface function
    double* normalOfClip = GeometryUtils::MatrixMultiply(clipR.mData,3,3,Vector(0,0,1).mData,3,1);
    double A = normalOfClip[0];
    double B = normalOfClip[1];
    double C = normalOfClip[2];
    double D = -(A * clipPoint.mData[0] + B * clipPoint.mData[1] + C * clipPoint.mData[2]);

    double t = -(A * trocarPosition.mData[0] + B * trocarPosition.mData[1]+ C * trocarPosition.mData[2] + D) / (A * A + B * B + C * C);
    Vector trocarPro(
        trocarPosition.mData[0] + A * t,
        trocarPosition.mData[1] + B * t,
        trocarPosition.mData[2] + C * t
    );

    //get the point of the end of shaft
    Vector clipPointMinusTro = clipPoint - trocarPro;
    double pro2shaftEndLength = sqrt(clipPointMinusTro.mData[0] * clipPointMinusTro.mData[0]+clipPointMinusTro.mData[1] * clipPointMinusTro.mData[1]+clipPointMinusTro.mData[2] * clipPointMinusTro.mData[2]);
    Vector shaftEndPoint;
    if(fabs(pro2shaftEndLength) <= 0.0001)
        shaftEndPoint = clipPoint + Vector(0,jointLength,0);
    else
    {
        shaftEndPoint.mData[0] = ((pro2shaftEndLength - jointLength) * (clipPoint.mData[0] - trocarPro.mData[0])) / pro2shaftEndLength + trocarPro.mData[0];
        shaftEndPoint.mData[1] = ((pro2shaftEndLength - jointLength) * (clipPoint.mData[1] - trocarPro.mData[1])) / pro2shaftEndLength + trocarPro.mData[1];
        shaftEndPoint.mData[2] = ((pro2shaftEndLength - jointLength) * (clipPoint.mData[2] - trocarPro.mData[2])) / pro2shaftEndLength + trocarPro.mData[2];
    }

    //normalization some vectors
    Vector tro2clipVector = clipPoint - shaftEndPoint;
    Vector shaftEnd2clipVector = shaftEndPoint - trocarPosition;
    Vector clip2targetVector = clipV - clipPoint;
    tro2clipVector = tro2clipVector / GeometryUtils::norm(tro2clipVector);
    shaftEnd2clipVector = shaftEnd2clipVector / GeometryUtils::norm(shaftEnd2clipVector);
    clip2targetVector = clip2targetVector / GeometryUtils::norm(clip2targetVector);

    //get yaw
    yaw = acos(GeometryUtils::vectorDot(clip2targetVector,tro2clipVector));
    Vector yawCross= GeometryUtils::vectorCross(clip2targetVector,tro2clipVector);
    double* newyMatrix = GeometryUtils::MatrixMultiply(clipR.mData,3,3,Vector(0,0,1).mData,3,1);
    Vector new_y(newyMatrix[0],newyMatrix[1],newyMatrix[2]);
    double flag2 = GeometryUtils::vectorDot(new_y,yawCross);
    if(flag2 > 0.0001)
        yaw = -yaw;
    double* rotYaw = GeometryUtils::rotateZ(-yaw);
    double* temp1 = GeometryUtils::MatrixMultiply(clipR.mData,3,3,rotYaw,3,3);

    //get pitch
    pitch = acos(GeometryUtils::vectorDot(shaftEnd2clipVector,tro2clipVector));
    Vector pitchCross = GeometryUtils::vectorCross(shaftEnd2clipVector,tro2clipVector);
    double* newxMatrix = GeometryUtils::MatrixMultiply(temp1,3,3,Vector(0,1,0).mData,3,1);
    Vector new_x(newxMatrix[0],newxMatrix[1],newxMatrix[2]);
    double flag1 = GeometryUtils::vectorDot(new_x,pitchCross);
    if(flag1 < -0.0001)
        pitch = -pitch;

    //get the frame of flange
    double* rotXhalfpi = GeometryUtils::rotateX(-PI/2);
    double* rotPitch = GeometryUtils::rotateZ(-pitch);
    double* rotZhalfpi = GeometryUtils::rotateZ(-PI/2);
    double* rotZmount = GeometryUtils::rotateZ(-TOOL_ROLL_DEGREE_TO_FLANGE * PI / 180);

    double* temp2 = GeometryUtils::MatrixMultiply(temp1,3,3,rotXhalfpi,3,3);
    double* temp3 = GeometryUtils::MatrixMultiply(temp2,3,3,rotPitch,3,3);
    double* temp4 = GeometryUtils::MatrixMultiply(temp3,3,3,rotZhalfpi,3,3);
    double* temp5 = GeometryUtils::MatrixMultiply(temp4,3,3,rotXhalfpi,3,3);
    double* temp6 = GeometryUtils::MatrixMultiply(temp5,3,3,rotZhalfpi,3,3);
    double* temp7 = GeometryUtils::MatrixMultiply(temp6,3,3,rotZmount,3,3);
    flangeFrame.mOrientation.mData[0] = temp7[0];
    flangeFrame.mOrientation.mData[1] = temp7[1];
    flangeFrame.mOrientation.mData[2] = temp7[2];
    flangeFrame.mOrientation.mData[3] = temp7[3];
    flangeFrame.mOrientation.mData[4] = temp7[4];
    flangeFrame.mOrientation.mData[5] = temp7[5];
    flangeFrame.mOrientation.mData[6] = temp7[6];
    flangeFrame.mOrientation.mData[7] = temp7[7];
    flangeFrame.mOrientation.mData[8] = temp7[8];

    //get the flange position
    Vector trocarMinuxShaftEnd = trocarPosition - shaftEndPoint;
    double insideLength= sqrt(
        trocarMinuxShaftEnd.mData[0] * trocarMinuxShaftEnd.mData[0] +
        trocarMinuxShaftEnd.mData[1] * trocarMinuxShaftEnd.mData[1] +
        trocarMinuxShaftEnd.mData[2] * trocarMinuxShaftEnd.mData[2]
    );
    double leftLength = shaftLength - insideLength;
    flangeFrame.mOrigine.mData[0] =  trocarPosition.mData[0] - (leftLength / insideLength) * (shaftEndPoint.mData[0] - trocarPosition.mData[0]);
    flangeFrame.mOrigine.mData[1] =  trocarPosition.mData[1] - (leftLength / insideLength) * (shaftEndPoint.mData[1] - trocarPosition.mData[1]);
    flangeFrame.mOrigine.mData[2] =  trocarPosition.mData[2] - (leftLength / insideLength) * (shaftEndPoint.mData[2] - trocarPosition.mData[2]);

    //delete memory
    delete[] newyMatrix;
    delete[] newxMatrix;
    delete[] normalOfClip;
    delete[] clipRMultVector;
    delete[] rotYaw;
    delete[] rotXhalfpi;
    delete[] rotPitch;
    delete[] rotZhalfpi;
    delete[] rotZmount;
    delete[] temp1;
    delete[] temp2;
    delete[] temp3;
    delete[] temp4;
    delete[] temp5;
    delete[] temp6;
    delete[] temp7;
}

/*****************************************************************************
Prototype    : InstrumentUnitTest.IK_ex
Description  :
Input        : double* clipHomo:  clip's homo-matrix (the fifth coordinate defined by Haoluo.Ning, 4*4 Homogeneous Matrix relative to kuka base coordinate system),
                                 where the original point is on the half clip length, the x axis is along the clip from original point to clip point.
              pdTrocarPoint:     trocar point in slave world coordinate system.
              shaftLength:       shaft length of instrument; The end of shaft is the pitch joint point;
              clipLength:        clip's length;
              jointLength:       palm's length; the length between pitch joint and yaw joint;
              pDH_param[in][out]:DH parameters matrix;  the units of roll, pitch and yaw in DH parameters matrix are radius;
              nDHparamRow:       the total rows of DH parameters matrix;

Output       :
              pitch:             unit:  radian
              yaw:               unit:  radian
              dHomoFlange:       the homo-matrix which describe current flange frame (relative to slave world coordinate system);
              pDataOut:          default value: 0; interface to log; use this pointer to output data to log;
Return Value : static void
Calls        :
Called By    :

History        :
1.Date         : 2017/11/13
  Author       : lin.lin
  Modification : Created function
note: new version of IK_ex; recommended!
*****************************************************************************/
void Algorithm::IK_ex(const double* clipHomo, const double* pdTrocarPoint, /*const Vector trocarPoint, */
    const double shaftLength, const double clipLength, const double jointLength, double *pDH_param, const int nDHparamRow,
    double& pitch, double& yaw, double* dHomoFlange, double& pitch2, double& yaw2, double* dHomoFlange2, double* pDataOut)
{
    double darrClipPoint[3] = {clipHomo[3], clipHomo[7], clipHomo[11]};         //clip point, at the point of 1/2 of the clip.
    double darrClipR[9] = {0.0};                            // store the rotation matrix of clipHomo

    double darrClip2YawPoint[3]    = {0.0};                 // vector from clip to yaw point in the Slave World coordinate system
    double darrYawPoint[3]         = {0.0};                 // yaw point in kuka base coordinate system;
    double darrClipPointInLocal[3] = {-clipLength/2, 0, 0}; // clip point in clip coordinate system (witch original point is in the middle of clip)

    double A = 0.0, B = 0.0, C = 0.0, D = 0.0, t = 0.0;     // the paramters which decided the plane equation formed by two clips: Ax + By + Cz + D = 0;
    double darrTrocarProject[3] = {0.0};                    // trocar projection point
    double darrYaw2TrocarProPoint[3] = {0.0};               // the vector from yaw to trocar projection point

    double dNormOfYaw2TrocarProPoint = 0.0;                 // norm(Yaw2TrocarProjectPoint)
    double darrPitchPointTemp[2][3] = {0.0};                // to store 2 solution of pitch point
    double darrPitchPoint[3] = {0.0};                       // store pitch point

    double darrPitch2YawPoint[3] = {0.0};                   // vector from pitch to yaw point (v1)
    double darrTro2PitchPoint[3] = {0.0};                   // vector from trocar to pitch point (v2)
    double darrYaw2ClipPoint[3]  = {0.0};                   // vector from yaw to clip point (v3)
    double dNormOfdarrPitch2YawPoint = 0.0;                 // norm(v1)
    double dNormOfdarrTro2PitchPoint = 0.0;                 // norm(v2)
    double dNormOfdarrYaw2ClipPoint  = 0.0;                 // norm(v3)
    double dVectorDot = 0.0;                                // v3 . v1; v1: darrPitch2YawPoint; v3: darrYaw2ClipPoint
    double darrYawCross[3] = {0.0};                         // v1 * v3
    double darrzInWorld[3] = {0, 0, 0};                     // unit vector z of the clip coordinate system in the Slave world coordinate system
    double darrzInLocal[3] = {0, 0, 1};                     // unit vector z of the clip coordinate system (witch original point is in the middle of clip)
    double flagRaw = 0.0;                                   // the sign flag of yaw
    double darrRotYaw[9] = {0.0};                           // to store current pitch rotation when clip coordinate system rotate yaw along clip's z
    double darrPitchR[9] = {0.0};                           // temp variable
    double dVectorDot2 = 0.0;                               // v2 . v1; v2: darrTro2PitchPoint
    double darrPitchCross[3] = {0.0};                       // v2 * v1: darrTro2PitchPoint * darrPitch2YawPoint
    double darryInLocal[3] = {0, 1, 0};                     // y axis direction in pitch(wrist 2) coordinate system
    double darryInWorld[3] = {0.0};                         // transform the y axis in pitch(wrist 2) coordinate system to slave world coordinate system
    double dFlagPitch = 0.0;                                // the sign flag of yaw
    double dHomoCur[16]   = {0.0};                          // T1*T2*T3*T4
    double dHomoDHInv[HOMO_MATRIX_ELEM_CNT] = {0.0};        // inv(T1234)
    double arrOriginTemp[HOMO_MATRIX_ELEM_CNT] = {0.0};     // temp homoMatrix that store the middle result while calculating T1*T2*T3*T4
    double darrTi[4][16] = {0.0};                           // store  T1, T2, T3, T4

    int i = 0, j = 0;
    double rTest = 0.0, pTest = 0.0, yTest = 0.0;

  // 1, caculate the point of yaw joint in the Slave World coordinate system
    for ( i = 0 ; i <3 ; ++i )
    {
        for ( j = 0 ; j <3 ; ++j )
        {
            darrClipR[i*3+j] = clipHomo[i*4+j];
        }
    }
    GeometryUtils::MatrixMultiply(darrClipR, darrClipPointInLocal, 3, 3, 1, darrClip2YawPoint);
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
    dNormOfdarrYaw2ClipPoint  = GeometryUtils::Distance(darrClipPoint, darrYawPoint);
    darrYaw2ClipPoint[0]  = darrClipPoint[0]  - darrYawPoint[0];
    darrYaw2ClipPoint[1]  = darrClipPoint[1]  - darrYawPoint[1];
    darrYaw2ClipPoint[2]  = darrClipPoint[2]  - darrYawPoint[2];
    darrYaw2ClipPoint[0]  /= dNormOfdarrYaw2ClipPoint;
    darrYaw2ClipPoint[1]  /= dNormOfdarrYaw2ClipPoint;
    darrYaw2ClipPoint[2]  /= dNormOfdarrYaw2ClipPoint;

    dNormOfYaw2TrocarProPoint = GeometryUtils::norm(darrYaw2TrocarProPoint);
    if(dNormOfYaw2TrocarProPoint <= ALGORITHM_EPS)//Two points (trocarProject and YawPoint) coincide
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

    //-------------------- solution 1 of pitch point: -----------------------------------------
    darrPitchPoint[0] = darrPitchPointTemp[0][0];
    darrPitchPoint[1] = darrPitchPointTemp[0][1];
    darrPitchPoint[2] = darrPitchPointTemp[0][2];
  // 4, get yaw and pitch
    for ( i = 0 ; i <3 ; ++i )
    {
        darrPitch2YawPoint[i] = darrYawPoint[i]   - darrPitchPoint[i];          //v1
        darrTro2PitchPoint[i] = darrPitchPoint[i] - pdTrocarPoint[i];           //v2

    }
    dNormOfdarrPitch2YawPoint = GeometryUtils::norm(darrPitch2YawPoint);
    dNormOfdarrTro2PitchPoint = GeometryUtils::norm(darrTro2PitchPoint);

    for ( i = 0 ; i <3 ; ++i )
    {
        darrPitch2YawPoint[i] /= dNormOfdarrPitch2YawPoint; //v1
        darrTro2PitchPoint[i] /= dNormOfdarrTro2PitchPoint; //v2

    }

    // get yaw and pitch
    dVectorDot = GeometryUtils::vectorDot(darrYaw2ClipPoint, darrPitch2YawPoint);//v3.v1
    if ( fabs(dVectorDot-1.0) < EPS_IK)
    {
        dVectorDot = 1.0;
    }else if ( fabs(dVectorDot+1.0) < EPS_IK)
    {
        dVectorDot = -1.0;
    }
    yaw = acos(dVectorDot);     // yaw = [0��PI]
    GeometryUtils::vectorCross(darrPitch2YawPoint, darrYaw2ClipPoint, darrYawCross); // v1 * v3
    // transform the z axis in clip coordinate system to kuka base coordinate system
    GeometryUtils::MatrixMultiply(darrClipR, darrzInLocal, 3, 3, 1, darrzInWorld);
    flagRaw = GeometryUtils::vectorDot(darrzInWorld, darrYawCross);
    // yaw direction flag
    if(flagRaw < -ALGORITHM_EPS)
    {
        yaw = -yaw;
    }

    // (get pitch )
    GeometryUtils::rotateZ(-yaw, darrRotYaw);
    matrix3x3Multiply(darrClipR, darrRotYaw, darrPitchR);
    dVectorDot2 = GeometryUtils::vectorDot(darrTro2PitchPoint, darrPitch2YawPoint);// v2 . v1
    if ( dVectorDot2 > 1 && dVectorDot2-1.0 < EPS_IK)                           // little bigger than 1
    {
        dVectorDot2 = 1.0;
    }else if ( dVectorDot2 < -1 && dVectorDot2+1.0 > -EPS_IK)                   //little less than -1
    {
        dVectorDot2 = -1.0;
    }
    pitch = acos(dVectorDot2);
    GeometryUtils::vectorCross(darrTro2PitchPoint, darrPitch2YawPoint, darrPitchCross);//v2 * v1
    GeometryUtils::MatrixMultiply(darrPitchR, darryInLocal, 3, 3, 1, darryInWorld);
    dFlagPitch = GeometryUtils::vectorDot(darryInWorld, darrPitchCross);        // pitch direction flag
    if(dFlagPitch > ALGORITHM_EPS)
    {
        pitch = -pitch;
    }

  // 5, get the frame of flange
    // refresh DH parameters
    pDH_param[11] = (90 + 0) * PI/180 + pitch;
    pDH_param[15] = yaw;  //rad
    eye(arrOriginTemp, 4);
    for (int i = 0 ; i < nDHparamRow; ++i )
    {
        DHfromPostposition2Homo(pDH_param[i*4], pDH_param[i*4 +1], pDH_param[i*4 +2], pDH_param[i*4 +3], darrTi[i]);
        matrix4x4Multiply(arrOriginTemp, darrTi[i], dHomoCur);
        memcpy(arrOriginTemp, dHomoCur, sizeof(double)*12);
    }
    homogeneous4x4Invers(dHomoCur, dHomoDHInv);
    matrix4x4Multiply(clipHomo, dHomoDHInv, dHomoFlange);

    // debug codes.
#ifdef dbg_IK_ex_new
    cout << __func__ << endl;
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

    cout << "2, calculate the trocar projection, darrYaw2TrocarProPoint: " << endl;
    cout << "trocar_project = " << endl;
    //cout << darrTrocarProject[0] << "    "  << darrTrocarProject[1] << "    "  << darrTrocarProject[2] << "    " << endl;
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

    cout << "darrPitch2YawPoint, darrTro2PitchPoint, darrYaw2ClipPoint, darrYawCross, darrPitchCross" << endl;
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
    cout << "5, get the frame of flange: " << endl;
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", dHomoFlange[i*4+j]);
        }
        printf("\n");
    }
    cout << "T1, T2, T3, T4, T1234, inv(T1234): " << endl;

    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrTi[0][i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrTi[1][i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrTi[2][i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrTi[3][i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", dHomoCur[i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", dHomoDHInv[i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
#endif

    //-------------------- solution 2 of pitch point : -----------------------------------------
    darrPitchPoint[0] = darrPitchPointTemp[1][0];
    darrPitchPoint[1] = darrPitchPointTemp[1][1];
    darrPitchPoint[2] = darrPitchPointTemp[1][2];

  // 4, get yaw and pitch
    for ( i = 0 ; i <3 ; ++i )
    {
        darrPitch2YawPoint[i] = darrYawPoint[i]   - darrPitchPoint[i];
        darrTro2PitchPoint[i] = darrPitchPoint[i] - pdTrocarPoint[i];
        darrYaw2ClipPoint[i]  = darrClipPoint[i]  - darrYawPoint[i];
    }
    dNormOfdarrPitch2YawPoint = GeometryUtils::norm(darrPitch2YawPoint);
    dNormOfdarrTro2PitchPoint = GeometryUtils::norm(darrTro2PitchPoint);
    dNormOfdarrYaw2ClipPoint  = GeometryUtils::norm(darrYaw2ClipPoint);
    for ( i = 0 ; i <3 ; ++i )
    {
        darrPitch2YawPoint[i] /= dNormOfdarrPitch2YawPoint;
        darrTro2PitchPoint[i] /= dNormOfdarrTro2PitchPoint;
        darrYaw2ClipPoint[i]  /= dNormOfdarrYaw2ClipPoint;
    }

    // get yaw and pitch
    dVectorDot = GeometryUtils::vectorDot(darrYaw2ClipPoint, darrPitch2YawPoint);
    if ( fabs(dVectorDot-1.0) < EPS_IK)
    {
        dVectorDot = 1.0;
    }else if ( fabs(dVectorDot+1.0) < EPS_IK)
    {
        dVectorDot = -1.0;
    }
    yaw2 = acos(dVectorDot);
    GeometryUtils::vectorCross(darrPitch2YawPoint, darrYaw2ClipPoint, darrYawCross);
    // transform the z axis in clip coordinate system to kuka base coordinate system
    GeometryUtils::MatrixMultiply(darrClipR, darrzInLocal, 3, 3, 1, darrzInWorld);
    flagRaw = GeometryUtils::vectorDot(darrzInWorld, darrYawCross);
    // yaw direction flag
    if(flagRaw < -ALGORITHM_EPS)
    {
        yaw2 = -yaw2;
    }

    // (get pitch )
    GeometryUtils::rotateZ(-yaw2, darrRotYaw);
    matrix3x3Multiply(darrClipR, darrRotYaw, darrPitchR);
    dVectorDot2 = GeometryUtils::vectorDot(darrTro2PitchPoint, darrPitch2YawPoint);
    if ( dVectorDot2 > 1 && dVectorDot2-1.0 < EPS_IK)                           // little bigger than 1
    {
        dVectorDot2 = 1.0;
    }else if ( dVectorDot2 < -1 && dVectorDot2+1.0 > -EPS_IK)                   //little less than -1
    {
        dVectorDot2 = -1.0;
    }
    pitch2 = acos(dVectorDot2);
    GeometryUtils::vectorCross(darrTro2PitchPoint, darrPitch2YawPoint, darrPitchCross);
    GeometryUtils::MatrixMultiply(darrPitchR, darryInLocal, 3, 3, 1, darryInWorld);
    dFlagPitch = GeometryUtils::vectorDot(darryInWorld, darrPitchCross);        // pitch direction flag
    if(dFlagPitch > ALGORITHM_EPS)
    {
        pitch2 = -pitch2;
    }

  // 5, get the frame of flange
    // refresh DH parameters
    pDH_param[11] = (90 + 0) * PI/180 + pitch2;
    pDH_param[15] = yaw2;  //rad
    eye(arrOriginTemp, 4);
    for (int i = 0 ; i < nDHparamRow; ++i )
    {
        DHfromPostposition2Homo(pDH_param[i*4], pDH_param[i*4 +1], pDH_param[i*4 +2], pDH_param[i*4 +3], darrTi[i]);
        matrix4x4Multiply(arrOriginTemp, darrTi[i], dHomoCur);
        memcpy(arrOriginTemp, dHomoCur, sizeof(double)*12);
    }
    homogeneous4x4Invers(dHomoCur, dHomoDHInv);
    matrix4x4Multiply(clipHomo, dHomoDHInv, dHomoFlange2);

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
            pDataOut[15 + i] = dHomoFlange[i*4+3];        // solution 1 of flange point
            pDataOut[18 + i] = dHomoFlange2[i*4+3];       // solution 2 of flange point
            //cout << __func__ << __LINE__ << endl;
        }
    }

    // debug codes.
#ifdef dbg_IK_ex_new
    cout << __func__ << endl;
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

    cout << "2, calculate the trocar projection, darrYaw2TrocarProPoint: " << endl;
    cout << "trocar_project = " << endl;
    //cout << darrTrocarProject[0] << "    "  << darrTrocarProject[1] << "    "  << darrTrocarProject[2] << "    " << endl;
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

    cout << "4, get yaw2 and pitch2: " << endl;
    cout << yaw2 << "\t" << pitch2 << endl;

    cout << "darrPitch2YawPoint, darrTro2PitchPoint, darrYaw2ClipPoint, darrYawCross, darrPitchCross" << endl;
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
    cout << "5, get the frame of flange2: " << endl;
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", dHomoFlange2[i*4+j]);
        }
        printf("\n");
    }
    cout << "T1, T2, T3, T4, T1234, inv(T1234): " << endl;

    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrTi[0][i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrTi[1][i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrTi[2][i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrTi[3][i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", dHomoCur[i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", dHomoDHInv[i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
#endif
}

/*****************************************************************************
 Prototype    : Algorithm.IK_ex
 Description  :
 Input        : clipHomo :      clip's frame (the fifth coordinate defined by Haoluo.Ning, 4*4 Homogeneous Matrix relative to kuka base coordinate system),
                                where the original point is on the half clip length, the x axis is along the clip from original point to endowrist point.
                trocarPoint   : trocar point in kuka base coordinate system.
                shaftLength   : the length of shaft. The end of shaft is the pitch joint point.
                clipLength    : clip' length.
                jointLength   : the length between pitch joint and yaw joint
 Output       : pitch         : unit:  radian
                yaw           : unit:  radian
                flangeFrame   : flange frame (relative to kuka base coordinate system)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/6/7
    Author       : lin.lin
    Modification : Created function
 notes:
    1, work out pitch,  yaw, and flange's frame according
       endowristFrame, trocarPoint, shaftLength, clipLength and jointLength;
*****************************************************************************/
void Algorithm::IK_ex(
    const Frame& clipHomo, const Vector& trocarPoint,
    const double& shaftLength, const double& clipLength, const double& jointLength,
    double& pitch, double& yaw, Frame& flangeFrame, double* pDataOut
)
{

//1, caculate the point of yaw joint in the kuka base coordinate system (Slave World coordinate system)
    //vector(3*1) from zero point to endowrist point in kuka base coordinate system;
    Vector   clipPoint = clipHomo.mOrigine;       // at the point of 1/2 of the clip's initial gesture.
    Rotation clipR     = clipHomo.mOrientation;
#if 1
    double*  clipRMultVect  = GeometryUtils::MatrixMultiply(clipR.mData,3,3, Vector(-clipLength/2,0,0).mData,3,1);
    //yawJointPoint: vector(3*1) from zero point to yaw joint point in kuka base coordinate system;
    Vector   yawJointPoint  = clipPoint + Vector(clipRMultVect[0], clipRMultVect[1], clipRMultVect[2]);
    delete[] clipRMultVect;
#else
    double   clipRMultVect[3] = { 0.0, 0.0, 0.0 } ;
    double   dTempVector[3]   = { -clipLength / 2, 0.0, 0.0 } ;
    //double*  clipRMultVect = GeometryUtils::MatrixMultiply(clipR.mData, 3, 3, Vector(-clipLength / 2, 0, 0).mData, 3, 1);
    Swift_Matrix3x3MultiplyVectorDouble(clipR.mData, dTempVector, clipRMultVect) ;
    //yawJointPoint: vector(3*1) from zero point to yaw joint point in kuka base coordinate system;
    Vector   yawJointPoint = clipPoint + Vector(clipRMultVect[0], clipRMultVect[1], clipRMultVect[2]);
#endif
    if (NULL != pDataOut)
    {
      pDataOut[3] = yawJointPoint.mData[0];
      pDataOut[4] = yawJointPoint.mData[1];
      pDataOut[5] = yawJointPoint.mData[2];
    }
//#define dbg_IK_ex
#ifdef dbg_IK_ex
    cout << __func__ << endl;
    double rTest, pTest, yTest;
    double darrclipHomo[16];
    int i, j;
    clipHomo.toHomo(darrclipHomo);
    homo2RPY(darrclipHomo, rTest, pTest, yTest);
    cout << "1, caculate the point of yaw joint " << endl;
    cout << "homo2RPY: " << rTest*180/PI << "\t" << pTest*180/PI << "\t" << yTest*180/PI << endl;
    cout << "clipPoint = " << endl;
    //cout << clipPoint.toString() << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%30.20f\t", clipPoint.mData[i]);
    }
    printf("\n");
    cout << "yawJointPoint = " << endl;
    //cout << yawJointPoint.toString() << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%30.20f\t", yawJointPoint.mData[i]);
    }
    printf("\n");
#endif
// 2, caculate the projection point of Trocar, and projection direction (trocar project to the plane formed by 2 clips)
    // the normal line of the plane which is formed by two clips in the initial zero pose is axis Z+.
    // the plane equation formed by two clips is: Ax + By + Cz + D = 0;
    // [A;B;C] = endowristR * [0;0;1];
    double A = clipR.mData[2+3*0];
    double B = clipR.mData[2+3*1];
    double C = clipR.mData[2+3*2];
    double D = -(A*yawJointPoint(0) + B*yawJointPoint(1) + C*yawJointPoint(2));

    // trocar projection: compute trocar projection coordinate in this plane
    // This problem is equivalent to find the crossover point between a perpendicular through trocar point and the clips' plane
    // Because it is a projection, the line must be perpendicular to the plane, The direction vector of the perpendicular line is A, B, C
    // The parametric equation of the perpendicular line is:
    // X = X_trocar + t * A
    // Y = Y_trocar + t * B
    // Z = Z_trocar + t * C
    // see the details: page 61 of analytic geometry

    //get trocar projection on that plane
    double t = -(A * trocarPoint.mData[0] + B * trocarPoint.mData[1]+ C * trocarPoint.mData[2] + D) / (A * A + B * B + C * C);
    Vector trocar_project(
        trocarPoint.mData[0] + A * t,
        trocarPoint.mData[1] + B * t,
        trocarPoint.mData[2] + C * t
    );
    // get the point of the end of shaft ; (vector from trocar projection point to yawJointPoint)
    Vector tp2yjp =  trocar_project - yawJointPoint;
#ifdef dbg_IK_ex
    cout << "2, trocar_project = " << endl;
    //cout << trocar_project.toString() << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%30.20f\t", trocar_project.mData[i]);
    }
    printf("\n");
    cout << "tp2yjp = " << endl;
    //cout << tp2yjp.toString() << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%30.20f\t", tp2yjp.mData[i]);
    }
    printf("\n");
#endif
// 3, get pitchJointPoint in kuka base coordinate system (end of shaft)
    double r = jointLength;
    double l = GeometryUtils::norm(tp2yjp);
    Vector pitchJointPoint;

    if(l <= ALGORITHM_EPS)
        pitchJointPoint = yawJointPoint + Vector(0, 0, -r);
    else
    {
//        pitchJointPoint.mData[0] = yawJointPoint.mData[0] + r * (tp2yjp.mData[0])/l;
//        pitchJointPoint.mData[1] = yawJointPoint.mData[1] + r * (tp2yjp.mData[1])/l;
//        pitchJointPoint.mData[2] = yawJointPoint.mData[2] + r * (tp2yjp.mData[2])/l;
        pitchJointPoint = yawJointPoint + tp2yjp * r / l;
    }

    if (NULL != pDataOut)
    {
        pDataOut[0] = pitchJointPoint.mData[0] ;
        pDataOut[1] = pitchJointPoint.mData[1] ;
        pDataOut[2] = pitchJointPoint.mData[2] ;
    }

#ifdef dbg_IK_ex
    cout << "3, pitchJointPoint = " << endl;
    //cout << pitchJointPoint.toString() << endl;
    for ( i = 0 ; i <3 ; ++i )
    {
        printf("%30.20f\t", pitchJointPoint.mData[i]);
    }
    printf("\n");
#endif
// 4, get yaw and pitch
    // define and normalization some vectors
    Vector pitch2yawV     = yawJointPoint - pitchJointPoint;// the vector from pitch joint to yaw joint; vector 1
    Vector trocar2pitchV  = pitchJointPoint - trocarPoint;  // the vector from trocar point to pitch joint; vector 2
    Vector yaw2EndowristV = clipPoint - yawJointPoint;      // the vector from yaw joint to endowrist point; vector 3
#ifdef dbg_IK_ex_
        cout << "line, darrPitch2YawPoint, norm: " << __LINE__ << "    ";
        //cout << darrPitch2YawPoint[0] << "    "  << darrPitch2YawPoint[1] << "    "  << darrPitch2YawPoint[2] << "    " << GeometryUtils::norm(darrPitch2YawPoint) << endl;
        for ( i = 0 ; i <3 ; ++i )
        {
            printf("%30.20f\t", pitch2yawV.mData[i]);
        }
        printf("\n");
        printf("norm(pitch2yawV) = %30.20f\n", GeometryUtils::norm(pitch2yawV));
#endif

    pitch2yawV     = pitch2yawV     / GeometryUtils::norm(pitch2yawV);          //sqrt(pitch2yawV(1)^2     + pitch2yawV(2)^2     + pitch2yawV(3)^2);
    trocar2pitchV  = trocar2pitchV  / GeometryUtils::norm(trocar2pitchV);       //sqrt(trocar2pitchV(1)^2  + trocar2pitchV(2)^2  + trocar2pitchV(3)^2);
    yaw2EndowristV = yaw2EndowristV / GeometryUtils::norm(yaw2EndowristV);      //sqrt(yaw2EndowristV(1)^2 + yaw2EndowristV(2)^2 + yaw2EndowristV(3)^2);

    //% get yaw
    yaw = acos(GeometryUtils::vectorDot(yaw2EndowristV, pitch2yawV));           //v3 . v1
    Vector yawCross= GeometryUtils::vectorCross(pitch2yawV, yaw2EndowristV);    //v1 * v3
    // transform the z axis in clip coordinate system to kuka base coordinate system

#if 0
    double* newyMatrix = GeometryUtils::MatrixMultiply(clipR.mData,3,3, Vector(0,0,1).mData,3,1);
    Vector z2slaveworldV(newyMatrix[0],newyMatrix[1],newyMatrix[2]);
    double flagRaw = GeometryUtils::vectorDot(z2slaveworldV, yawCross);
    // yaw direction flag
    if(flagRaw< -ALGORITHM_EPS) {
        yaw = -yaw;
    }
    delete[] newyMatrix;
    //% get pitch
    double* rotYaw = GeometryUtils::rotateZ(-yaw);
    double* temp1 = GeometryUtils::MatrixMultiply(clipR.mData,3,3, rotYaw,3,3);

    pitch = acos(GeometryUtils::vectorDot(trocar2pitchV, pitch2yawV));          //v2 . v1
    Vector pitchCross = GeometryUtils::vectorCross(trocar2pitchV, pitch2yawV);  //v2 * v1
    // transform the y axis in wrist2 coordinate system to kuka base coordinate system
    double* newxMatrix = GeometryUtils::MatrixMultiply(temp1,3,3, Vector(0,1,0).mData,3,1);
    Vector y2slaveworldV(newxMatrix[0],newxMatrix[1],newxMatrix[2]);
    // pitch direction flag
    double flagPitch = GeometryUtils::vectorDot(y2slaveworldV, pitchCross);
    if(flagPitch > ALGORITHM_EPS){
        pitch = -pitch;
    }
    delete[] rotYaw;
    delete[] temp1;
    delete[] newxMatrix;
#else
    double* newyMatrix = GeometryUtils::MatrixMultiply(clipR.mData,3,3, Vector(0,0,1).mData,3,1);
    Vector z2slaveworldV(newyMatrix[0], newyMatrix[1], newyMatrix[2]);
    double flagRaw = GeometryUtils::vectorDot(z2slaveworldV, yawCross);
    // yaw direction flag
    if (flagRaw< -ALGORITHM_EPS) {
        yaw = -yaw;
    }
    //% get pitch
    double* rotYaw = GeometryUtils::rotateZ(-yaw);
    double* temp1 = GeometryUtils::MatrixMultiply(clipR.mData,3,3, rotYaw,3,3);
    pitch = acos(GeometryUtils::vectorDot(trocar2pitchV, pitch2yawV));          //v2 . v1
    Vector pitchCross = GeometryUtils::vectorCross(trocar2pitchV, pitch2yawV);  //v2 * v1
                                                                                // transform the y axis in wrist2 coordinate system to kuka base coordinate system
    double* newxMatrix = GeometryUtils::MatrixMultiply(temp1,3,3, Vector(0,1,0).mData,3,1);
    Vector y2slaveworldV(newxMatrix[0], newxMatrix[1], newxMatrix[2]);
    // pitch direction flag
    double flagPitch = GeometryUtils::vectorDot(y2slaveworldV, pitchCross);
    if (flagPitch > ALGORITHM_EPS) {
        pitch = -pitch;
    }
    delete[] rotYaw;
#endif
#ifdef dbg_IK_ex
    //int i, j;
    cout << "4, (yaw, pitch) = " << endl;
    cout << yaw << "\t" << pitch << endl;

    cout << "pitch2yawV, trocar2pitchV, yaw2EndowristV, yawCross, pitchCross" << endl;
    for ( i = 0 ; i <3 ; i++ )
    {
        printf("%30.20f\t", pitch2yawV.mData[i]);
    }
    cout << endl;
    for ( i = 0 ; i <3 ; i++ )
    {
        printf("%30.20f\t", trocar2pitchV.mData[i]);
    }
    cout << endl;
    for ( i = 0 ; i <3 ; i++ )
    {
        printf("%30.20f\t", yaw2EndowristV.mData[i]);
    }
    cout << endl;
    for ( i = 0 ; i <3 ; i++ )
    {
        printf("%30.20f\t", yawCross.mData[i]);
    }
    cout << endl;
    for ( i = 0 ; i <3 ; i++ )
    {
        printf("%30.20f\t", pitchCross.mData[i]);
    }
    cout << endl;
    cout << "dVectorDot = "  << GeometryUtils::vectorDot(yaw2EndowristV, pitch2yawV)  << endl;
    cout << "dVectorDot2 = " << GeometryUtils::vectorDot(trocar2pitchV, pitch2yawV) << endl;
#endif
// 5, get the frame of flange
    double d1, d2, a3, a4;
    double dHomoCur[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT]        = { 0.0 } ;
    double dHomoFlange[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT]     = { 0.0 } ;
    double dHomoDHInv[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT]      = { 0.0 } ;
#ifndef NEW_INSTR_DH
    d1 = 0;
    d2 = shaftLength;
    a3 = jointLength;
    a4 = clipLength / 2;
    // DH transform parameter. unit: degree.
    double DH_param[4][4] = {
        0,  0, d1, TOOL_ROLL_DEGREE_TO_FLANGE,
        90,  0, d2, 90,
        -90, a3, 0,  90 + pitch * 180 / PI,
        0, a4, 0,  yaw * 180 / PI };
#else
#ifndef NEW_INSTR_DH_GEAR
    pitch = -pitch ;
    d1 = shaftLength;
    d2 = 0;
    a3 = jointLength;
    a4 = clipLength / 2.0 ;
    // DH transform parameter. unit: degree.
    double DH_param[4][4] = {
      0.0,    0.0,  d1,     TOOL_ROLL_DEGREE_TO_FLANGE,
      90.0,   0.0,  d2,     90.0 + pitch * 180.0 / PI,
      90.0,   a3,   0.0,    yaw * 180.0 / PI,
      0.0,    a4,   0.0,    0.0 } ;
#else
    pitch = -pitch ;
    d1    = shaftLength ;
    d2    = 0 ;
    a3    = jointLength ;
    a4    = clipLength / 2.0 ;
    // DH transform parameter. unit: degree.
    double DH_param[4][4] = {
        0.0,    0.0,  d1,     90.0 + TOOL_ROLL_DEGREE_TO_FLANGE,
        90.0,   0.0,  d2,     90.0 + pitch * 180.0 / PI,
        90.0,   a3,   0.0,    yaw * 180.0 / PI,
        0.0,    a4,   0.0,    0.0 };
#endif
#endif
#if 1
#ifndef NEW_INSTR_DH
    Frame T1     = Frame::fromPostpositionDH(DH_param[0][1], DH_param[0][0] * PI/180, DH_param[0][2], DH_param[0][3] * PI/180);
    Frame T2     = Frame::fromPostpositionDH(DH_param[1][1], DH_param[1][0] * PI/180, DH_param[1][2], DH_param[1][3] * PI/180);
    Frame T3     = Frame::fromPostpositionDH(DH_param[2][1], DH_param[2][0] * PI/180, DH_param[2][2], DH_param[2][3] * PI/180);
    Frame T4     = Frame::fromPostpositionDH(DH_param[3][1], DH_param[3][0] * PI/180, DH_param[3][2], DH_param[3][3] * PI/180);
    Frame frmDH  = T1 * T2 * T3 * T4 ;
    frmDH.toHomo(dHomoCur) ;
    homogeneous4x4Invers(dHomoCur, dHomoDHInv) ;
    clipHomo.toHomo(dHomoCur) ;
    matrix4x4Multiply(dHomoCur, dHomoDHInv, dHomoFlange) ;
    flangeFrame.fromHomo(dHomoFlange) ;
#else
    Frame T1 = Frame::fromPrepositionDH(DH_param[0][1], DH_param[0][0] * PI / 180, DH_param[0][2], DH_param[0][3] * PI / 180);
    Frame T2 = Frame::fromPrepositionDH(DH_param[1][1], DH_param[1][0] * PI / 180, DH_param[1][2], DH_param[1][3] * PI / 180);
    Frame T3 = Frame::fromPrepositionDH(DH_param[2][1], DH_param[2][0] * PI / 180, DH_param[2][2], DH_param[2][3] * PI / 180);
    Frame T4 = Frame::fromPrepositionDH(DH_param[3][1], DH_param[3][0] * PI / 180, DH_param[3][2], DH_param[3][3] * PI / 180);
    Frame frmDH = T1 * T2 * T3 * T4;
    frmDH.toHomo(dHomoCur);
    homogeneous4x4Invers(dHomoCur, dHomoDHInv);
    clipHomo.toHomo(dHomoCur);
    matrix4x4Multiply(dHomoCur, dHomoDHInv, dHomoFlange);
    flangeFrame.fromHomo(dHomoFlange);
#endif
    if (NULL != pDataOut)
    {
      pDataOut[6] = pitch ;
      pDataOut[7] = yaw ;
      pDataOut[8]  = trocarPoint.mData[0] ;
      pDataOut[9]  = trocarPoint.mData[1] ;
      pDataOut[10] = trocarPoint.mData[2] ;
    }

#else
    Swift_DH2HomoWithPostPositionDouble(DH_param[0][1], DH_param[0][0] * KY_DEG2RAD_PIDIV180, DH_param[0][2], DH_param[0][3] * KY_DEG2RAD_PIDIV180, dHomoDHInv) ;
    Swift_DH2HomoWithPostPositionDouble(DH_param[1][1], DH_param[1][0] * KY_DEG2RAD_PIDIV180, DH_param[1][2], DH_param[1][3] * KY_DEG2RAD_PIDIV180, dHomoFlange) ;
    Swift_Matrix4x4MultiplyDouble(dHomoDHInv, dHomoFlange, dHomoCur) ;
    Swift_DH2HomoWithPostPositionDouble(DH_param[2][1], DH_param[2][0] * KY_DEG2RAD_PIDIV180, DH_param[2][2], DH_param[2][3] * KY_DEG2RAD_PIDIV180, dHomoDHInv) ;
    Swift_Matrix4x4MultiplyDouble(dHomoCur, dHomoDHInv, dHomoFlange) ;
    Swift_DH2HomoWithPostPositionDouble(DH_param[3][1], DH_param[3][0] * KY_DEG2RAD_PIDIV180, DH_param[3][2], DH_param[3][3] * KY_DEG2RAD_PIDIV180, dHomoDHInv) ;
    Swift_Matrix4x4MultiplyDouble(dHomoFlange, dHomoDHInv, dHomoCur) ;
    Swift_HomoInverseDouble(dHomoCur, dHomoDHInv) ;
    clipHomo.toHomo(dHomoCur) ;
    Swift_Matrix4x4MultiplyDouble(dHomoCur, dHomoDHInv, dHomoFlange) ;
    flangeFrame.fromHomo(dHomoFlange) ;
#endif
#ifdef dbg_IK_ex
    cout << "5, get the frame of flange: " << endl;
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", dHomoFlange[i*4+j]);
        }
        printf("\n");
    }
    cout << "T1, T2, T3, T4, T1234, inv(T1234): " << endl;
    double darrT[16];
    T1.toHomo(darrT);
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrT[i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    T2.toHomo(darrT);
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrT[i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    T3.toHomo(darrT);
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrT[i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    T4.toHomo(darrT);
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrT[i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    frmDH.toHomo(darrT);
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", darrT[i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
    for ( i = 0 ; i <4 ; i++ )
    {
        for ( j = 0 ; j <4 ; j++ )
        {
            printf("%30.15f\t", dHomoDHInv[i*4+j]);
        }
        printf("\n");
    }
    printf("\n");
#endif
}
/* END: Added by Linl, 2017/3~4 */
/* BEGIN: Added by lin.lin, 2017/8/11 */
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
*****************************************************************************/
void Algorithm::CreateAdjustedVectors(const double* darrSphereCenterV, const double dSphereR, double** ppdNeighbourVectTrans/* 15*3 */)
{
    //cout << "func, line = " << __func__ << "  " << __LINE__ << endl;
    double arrRot[9]  = {0.0};
    double darrVectorZ[3] = {0.0, 0.0, dSphereR};
    int i, j;
    double *pdNeighbourVectTrans = (double*)ppdNeighbourVectTrans;
    pdNeighbourVectTrans[0] = 0.0;
    pdNeighbourVectTrans[1] = 0.0;
    pdNeighbourVectTrans[2] = 0.0;
    pdNeighbourVectTrans = (double*)ppdNeighbourVectTrans+13*3;

    pdNeighbourVectTrans[0] = darrVectorZ[0];
    pdNeighbourVectTrans[1] = darrVectorZ[1];
    pdNeighbourVectTrans[2] = darrVectorZ[2];

    GeometryUtils::rotateY(DEG2RAD_45, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (double*)ppdNeighbourVectTrans+5*3);
    GeometryUtils::rotateX(-DEG2RAD_45, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (double*)ppdNeighbourVectTrans+6*3);
    GeometryUtils::rotateY(-DEG2RAD_45, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (double*)ppdNeighbourVectTrans+7*3);
    GeometryUtils::rotateX(DEG2RAD_45, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (double*)ppdNeighbourVectTrans+8*3);
//    % A0 B0 C0 D0 2~5
    GeometryUtils::rotateY(DEG2RAD_90, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (double*)ppdNeighbourVectTrans+1*3);
    GeometryUtils::rotateX(-DEG2RAD_90, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (double*)ppdNeighbourVectTrans+2*3);
    GeometryUtils::rotateY(-DEG2RAD_90, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (double*)ppdNeighbourVectTrans+3*3);
    GeometryUtils::rotateX(DEG2RAD_90, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, (double*)ppdNeighbourVectTrans+4*3);
//     % A-1 B-1 C-1 D-1 10~13
    GeometryUtils::rotateY(DEG2RAD_135, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, ((double*)ppdNeighbourVectTrans + 9*3));
    GeometryUtils::rotateX(-DEG2RAD_135, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, ((double*)ppdNeighbourVectTrans + 10*3));
    GeometryUtils::rotateY(-DEG2RAD_135, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, ((double*)ppdNeighbourVectTrans + 11*3));
    GeometryUtils::rotateX(DEG2RAD_135, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, ((double*)ppdNeighbourVectTrans + 12*3));

//    neighbourhoodV(:,15) = roty(pi) * darrVectorZ;
    GeometryUtils::rotateY(PI, arrRot);
    matrix3x1Multiply(arrRot, darrVectorZ, ((double*)ppdNeighbourVectTrans + 14*3));
    for ( i = 0 ; i < 15 ; ++i )
    {
        pdNeighbourVectTrans = (double*)ppdNeighbourVectTrans + i*3;
        pdNeighbourVectTrans[0] +=  darrSphereCenterV[0];
        pdNeighbourVectTrans[1] +=  darrSphereCenterV[1];
        pdNeighbourVectTrans[2] +=  darrSphereCenterV[2];
    }
}

/* END:   Added by lin.lin, 2017/8/11   PN: */

// ABC: Euler angles, A: round Z, B: round Y, C: round X
// See Section "Coordinate Systems" in the Sunrise.OS user manual
void Algorithm::kukaTransFromEuler(double x, double y, double z, double aDeg, double bDeg, double cDeg, double trans[][4])
{
    double a = aDeg * PI / 180;
    double b = bDeg * PI / 180;
    double c = cDeg * PI / 180;

    double cz = cos(a);
    double sz = sin(a);
    double cy = cos(b);
    double sy = sin(b);
    double cx = cos(c);
    double sx = sin(c);

    trans[0][0] = cy * cz;
    trans[0][1] = cz * sx * sy - cx * sz;
    trans[0][2] = cx * cz * sy + sx * sz;
    trans[0][3] = x;

    trans[1][0] = cy * sz;
    trans[1][1] = cx * cz + sx * sy * sz;
    trans[1][2] = -cz * sx + cx * sy * sz;
    trans[1][3] = y;

    trans[2][0] = -sy;
    trans[2][1] = cy * sx;
    trans[2][2] = cx * cy;
    trans[2][3] = z;
}


//Convert Euler angles to rotation matrix
//The default rotation sequence is 'ZYX', where the order of rotation
//    angles is Z Axis Rotation, Y Axis Rotation, and X Axis Rotation.
double* Algorithm::eul2rotm(double Euler[])
{
    double* resTemp = new double[3*3];
    double* result = new double[3*3];
    double* rz = GeometryUtils::rotateZ(Euler[0]);
    double* ry = GeometryUtils::rotateY(Euler[1]);
    double* rx = GeometryUtils::rotateX(Euler[2]);

    GeometryUtils::MatrixMultiply(rz, ry, 3, 3, 3, resTemp);
    GeometryUtils::MatrixMultiply(resTemp, rx, 3, 3, 3, result);

    //free memory
    delete[] rx;
    delete[] ry;
    delete[] rz;
    delete[] resTemp;
    return result;
}
/* END:   Added by lin.lin, 2017/4/12 */
//computes the forward kinematics
//homogeneous transformation matrix given the vectors of DH patameters
//a,input vectors(array) of DH parameters
//b,input vectors(array) of DH parameters
//alpha,input vectors(array) of DH parameters
//theta,input vectors(array) of DH parameters
//T,output,homogeneous transformation matrix
void Algorithm::fkine(const double* a,const double* d,const double* alpha,const double* theta,Frame& frame)
{
    double *T= new double[4*4];
    memset(T,0,sizeof(double)*4*4);
    T[0] = T[5] = T[10] = T[15] = 1;
    //length of DH
    int n = 6;
    for(int i=0;i<n;++i)
    {
        double temp[] = {
            cos(theta[i]),-sin(theta[i]) * cos(alpha[i]),sin(theta[i]) * sin(alpha[i]),a[i] * cos(theta[i]),
            sin(theta[i]),cos(theta[i]) * cos(alpha[i]),-cos(theta[i]) * sin(alpha[i]),a[i] * sin(theta[i]),
            0,sin(alpha[i]),cos(alpha[i]),d[i],
            0,0,0,1
        };
        double* t = GeometryUtils::MatrixMultiply(T,4,4,temp,4,4);
        memcpy(T,t,sizeof(double)*4*4);
        delete[] t;
    }
    for(int i=0;i<3;++i)
    {
        for(int j=0;j<3;++j)
        {
            frame.mOrientation.mData[i*3+j] = T[i*4+j];
        }
    }
    frame.mOrigine.mData[0] = T[3];
    frame.mOrigine.mData[1] = T[7];
    frame.mOrigine.mData[2] = T[11];
    delete[] T;
}

Eigen::Matrix<double, 6, 1> Algorithm::NewSurgnova_IK_6Joints(Eigen::Matrix<double, 4, 4> clipHomo, Eigen::Vector3d trocarPosition)
{
  const double lengthPitch = 11.25 / 1000;
  const double lengthYaw = 9.52 / 1000;

  double A3 = lengthPitch;
  double A4 = lengthYaw;

  Eigen::Vector3d ClipPosition = clipHomo.block(0, 3, 3, 1);
  Eigen::Vector3d ClipZ = clipHomo.block(0, 2, 3, 1);
  Eigen::Vector3d ClipY = clipHomo.block(0, 1, 3, 1);

  Eigen::Matrix<double, 4, 4> trans;
  trans << 1, 0, 0, -A4,
           0, 1, 0,   0,
           0, 0, 1,   0,
           0, 0, 0,   1;

  Eigen::Matrix<double, 4, 4> YawPointHomo = clipHomo * trans;
  Eigen::Vector3d YawPosition = YawPointHomo.block(0, 3, 3, 1);

  double B = ClipPosition.dot(ClipZ);
  double X4 = trocarPosition(0);
  double Y4 = trocarPosition(1);
  double Z4 = trocarPosition(2);

  double a = ClipZ(0);
  double b = ClipZ(1);
  double c = ClipZ(2);

  double C = B - Y4*b - Z4*c + (X4*b*b)/a + (X4*c*c)/a;
  double D = a + b*b/a + c*c/a;

  double XA = C / D;
  double YA = (XA - X4)*b/a + Y4;
  double ZA = (XA - X4)*c/a + Z4;

  Eigen::Vector3d TrocarProjectionPosition;
  TrocarProjectionPosition << XA, YA, ZA;

  double L_Clip2TrocarProjection = (TrocarProjectionPosition - ClipPosition).norm();
  double L_Yaw2TrocarProjection = (TrocarProjectionPosition-YawPosition).norm();
  double Cos_AngleClipYawTrocarProject = (pow(A4, 2) + pow(L_Yaw2TrocarProjection, 2) - pow(L_Clip2TrocarProjection, 2)) / (2*A4*L_Yaw2TrocarProjection);

  if (Cos_AngleClipYawTrocarProject > 1 && fabs(Cos_AngleClipYawTrocarProject) - 1 < 1e-5)
    Cos_AngleClipYawTrocarProject = 1;
  else if (Cos_AngleClipYawTrocarProject < -1 && fabs(Cos_AngleClipYawTrocarProject) - 1 < 1e-5)
    Cos_AngleClipYawTrocarProject = -1;
  else
    Cos_AngleClipYawTrocarProject = Cos_AngleClipYawTrocarProject;

  double AngleClipYawTrocarProjectRad = acos(Cos_AngleClipYawTrocarProject);
  double AngleClipYawTrocarProjectDeg = AngleClipYawTrocarProjectRad * 180 / M_PI;
  Eigen::Vector3d Vector_Trocar2Clip = ClipPosition - trocarPosition;

  double costheta2 = Vector_Trocar2Clip.dot(ClipY) / (Vector_Trocar2Clip.norm() * ClipY.norm());

  double YawRad = 0.0;
  double YawDeg = 0.0;
  double PitchRad = 0.0;
  double PitchDeg = 0.0;

  if (costheta2 <= 0) {
    YawRad = M_PI - AngleClipYawTrocarProjectRad;
    YawDeg = YawRad * 180 / M_PI;
  } else {
    YawRad = -(M_PI - AngleClipYawTrocarProjectRad);
    YawDeg = YawRad * 180 / M_PI;
  }

  Eigen::Vector3d Vector_TrocarProject2Yaw = YawPosition - TrocarProjectionPosition;
  double L_TrocarProject2Yaw = Vector_TrocarProject2Yaw.norm();
  double L_TrocarProject2Pitch = L_TrocarProject2Yaw - A3;
  double OScale = L_TrocarProject2Pitch / L_TrocarProject2Yaw;

  Eigen::Vector3d PitchPosition;
  PitchPosition << Vector_TrocarProject2Yaw(0) * OScale + TrocarProjectionPosition(0),
                   Vector_TrocarProject2Yaw(1) * OScale + TrocarProjectionPosition(1),
                   Vector_TrocarProject2Yaw(2) * OScale + TrocarProjectionPosition(2);

  Eigen::Vector3d Vector_Trocar2Pitch = PitchPosition - trocarPosition;
  double L_Trocar2Pitch = Vector_Trocar2Pitch.norm();
  Eigen::Vector3d Vector_Trocar2Yaw = YawPosition - trocarPosition;
  double L_TrocarYaw = Vector_Trocar2Yaw.norm();
  double Cos_AngleYawPitchTrocar = (pow(L_Trocar2Pitch, 2) + pow(A3, 2) - pow(L_TrocarYaw, 2)) / (2*L_Trocar2Pitch*A3);

  if (Cos_AngleYawPitchTrocar > 1 && fabs(Cos_AngleYawPitchTrocar) - 1 < 1e-9)
    Cos_AngleYawPitchTrocar = 1;
  else if (Cos_AngleYawPitchTrocar < -1 && fabs(Cos_AngleYawPitchTrocar) - 1 < 1e-9)
    Cos_AngleYawPitchTrocar = -1;
  else
    Cos_AngleYawPitchTrocar = Cos_AngleYawPitchTrocar;

  double AngleYawPitchTrocarRad = acos(Cos_AngleYawPitchTrocar);
  double AngleYawPitchTrocarDeg = AngleYawPitchTrocarRad * 180 / M_PI;
  double costheta = Vector_Trocar2Yaw.dot(ClipZ) / ((Vector_Trocar2Yaw).norm() * ClipZ.norm());

  if (costheta <= 0) {
    PitchRad = AngleYawPitchTrocarRad - M_PI / 2;
    PitchDeg = PitchRad * 180 / M_PI ;
  } else {
    PitchRad = 2*M_PI - AngleYawPitchTrocarRad - M_PI / 2;
    PitchDeg = PitchRad * 180 / M_PI;
  }

  //Get D4
  double d4 = (PitchPosition - trocarPosition).norm();
  //Get TrocarHomo
  double q6 = YawDeg * M_PI / 180;
  double a6 = lengthYaw;

  Eigen::Matrix<double, 4, 4> T6;
  T6 << cos(q6), -sin(q6), 0, a6*cos(q6),
        sin(q6),  cos(q6), 0, a6*sin(q6),
              0,        0, 1,          0,
              0,        0, 0,          1;

  double q5 = PitchDeg * M_PI / 180;
  double a5 = lengthPitch;

  Eigen::Matrix<double, 4, 4> T5;
  T5 << cos(q5), 0,  sin(q5), a5*cos(q5),
        sin(q5), 0, -cos(q5), a5*sin(q5),
              0, 1,        0,          0,
              0, 0,        0,          1;

  Eigen::Matrix<double, 4, 4> T4;
  T4 << 1, 0,  0,  0,
        0, 0, -1,  0,
        0, 1,  0, d4,
        0, 0,  0,  1;

  Eigen::Matrix<double, 4, 4> TrocarHomo = clipHomo*T6.inverse()*T5.inverse()*T4.inverse();
  // Get Gimbal
  double T11 = TrocarHomo(0, 0);
  double T13 = TrocarHomo(0 ,2);
  double T23 = TrocarHomo(1, 2);
  double T31 = TrocarHomo(2, 0);
  double T32 = TrocarHomo(2, 1);
  double T33 = TrocarHomo(2, 2);

  double S1 = -T23*sqrt(-T11/(T23*T32 + T13*T31*T33));
  double C1 = -T13*sqrt(-T11/(T23*T32 + T13*T31*T33));

  double S2 = -((T23*T32 + T13*T31*T33)*sqrt(-T11/(T23*T32 + T13*T31*T33))) / T11;
  double C2 = T33;

  double S3 = -T32*sqrt(-T11/(T23*T32 + T13*T31*T33));
  double C3 = T31*sqrt(-T11/(T23*T32 + T13*T31*T33));

  double q1 = atan2(S1,C1);
  double q2 = atan2(S2,C2);
  double q3 = atan2(S3,C3);

  Eigen::Vector3d ThetaTrocarGimbal;
  ThetaTrocarGimbal << q1, q2, q3;

  Eigen::Matrix<double, 6, 1> sixAxes;
  sixAxes << q1, q2, q3, d4, PitchRad, YawRad;
  return sixAxes;
}

void Algorithm::homoFrmMultiplyFrm(const double* pdA, const Frame& frmB, Frame& frmC)
{
    frmC.mOrientation.mData[0]  = pdA[0] * frmB.mOrientation.mData[0] + pdA[1] * frmB.mOrientation.mData[3] + pdA[2] * frmB.mOrientation.mData[6] ;
    frmC.mOrientation.mData[1]  = pdA[0] * frmB.mOrientation.mData[1] + pdA[1] * frmB.mOrientation.mData[4] + pdA[2] * frmB.mOrientation.mData[7] ;
    frmC.mOrientation.mData[2]  = pdA[0] * frmB.mOrientation.mData[2] + pdA[1] * frmB.mOrientation.mData[5] + pdA[2] * frmB.mOrientation.mData[8] ;
    frmC.mOrigine.mData[0]      = pdA[0] * frmB.mOrigine.mData[0] + pdA[1] * frmB.mOrigine.mData[1] + pdA[2] * frmB.mOrigine.mData[2] + pdA[3] ;
    frmC.mOrientation.mData[3]  = pdA[4] * frmB.mOrientation.mData[0] + pdA[5] * frmB.mOrientation.mData[3] + pdA[6] * frmB.mOrientation.mData[6] ;
    frmC.mOrientation.mData[4]  = pdA[4] * frmB.mOrientation.mData[1] + pdA[5] * frmB.mOrientation.mData[4] + pdA[6] * frmB.mOrientation.mData[7] ;
    frmC.mOrientation.mData[5]  = pdA[4] * frmB.mOrientation.mData[2] + pdA[5] * frmB.mOrientation.mData[5] + pdA[6] * frmB.mOrientation.mData[8] ;
    frmC.mOrigine.mData[1]      = pdA[4] * frmB.mOrigine.mData[0] + pdA[5] * frmB.mOrigine.mData[1] + pdA[6] * frmB.mOrigine.mData[2] + pdA[7] ;
    frmC.mOrientation.mData[6]  = pdA[8] * frmB.mOrientation.mData[0] + pdA[9] * frmB.mOrientation.mData[3] + pdA[10] * frmB.mOrientation.mData[6] ;
    frmC.mOrientation.mData[7]  = pdA[8] * frmB.mOrientation.mData[1] + pdA[9] * frmB.mOrientation.mData[4] + pdA[10] * frmB.mOrientation.mData[7] ;
    frmC.mOrientation.mData[8]  = pdA[8] * frmB.mOrientation.mData[2] + pdA[9] * frmB.mOrientation.mData[5] + pdA[10] * frmB.mOrientation.mData[8] ;
    frmC.mOrigine.mData[2]      = pdA[8] * frmB.mOrigine.mData[0] + pdA[9] * frmB.mOrigine.mData[1] + pdA[10] * frmB.mOrigine.mData[2] + pdA[11] ;
}

void Algorithm::homohomoMultiplyFrm(const double* pdA, const double* pdB, Frame& frmC)
{
    frmC.mOrientation.mData[0] = pdA[0] * pdB[0] + pdA[1] * pdB[4] + pdA[2] * pdB[8] ;
    frmC.mOrientation.mData[1] = pdA[0] * pdB[1] + pdA[1] * pdB[5] + pdA[2] * pdB[9] ;
    frmC.mOrientation.mData[2] = pdA[0] * pdB[2] + pdA[1] * pdB[6] + pdA[2] * pdB[10] ;
    frmC.mOrigine.mData[0]     = pdA[0] * pdB[3] + pdA[1] * pdB[7] + pdA[2] * pdB[11] + pdA[3] ;
    frmC.mOrientation.mData[3] = pdA[4] * pdB[0] + pdA[5] * pdB[4] + pdA[6] * pdB[8] ;
    frmC.mOrientation.mData[4] = pdA[4] * pdB[1] + pdA[5] * pdB[5] + pdA[6] * pdB[9] ;
    frmC.mOrientation.mData[5] = pdA[4] * pdB[2] + pdA[5] * pdB[6] + pdA[6] * pdB[10] ;
    frmC.mOrigine.mData[1]     = pdA[4] * pdB[3] + pdA[5] * pdB[7] + pdA[6] * pdB[11] + pdA[7] ;
    frmC.mOrientation.mData[6] = pdA[8] * pdB[0] + pdA[9] * pdB[4] + pdA[10] * pdB[8] ;
    frmC.mOrientation.mData[7] = pdA[8] * pdB[1] + pdA[9] * pdB[5] + pdA[10] * pdB[9] ;
    frmC.mOrientation.mData[8] = pdA[8] * pdB[2] + pdA[9] * pdB[6] + pdA[10] * pdB[10] ;
    frmC.mOrigine.mData[2]     = pdA[8] * pdB[3] + pdA[9] * pdB[7] + pdA[10] * pdB[11] + pdA[11] ;
}

void Algorithm::frameHomoMultiply(const Frame& frmA, const double* pdB, Frame& frmC)
{
    frmC.mOrientation.mData[0]  = frmA.mOrientation.mData[0] * pdB[0] + frmA.mOrientation.mData[1] * pdB[4] + frmA.mOrientation.mData[2] * pdB[8] ;
    frmC.mOrientation.mData[1]  = frmA.mOrientation.mData[0] * pdB[1] + frmA.mOrientation.mData[1] * pdB[5] + frmA.mOrientation.mData[2] * pdB[9] ;
    frmC.mOrientation.mData[2]  = frmA.mOrientation.mData[0] * pdB[2] + frmA.mOrientation.mData[1] * pdB[6] + frmA.mOrientation.mData[2] * pdB[10] ;
    frmC.mOrigine.mData[0]      = frmA.mOrientation.mData[0] * pdB[3] + frmA.mOrientation.mData[1] * pdB[7] + frmA.mOrientation.mData[2] * pdB[11] + frmA.mOrigine.mData[0] ;
    frmC.mOrientation.mData[3]  = frmA.mOrientation.mData[3] * pdB[0] + frmA.mOrientation.mData[4] * pdB[4] + frmA.mOrientation.mData[5] * pdB[8] ;
    frmC.mOrientation.mData[4]  = frmA.mOrientation.mData[3] * pdB[1] + frmA.mOrientation.mData[4] * pdB[5] + frmA.mOrientation.mData[5] * pdB[9] ;
    frmC.mOrientation.mData[5]  = frmA.mOrientation.mData[3] * pdB[2] + frmA.mOrientation.mData[4] * pdB[6] + frmA.mOrientation.mData[5] * pdB[10] ;
    frmC.mOrigine.mData[1]      = frmA.mOrientation.mData[3] * pdB[3] + frmA.mOrientation.mData[4] * pdB[7] + frmA.mOrientation.mData[5] * pdB[11] + frmA.mOrigine.mData[1] ;
    frmC.mOrientation.mData[6]  = frmA.mOrientation.mData[6] * pdB[0] + frmA.mOrientation.mData[7] * pdB[4] + frmA.mOrientation.mData[8] * pdB[8] ;
    frmC.mOrientation.mData[7]  = frmA.mOrientation.mData[6] * pdB[1] + frmA.mOrientation.mData[7] * pdB[5] + frmA.mOrientation.mData[8] * pdB[9] ;
    frmC.mOrientation.mData[8]  = frmA.mOrientation.mData[6] * pdB[2] + frmA.mOrientation.mData[7] * pdB[6] + frmA.mOrientation.mData[8] * pdB[10] ;
    frmC.mOrigine.mData[2]      = frmA.mOrientation.mData[6] * pdB[3] + frmA.mOrientation.mData[7] * pdB[7] + frmA.mOrientation.mData[8] * pdB[11] + frmA.mOrigine.mData[2] ;
}

void Algorithm::ortHomoMultiplyHomo(const double* pOrt, const double* pHomoIn, double* pHomoOut)
{
    pHomoOut[0]  = pOrt[0] * pHomoIn[0] + pOrt[1] * pHomoIn[4] + pOrt[2] * pHomoIn[8] ;
    pHomoOut[1]  = pOrt[0] * pHomoIn[1] + pOrt[1] * pHomoIn[5] + pOrt[2] * pHomoIn[9] ;
    pHomoOut[2]  = pOrt[0] * pHomoIn[2] + pOrt[1] * pHomoIn[6] + pOrt[2] * pHomoIn[10] ;
    pHomoOut[4]  = pOrt[3] * pHomoIn[0] + pOrt[4] * pHomoIn[4] + pOrt[5] * pHomoIn[8] ;
    pHomoOut[5]  = pOrt[3] * pHomoIn[1] + pOrt[4] * pHomoIn[5] + pOrt[5] * pHomoIn[9] ;
    pHomoOut[6]  = pOrt[3] * pHomoIn[2] + pOrt[4] * pHomoIn[6] + pOrt[5] * pHomoIn[10] ;
    pHomoOut[8]  = pOrt[6] * pHomoIn[0] + pOrt[7] * pHomoIn[4] + pOrt[8] * pHomoIn[8] ;
    pHomoOut[9]  = pOrt[6] * pHomoIn[1] + pOrt[7] * pHomoIn[5] + pOrt[8] * pHomoIn[9] ;
    pHomoOut[10] = pOrt[6] * pHomoIn[2] + pOrt[7] * pHomoIn[6] + pOrt[8] * pHomoIn[10] ;
}

void Algorithm::homoOrtMultiplyOrt(const double* pHomo, const double* pOrtIn, double* pOrtOut)
{
    pOrtOut[0] = pHomo[0] * pOrtIn[0] + pHomo[1] * pOrtIn[3] + pHomo[2] * pOrtIn[6] ;
    pOrtOut[1] = pHomo[0] * pOrtIn[1] + pHomo[1] * pOrtIn[4] + pHomo[2] * pOrtIn[7] ;
    pOrtOut[2] = pHomo[0] * pOrtIn[2] + pHomo[1] * pOrtIn[5] + pHomo[2] * pOrtIn[8] ;
    pOrtOut[3] = pHomo[4] * pOrtIn[0] + pHomo[5] * pOrtIn[3] + pHomo[6] * pOrtIn[6] ;
    pOrtOut[4] = pHomo[4] * pOrtIn[1] + pHomo[5] * pOrtIn[4] + pHomo[6] * pOrtIn[7] ;
    pOrtOut[5] = pHomo[4] * pOrtIn[2] + pHomo[5] * pOrtIn[5] + pHomo[6] * pOrtIn[8] ;
    pOrtOut[6] = pHomo[8] * pOrtIn[0] + pHomo[9] * pOrtIn[3] + pHomo[10] * pOrtIn[6] ;
    pOrtOut[7] = pHomo[8] * pOrtIn[1] + pHomo[9] * pOrtIn[4] + pHomo[10] * pOrtIn[7] ;
    pOrtOut[8] = pHomo[8] * pOrtIn[2] + pHomo[9] * pOrtIn[5] + pHomo[10] * pOrtIn[8] ;
}

void Algorithm::framePosAndOrtTrans(const Frame& frmSrc, const double dZoomPreTrans, const double* pdTransHomogeneous, const double dZoomPostTrans, double* pdDstFrameHomo)
{
    double dResult[8] = { 0.0 } ;
    dResult[0] = dZoomPreTrans * frmSrc.mOrigine.mData[0] ;
    dResult[1] = dZoomPreTrans * frmSrc.mOrigine.mData[1] ;
    dResult[2] = dZoomPreTrans * frmSrc.mOrigine.mData[2] ;

    framePosTrans(dResult, pdTransHomogeneous, dResult + 4) ;

    pdDstFrameHomo[3]    = dZoomPostTrans * dResult[4] ;
    pdDstFrameHomo[7]    = dZoomPostTrans * dResult[5] ;
    pdDstFrameHomo[11]   = dZoomPostTrans * dResult[6] ;

    frameOrt2HomoTrans(frmSrc.mOrientation.mData, pdTransHomogeneous, pdDstFrameHomo) ;
}

void Algorithm::framePosAndOrtTrans(const Frame& frmSrc, const double dZoomPreTrans, const double* pdTransHomogeneous, const double dZoomPostTrans, Frame& frmDst)
{
    double dResult[8] = { 0.0 };
    dResult[0] = dZoomPreTrans * frmSrc.mOrigine.mData[0];
    dResult[1] = dZoomPreTrans * frmSrc.mOrigine.mData[1];
    dResult[2] = dZoomPreTrans * frmSrc.mOrigine.mData[2];

    framePosTrans(dResult, pdTransHomogeneous, dResult + 4) ;

    frmDst.mOrigine.mData[0]  = dZoomPostTrans * dResult[4] ;
    frmDst.mOrigine.mData[1]  = dZoomPostTrans * dResult[5] ;
    frmDst.mOrigine.mData[2]  = dZoomPostTrans * dResult[6] ;

    frameOrientationTrans(frmSrc.mOrientation.mData, pdTransHomogeneous, frmDst.mOrientation.mData) ;
}

void Algorithm::computeHaption2EyeTransform(const double* dKuka2EyeTrans, const double* dKuka2HaptionTrans, double* dHaption2EyeTrans)
{
    double dKuka2HaptionTransInvers[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT] = { 0.0 } ;

    homogeneous4x4Invers(dKuka2HaptionTrans, dKuka2HaptionTransInvers) ;
    matrix4x4Multiply(dKuka2EyeTrans, dKuka2HaptionTransInvers, dHaption2EyeTrans) ;
}

int Algorithm::prepareTrajPlanCubicParam(double dPosStart, double dPosEnd, double dSpeedStart, double dSpeedEnd, double dDuration, double* pCubicParamC)
{
    double   dDuration2   = dDuration * dDuration ;
    double   dDuration3   = dDuration2 * dDuration ;
    pCubicParamC[0]       = dPosStart ;        //Start Pos
    pCubicParamC[1]       = dSpeedStart ;       //Start Speed
    pCubicParamC[2]       = (3.0 * (dPosEnd - dPosStart) - (2 * dSpeedStart + dSpeedEnd) * dDuration) / dDuration2 ;
    pCubicParamC[3]       = (2.0 * (dPosStart - dPosEnd) + (dSpeedStart + dSpeedEnd) * dDuration) / dDuration3 ;


    return 0 ;
}

int Algorithm::prepareTrajPlanCubicParam(const Frame& frmStart, const Frame& frmEnd, double dSpeedStart, double dSpeedEnd, double dDuration, double* pStartQuat, double* pEndQuat, double* pCubicParamC, int* pIsSlerpLinear, double* pSlerpTheta, double* pSlerpInvSinTheta)
{
    double dHomoStart[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT]      = { 0.0 } ;
    double dHomoEnd[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT]        = { 0.0 } ;
    double dSevenDataStart[8]                                  = { 0.0 } ;
    double dSevenDataEnd[8]                                    = { 0.0 } ;
    double dSevenDataEndOut[8]                                 = { 0.0 } ;
    frmStart.toHomo(dHomoStart) ;
    frmEnd.toHomo(dHomoEnd) ;
    homo2SevenData(dHomoStart, dSevenDataStart) ;
    homo2SevenData(dHomoEnd, dSevenDataEnd) ;

    prepareTrajPlanCubicParam(dSevenDataStart, dSevenDataEnd, dSpeedStart, dSpeedEnd, dDuration, pCubicParamC, pIsSlerpLinear, pSlerpTheta, pSlerpInvSinTheta, dSevenDataEndOut + 3) ;
    pStartQuat[0] = dSevenDataStart[3] ;
    pStartQuat[1] = dSevenDataStart[4] ;
    pStartQuat[2] = dSevenDataStart[5] ;
    pStartQuat[3] = dSevenDataStart[6] ;
    pEndQuat[0]   = dSevenDataEndOut[3] ;
    pEndQuat[1]   = dSevenDataEndOut[4] ;
    pEndQuat[2]   = dSevenDataEndOut[5] ;
    pEndQuat[3]   = dSevenDataEndOut[6] ;

    return 0 ;
}

int Algorithm::prepareTrajPlanCubicParam(const double* pSevenDataStart, const double* pSevenDataEnd, double dSpeedStart, double dSpeedEnd, double dDuration, double* pCubicParamC, int* pIsSlerpLinear, double* pSlerpTheta, double* pSlerpInvSinTheta, double* pQuatEndOut)
{
    double dCosTheta  = 0.0 ;
    double dSinTheta  = 0 ;

    prepareTrajPlanCubicParam(pSevenDataStart[0], pSevenDataEnd[0], dSpeedStart, dSpeedEnd, dDuration, pCubicParamC);
    prepareTrajPlanCubicParam(pSevenDataStart[1], pSevenDataEnd[1], dSpeedStart, dSpeedEnd, dDuration, pCubicParamC + CUBIC_TRAJ_PLAN_PARAM_CNT) ;
    prepareTrajPlanCubicParam(pSevenDataStart[2], pSevenDataEnd[2], dSpeedStart, dSpeedEnd, dDuration, pCubicParamC + 2 * CUBIC_TRAJ_PLAN_PARAM_CNT) ;

    dCosTheta        = pSevenDataStart[3] * pSevenDataEnd[3] + pSevenDataStart[4] * pSevenDataEnd[4] + pSevenDataStart[5] * pSevenDataEnd[5] + pSevenDataStart[6] * pSevenDataEnd[6] ;
    pQuatEndOut[0]   = pSevenDataEnd[3] ;
    pQuatEndOut[1]   = pSevenDataEnd[4] ;
    pQuatEndOut[2]   = pSevenDataEnd[5] ;
    pQuatEndOut[3]   = pSevenDataEnd[6] ;
    if (dCosTheta < 0.0)
    {
        pQuatEndOut[0]  = -pQuatEndOut[0] ;
        pQuatEndOut[1]  = -pQuatEndOut[1] ;
        pQuatEndOut[2]  = -pQuatEndOut[2] ;
        pQuatEndOut[3]  = -pQuatEndOut[3] ;
        dCosTheta = -dCosTheta ;
    }
    *pIsSlerpLinear = 0 ;

    if (dCosTheta > 0.999999)
    {
        *pIsSlerpLinear = 1 ;
    }
    else
    {
        dSinTheta                                        = sqrt(1.0 - dCosTheta * dCosTheta) ;
        *pSlerpTheta                                     = atan2(dSinTheta, dCosTheta) ;
        *pSlerpInvSinTheta                               = 1.0 / dSinTheta ;
        prepareTrajPlanCubicParam(0.0, 1.0, 0.0, 0.0, dDuration, pCubicParamC + 3 * CUBIC_TRAJ_PLAN_PARAM_CNT) ;
    }

    return 0 ;
}

int Algorithm::trajPlanCartesianCubic(const double* pCubicParamC, double dCurTime, double dDuration, const double* pStartQuat, const double* pEndQuat, int iIsSlerpLinear, double dSlerpTheta, double dSlerpInvSinTheta, Frame& frmNext, double* pSpeedNext)
{
    double dNextSevenData[7]                              = { 0.0 } ;
    double dNextHomo[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT]  = { 0.0 } ;
    trajPlanCartesianCubic(pCubicParamC, dCurTime, dDuration, pStartQuat, pEndQuat, iIsSlerpLinear, dSlerpTheta, dSlerpInvSinTheta, dNextSevenData, pSpeedNext) ;
    sevenData2Homo(dNextSevenData, dNextHomo) ;
    frmNext.fromHomo(dNextHomo) ;
    return 0 ;
}

int Algorithm::trajPlanCartesianCubic(const double* pCubicParamC, double dCurTime, double dDuration, const double* pStartQuat, const double* pEndQuat, int iIsSlerpLinear, double dSlerpTheta, double dSlerpInvSinTheta, double* pSevenDataNext, double* pSpeedNext)
{
    double dNextPos  = 0.0 ;
    double dK0       = 0.0 ;
    double dK1       = 0.0 ;

    trajPlanCartesianCubic(pCubicParamC, dCurTime, pSevenDataNext, pSpeedNext) ;
    trajPlanCartesianCubic(pCubicParamC + CUBIC_TRAJ_PLAN_PARAM_CNT, dCurTime, pSevenDataNext + 1, pSpeedNext + 1) ;
    trajPlanCartesianCubic(pCubicParamC + 2 * CUBIC_TRAJ_PLAN_PARAM_CNT, dCurTime, pSevenDataNext + 2, pSpeedNext + 2) ;

    if (1 == iIsSlerpLinear)
    {
        dK0 = (1 - dCurTime / dDuration) ;
        dK1 = dCurTime / dDuration ;
    }
    else
    {
        trajPlanCartesianCubic(pCubicParamC + 3 * CUBIC_TRAJ_PLAN_PARAM_CNT, dCurTime, &dNextPos, 0) ;
        dK0 = sin((1.0 - dNextPos) * dSlerpTheta) * dSlerpInvSinTheta ;
        dK1 = sin(dNextPos * dSlerpTheta) * dSlerpInvSinTheta ;
    }

    pSevenDataNext[3] = pStartQuat[0] * dK0 + pEndQuat[0] * dK1 ;
    pSevenDataNext[4] = pStartQuat[1] * dK0 + pEndQuat[1] * dK1 ;
    pSevenDataNext[5] = pStartQuat[2] * dK0 + pEndQuat[2] * dK1 ;
    pSevenDataNext[6] = pStartQuat[3] * dK0 + pEndQuat[3] * dK1 ;

    return 0 ;
}


int Algorithm::prepareTrajPlanCubicParam(const Frame& frmStart, const Frame& frmEnd, double dSpeedStart, double dSpeedEnd, double dDuration, double* pCubicParamC, double* pRotAxisVector)
{
    prepareTrajPlanCubicParam(frmStart.mOrigine.mData[0], frmEnd.mOrigine.mData[0], 0, 0, dDuration, pCubicParamC) ;
    prepareTrajPlanCubicParam(frmStart.mOrigine.mData[1], frmEnd.mOrigine.mData[1], 0, 0, dDuration, pCubicParamC + CUBIC_TRAJ_PLAN_PARAM_CNT) ;
    prepareTrajPlanCubicParam(frmStart.mOrigine.mData[2], frmEnd.mOrigine.mData[2], 0, 0, dDuration, pCubicParamC + 2 * CUBIC_TRAJ_PLAN_PARAM_CNT) ;

    double    dRotStartTranspose[9]  = { 0.0 } ;
    double    dRotTotal[9]           = { 0.0 } ;
    double    dTheta                 = 0.0 ;
    dRotStartTranspose[0] = frmStart.mOrientation.mData[0] ;
    dRotStartTranspose[1] = frmStart.mOrientation.mData[3] ;
    dRotStartTranspose[2] = frmStart.mOrientation.mData[6] ;
    dRotStartTranspose[3] = frmStart.mOrientation.mData[1] ;
    dRotStartTranspose[4] = frmStart.mOrientation.mData[4] ;
    dRotStartTranspose[5] = frmStart.mOrientation.mData[7] ;
    dRotStartTranspose[6] = frmStart.mOrientation.mData[2] ;
    dRotStartTranspose[7] = frmStart.mOrientation.mData[5] ;
    dRotStartTranspose[8] = frmStart.mOrientation.mData[8] ;

    matrix3x3Multiply(frmEnd.mOrientation.mData, dRotStartTranspose, dRotTotal) ;

    rot2axis((double*)dRotTotal, dTheta, pRotAxisVector) ;
    prepareTrajPlanCubicParam(0.0, dTheta, 0.0, 0.0, dDuration, pCubicParamC + 3 * CUBIC_TRAJ_PLAN_PARAM_CNT) ;

    return 0 ;
}

int Algorithm::trajPlanCartesianCubic(const double* pCubicParamC, double dCurTime, const double* pAxisVector, const double* pRotStart, Frame& frmNext, double* pSpeedNext)
{
    trajPlanCartesianCubic(pCubicParamC, dCurTime, frmNext.mOrigine.mData, 0) ;
    trajPlanCartesianCubic(pCubicParamC + CUBIC_TRAJ_PLAN_PARAM_CNT, dCurTime, frmNext.mOrigine.mData + 1, 0) ;
    trajPlanCartesianCubic(pCubicParamC + 2 * CUBIC_TRAJ_PLAN_PARAM_CNT, dCurTime, frmNext.mOrigine.mData + 2, 0) ;

    double dTheta        = 0 ;
    double dOrtTrans[9]  = { 0.0 } ;
    trajPlanCartesianCubic(pCubicParamC + 3 * CUBIC_TRAJ_PLAN_PARAM_CNT, dCurTime, &dTheta, 0) ;

    axis2rot(dTheta, (double*)pAxisVector, dOrtTrans) ;
    matrix3x3Multiply(dOrtTrans, pRotStart, frmNext.mOrientation.mData) ;

    return 0 ;
}

int Algorithm::trajPlanCartesianCubic(const double* pCubicParamC, double dCurTime, double* pPosNext, double* pSpeedNext)
{
    double  dCurTime2 = dCurTime * dCurTime;
    double  dCurTime3 = dCurTime2 * dCurTime;

    *pPosNext    = pCubicParamC[0] + pCubicParamC[1] * dCurTime + pCubicParamC[2] * dCurTime2 + pCubicParamC[3] * dCurTime3 ;
    if(NULL != pSpeedNext)
        *pSpeedNext  = pCubicParamC[1] + 2.0 * pCubicParamC[2] * dCurTime + 3 * pCubicParamC[3] * dCurTime2 ;

    return 0 ;
}

int Algorithm::prepareMotionFuseLinear(const double* pHomoA, const double* pHomoB, double* pOutDeltaPos, double* pOutDeltaAxisVector, double* pOutDeltaAxisTheta)
{
    double    dOrtBT[9]    = { 1.0 } ;
    double    dRotTotal[9] = { 0.0 } ;
    pOutDeltaPos[0]        = pHomoA[3]  - pHomoB[3] ;
    pOutDeltaPos[1]        = pHomoA[7]  - pHomoB[7] ;
    pOutDeltaPos[2]        = pHomoA[11] - pHomoB[11] ;
    dOrtBT[0]              = pHomoA[0] ;
    dOrtBT[1]              = pHomoA[4] ;
    dOrtBT[2]              = pHomoA[8] ;
    dOrtBT[3]              = pHomoA[1] ;
    dOrtBT[4]              = pHomoA[5] ;
    dOrtBT[5]              = pHomoA[9] ;
    dOrtBT[6]              = pHomoA[2] ;
    dOrtBT[7]              = pHomoA[6] ;
    dOrtBT[8]              = pHomoA[10] ;

    matrix3x3Multiply(pHomoA, dOrtBT, dRotTotal) ;
    rot2axis(dRotTotal, *pOutDeltaAxisTheta, pOutDeltaAxisVector) ;

    return 0 ;
}

int Algorithm::motionFuseLinear(const double* pHomoA, const double* pDeltaPos, const double* pDeltaAxisVector, double dDeltaAxisTheta, double dCurTickRatio, double* pOutHomoB)
{
    double    dOrtTrans[9]  = { 0.0 } ;
    double    dThetaT       = 0.0 ;

    pOutHomoB[3]            = pHomoA[3]  -  (1.0 - dCurTickRatio) * pDeltaPos[0] ;
    pOutHomoB[7]            = pHomoA[7]  -  (1.0 - dCurTickRatio) * pDeltaPos[1] ;
    pOutHomoB[11]           = pHomoA[11] -  (1.0 - dCurTickRatio) * pDeltaPos[2] ;

    dThetaT                 = (1.0 - dCurTickRatio) * dDeltaAxisTheta ;
    axis2rot(dThetaT, (double*)pDeltaAxisVector, (double*)dOrtTrans) ;
    ortHomoMultiplyHomo(dOrtTrans, pHomoA, pOutHomoB) ;

    pOutHomoB[12]           = 0.0 ;
    pOutHomoB[13]           = 0.0 ;
    pOutHomoB[14]           = 0.0 ;
    pOutHomoB[15]           = 1.0 ;

    return 0 ;
}

int  Algorithm::prepareMotionFuseCubic(const double* pHomoA, const double* pHomoB, const double dDuration, double* pOutDeltaAxisVector, double* pOutDeltaAxisTheta, double* pOutTransOrt, double* pCubicParam)
{
    double    dDeltaOrt[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 } ;
    double    dOrtT[9]     = { 0.0 } ;

    prepareTrajPlanCubicParam(pHomoB[3]  - pHomoA[3], 0.0, 0.0, 0.0, dDuration, pCubicParam) ;
    prepareTrajPlanCubicParam(pHomoB[7]  - pHomoA[7], 0.0, 0.0, 0.0, dDuration, pCubicParam + CUBIC_TRAJ_PLAN_PARAM_CNT) ;
    prepareTrajPlanCubicParam(pHomoB[11] - pHomoA[11], 0.0, 0.0, 0.0, dDuration, pCubicParam + 2 * CUBIC_TRAJ_PLAN_PARAM_CNT) ;
    dOrtT[0]                  = pHomoA[0] ;
    dOrtT[1]                  = pHomoA[4] ;
    dOrtT[2]                  = pHomoA[8] ;
    dOrtT[3]                  = pHomoA[1] ;
    dOrtT[4]                  = pHomoA[5] ;
    dOrtT[5]                  = pHomoA[9] ;
    dOrtT[6]                  = pHomoA[2] ;
    dOrtT[7]                  = pHomoA[6] ;
    dOrtT[8]                  = pHomoA[10] ;
    homoOrtMultiplyOrt(pHomoB, dOrtT, dDeltaOrt) ;
    pOutTransOrt[0]           = dDeltaOrt[0] ;
    pOutTransOrt[1]           = dDeltaOrt[3] ;
    pOutTransOrt[2]           = dDeltaOrt[6] ;
    pOutTransOrt[3]           = dDeltaOrt[1] ;
    pOutTransOrt[4]           = dDeltaOrt[4] ;
    pOutTransOrt[5]           = dDeltaOrt[7] ;
    pOutTransOrt[6]           = dDeltaOrt[2] ;
    pOutTransOrt[7]           = dDeltaOrt[5] ;
    pOutTransOrt[8]           = dDeltaOrt[8] ;
    rot2axis(pOutTransOrt, *pOutDeltaAxisTheta, pOutDeltaAxisVector) ;
    prepareTrajPlanCubicParam(0.0, *pOutDeltaAxisTheta, 0.0, 0.0, dDuration, pCubicParam + 3 * CUBIC_TRAJ_PLAN_PARAM_CNT) ;
    pOutTransOrt[0]           = dDeltaOrt[0] ;
    pOutTransOrt[1]           = dDeltaOrt[1] ;
    pOutTransOrt[2]           = dDeltaOrt[2] ;
    pOutTransOrt[3]           = dDeltaOrt[3] ;
    pOutTransOrt[4]           = dDeltaOrt[4] ;
    pOutTransOrt[5]           = dDeltaOrt[5] ;
    pOutTransOrt[6]           = dDeltaOrt[6] ;
    pOutTransOrt[7]           = dDeltaOrt[7] ;
    pOutTransOrt[8]           = dDeltaOrt[8] ;

    return 0 ;
}

int  Algorithm::motionFuseCubic(const double* pHomoA, const double* pDeltaAxisVector, const double* pDeltaTransOrt, const double* pCubicParam, double dCurTick, double* pOutHomoB)
{
    double    dDelta[4]      = { 0.0, 0.0, 0.0, 0.0 } ;
    double    dOrtTrans0[9]  = { 0.0 } ;
    double    dOrtTrans1[9]  = { 0.0 } ;
    trajPlanCartesianCubic(pCubicParam, dCurTick, dDelta, NULL) ;
    trajPlanCartesianCubic(pCubicParam + CUBIC_TRAJ_PLAN_PARAM_CNT, dCurTick, dDelta + 1, NULL) ;
    trajPlanCartesianCubic(pCubicParam + 2 * CUBIC_TRAJ_PLAN_PARAM_CNT, dCurTick, dDelta + 2, NULL) ;
    trajPlanCartesianCubic(pCubicParam + 3 * CUBIC_TRAJ_PLAN_PARAM_CNT, dCurTick, dDelta + 3, NULL) ;

    pOutHomoB[3]  = pHomoA[3]  + dDelta[0] ;
    pOutHomoB[7]  = pHomoA[7]  + dDelta[1] ;
    pOutHomoB[11] = pHomoA[11] + dDelta[2] ;

    axis2rot(dDelta[3], (double*)pDeltaAxisVector, dOrtTrans0) ;
    matrix3x3Multiply(dOrtTrans0, pDeltaTransOrt, dOrtTrans1) ;
    ortHomoMultiplyHomo(dOrtTrans1, pHomoA, pOutHomoB) ;
    pOutHomoB[12] = 0.0 ;
    pOutHomoB[13] = 0.0 ;
    pOutHomoB[14] = 0.0 ;
    pOutHomoB[15] = 1.0 ;

    return 0 ;
}
/* BEGIN: Modified by lin.lin, 2017/9/6 */

/*****************************************************************************
 Prototype    : Algorithm.IK_DH
 Description  : IK with DH parameters.
 Input        : const double* parrEndHomo: end homo matrix
                const double* pDH_param  : DH parameters; the unit of alpha and theta are radius. ; the address is relative to parrStartHomo
                const int nRow           : total rows of DH_param
 Output       : double* parrStartHomo    : destination DH matrix (4*4)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/12/04
    Author       : lin.lin
    Modification : Created function
  note: optimized the original ik algorithm, this version is recommanded!
*****************************************************************************/
void Algorithm::IK_DH(const double* parrEndHomo, const double* pDH_param, const int nRow, double* parrStartHomo)
{
    //cout << "func, line = " << __func__ << "  " << __LINE__ << endl;
    double dHomoCur[HOMO_MATRIX_ELEM_CNT]   = {0.0};        // T1*T2*T3*T4
    double dHomoDHInv[HOMO_MATRIX_ELEM_CNT] = {0.0};        // inv(T1234)
    double darrTi[HOMO_MATRIX_ELEM_CNT]  = {0.0};           // store  T1, T2, T3, T4

    // temp homoMatrix that store the middle result while calculating T1*T2*T3*T4
    double arrOriginTemp[HOMO_MATRIX_ELEM_CNT] =
       {1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1};

    // parrStartHomo = parrEndHomo * inv(T1*T2*T3*T4)
    for (int i = 0 ; i < nRow; ++i )
    {
        DHfromPostposition2Homo(pDH_param[i*4], pDH_param[i*4 +1], pDH_param[i*4 +2], pDH_param[i*4 +3], darrTi);
        matrix4x4Multiply(arrOriginTemp, darrTi, dHomoCur);
        memcpy(arrOriginTemp, dHomoCur, sizeof(double)*12);
    }
    homogeneous4x4Invers(dHomoCur, dHomoDHInv);
    matrix4x4Multiply(parrEndHomo, dHomoDHInv, parrStartHomo);
}

/*****************************************************************************
 Prototype    : Algorithm.FK_DH
 Description  :
 Input        : const double arrOrigin[4][4] : original DH matrix (4*4)
                const double DH_param[][4]   : DH parameters
                const int nRow               : total rows of DH_param
 Output       : arrDest[4][4] [out]          : destination DH matrix (4*4)
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/15
    Author       : lin.lin
    Modification : Created function
note: The overload version of FK_DH is Recommended.
*****************************************************************************/
//void Algorithm::FK_DH(const double arrOrigin[4][4], const double DH_param[][4], const int nRow, double arrDest[4][4])
//{
//    FK_DH((double*)arrOrigin, (double*)DH_param, nRow, (double*)arrDest);
//}
/*****************************************************************************
 Prototype    : Algorithm.FK_DH
 Description  :
 Input        : const Frame& OriginFrame: original Frame
                double DH_param[][4]    : DH parameters
                int nRow                : total rows of DH_param

 Output       : Frame& DestFrame        : destination Frame
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/6/20
    Author       : lin.lin
    Modification : Created function
note: overload functions of FK_DH
*****************************************************************************/
void Algorithm::FK_DH(const Frame& OriginFrame, const double DH_param[][4], const int nRow, Frame& DestFrame)
{
    double arrOrigin[16] = {0};
    double arrTi[16]     = {0};
    double arrTmulti[16] = {0};
    Frame T[10];

    int i;
    OriginFrame.toHomo(arrOrigin);
    for ( i = 0 ; i < nRow ; i++ )
    {
        T[i] = Frame::fromPostpositionDH(DH_param[i][1], DH_param[i][0] * DEG2RAD_1, DH_param[i][2], DH_param[i][3] * DEG2RAD_1);
        T[i].toHomo(arrTi);

        matrix4x4Multiply((double*)arrOrigin, (double*)arrTi, (double*)arrTmulti);
        memcpy(arrOrigin, arrTmulti, sizeof(double)*16);
    }
    DestFrame.fromHomo(arrTmulti);
}

// this function FK_DH is not recommended! It is just a historical left version.
// It is recommended use FK_DH with DH parameters. (see FK_DH above this function)
void Algorithm::FK_DH(const Frame& flangeFrame,
        const double& shaftLength, const double& clipLength, const double& jointLength,
        double pitch, double yaw, Frame& clipFrame
    )
{
    double d1, d2, a3, a4;
    d1 = 0;
    d2 = shaftLength;
    a3 = jointLength;
    a4 = clipLength / 2;
    // DH transform parameter. unit: degree.
    double DH_param[4][4] = {
          0,  0, d1, TOOL_ROLL_DEGREE_TO_FLANGE,
         90,  0, d2, 90,
        -90, a3, 0,  90+pitch*_180_DIV_PI,
          0, a4, 0,  yaw*_180_DIV_PI};

    Frame T1 = Frame::fromPostpositionDH(DH_param[0][1], DH_param[0][0] * DEG2RAD_1, DH_param[0][2], DH_param[0][3] * DEG2RAD_1);
    Frame T2 = Frame::fromPostpositionDH(DH_param[1][1], DH_param[1][0] * DEG2RAD_1, DH_param[1][2], DH_param[1][3] * DEG2RAD_1);
    Frame T3 = Frame::fromPostpositionDH(DH_param[2][1], DH_param[2][0] * DEG2RAD_1, DH_param[2][2], DH_param[2][3] * DEG2RAD_1);
    Frame T4 = Frame::fromPostpositionDH(DH_param[3][1], DH_param[3][0] * DEG2RAD_1, DH_param[3][2], DH_param[3][3] * DEG2RAD_1);

    clipFrame = flangeFrame * T1 * T2 * T3 * T4;
}

void Algorithm::FK_DH_Crag(const Frame& flangeFrame, const double& shaftLength, const double& clipLength, const double& jointLength, double pitch, double yaw, Frame& clipFrame)
{
  double d1, d2, a3, a4;
  d1 = shaftLength ;
  d2 = 0.0 ;
  a3 = jointLength;
  a4 = clipLength / 2 ;
  // DH transform parameter. unit: degree.
  double DH_param[4][4] = {
     0.0,     0.0,    d1,     TOOL_ROLL_DEGREE_TO_FLANGE,
     90.0,    0.0,    d2,     90.0 + pitch * _180_DIV_PI,
     90.0,    a3,     0.0,    yaw * _180_DIV_PI,
     0.0,     a4,     0.0,    0.0
  } ;

  Frame T1 = Frame::fromPrepositionDH(DH_param[0][1], DH_param[0][0] * DEG2RAD_1, DH_param[0][2], DH_param[0][3] * DEG2RAD_1) ;
  Frame T2 = Frame::fromPrepositionDH(DH_param[1][1], DH_param[1][0] * DEG2RAD_1, DH_param[1][2], DH_param[1][3] * DEG2RAD_1) ;
  Frame T3 = Frame::fromPrepositionDH(DH_param[2][1], DH_param[2][0] * DEG2RAD_1, DH_param[2][2], DH_param[2][3] * DEG2RAD_1) ;
  Frame T4 = Frame::fromPrepositionDH(DH_param[3][1], DH_param[3][0] * DEG2RAD_1, DH_param[3][2], DH_param[3][3] * DEG2RAD_1) ;

  clipFrame = flangeFrame * T1 * T2 * T3 * T4 ;
}

void Algorithm::FK_DH_Crag(const double* parOrigin, const double* pDH_param, const int nRow, double* parrDest)
{
  double darrTi[HOMO_MATRIX_ELEM_CNT] = { 0 };       // store  T1, T2, T3, T4
  double arrOriginTemp[HOMO_MATRIX_ELEM_CNT] = { 0 };
  memcpy(arrOriginTemp, parOrigin, sizeof(double) * 12);
  arrOriginTemp[15] = 1.0;

  for (int i = 0; i < nRow; ++i)
  {
    DHfromPreposition2Homo(pDH_param[i * 4], pDH_param[i * 4 + 1], pDH_param[i * 4 + 2], pDH_param[i * 4 + 3], darrTi) ;
    matrix4x4Multiply(arrOriginTemp, darrTi, parrDest) ;
    memcpy(arrOriginTemp, parrDest, sizeof(double) * 12) ;
  }
}

// this function FK_DH is not recommended! It is just a historical left version.
// It is recommended use FK_DH with DH parameters. (see FK_DH above this function)
void Algorithm::FK(const Frame& flangeFrame,
        const double& shaftLength, const double& clipLength, const double& jointLength,
        double pitch, double yaw, Frame& clipFrame
    )
{
    double T1[16] = {0};
    double T2[16] = {0};
    double T3[16] = {0};
    double T4[16] = {0};
    double T1234[16] = {0};
    double arrFlangeFrame[16] = {0};
    double arrClipFrame[16] = {0};
    double d1, d2, a3, a4;
    d1 = 0;
    d2 = shaftLength;
    a3 = jointLength;
    a4 = clipLength / 2;
    // DH transform parameter. unit: degree.
    double DH_param[4][4] = {
          0,  0, d1, TOOL_ROLL_DEGREE_TO_FLANGE,
         90,  0, d2, 90,
        -90, a3, 0,  90+pitch*_180_DIV_PI,
          0, a4, 0,  yaw*_180_DIV_PI};
    double Ttemp1[16] = {0};
    double Ttemp2[16] = {0};
    double Ttemp3[16] = {0};
    double arrEndowristHomo[16]={0};

    //caculate T1 = RotZ(TOOL_ROLL_DEGREE_TO_FLANGE)*TransZ(d1)
    double* rotZ1   = GeometryUtils::rotateZ2Homo(DH_param[0][3]*PI_DIV_180);
    double* transZ1 = GeometryUtils::transZ2Homo(DH_param[0][2]);
    matrix4x4Multiply(rotZ1, transZ1, (double*)T1);
    delete[] rotZ1;
    delete[] transZ1;

    //caculate T2 = RotZ(90)*TransZ(d2)*RotX(90)
    double* rotZ2   = GeometryUtils::rotateZ2Homo(DH_param[1][3]*PI_DIV_180);
    double* transZ2 = GeometryUtils::transZ2Homo(DH_param[1][2]);
    double* rotX2   = GeometryUtils::rotateX2Homo(DH_param[1][0]*PI_DIV_180);
    matrix4x4Multiply(rotZ2, transZ2, (double*)Ttemp1);
    matrix4x4Multiply((double*)Ttemp1, rotX2, (double*)T2);
    delete[] rotZ2;
    delete[] transZ2;
    delete[] rotX2;
    //caculate T3 = RotZ(90+pitch)*TransX(a3)*RotX(90)
    double* rotZ3   = GeometryUtils::rotateZ2Homo(DH_param[2][3]*PI_DIV_180);
    double* transX3 = GeometryUtils::transX2Homo(DH_param[2][1]);
    double* rotX3   = GeometryUtils::rotateX2Homo(DH_param[2][0]*PI_DIV_180);
    matrix4x4Multiply(rotZ3, transX3, (double*)Ttemp1);
    matrix4x4Multiply((double*)Ttemp1, rotX3, (double*)T3);
    delete[] rotZ3;
    delete[] transX3;
    delete[] rotX3;
    //caculate T4 = RotZ(yaw)*TransX(a4)
    double* rotZ4   = GeometryUtils::rotateZ2Homo(DH_param[3][3]*PI_DIV_180);
    double* transX4 = GeometryUtils::transX2Homo(DH_param[3][1]);
    matrix4x4Multiply(rotZ4, transX4, (double*)T4);
    delete[] rotZ4;
    delete[] transX4;

    // clipFrm = flangeFrm * T1 * T2 * T3 * T4;
    flangeFrame.toHomo(arrFlangeFrame);
    matrix4x4Multiply((double*)arrFlangeFrame, (double*)T1, (double*)Ttemp1);
    matrix4x4Multiply((double*)Ttemp1, (double*)T2, (double*)Ttemp2);
    matrix4x4Multiply((double*)Ttemp2, (double*)T3, (double*)Ttemp3);
    matrix4x4Multiply((double*)Ttemp3, (double*)T4, (double*)arrClipFrame);
    clipFrame.fromHomo(arrClipFrame);
}
/* END:   Modified by lin.lin, 2017/9/6   PN: */


/* BEGIN: Added by lin.lin, 2017/8/14 */
/*****************************************************************************
 Prototype    : Algorithm.eye
 Description  : create Identity matrix
 Input        : arrd[out]:     the nOrder-by-nOrder identity matrix
                int nOrder:    the order of Identity matrix; the nOrder-by-nOrder identity matrix
 Output       : None
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/14
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
void Algorithm::eye(double* arrd, int nOrder)
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
                arrd[i*nOrder+j] = 1.0;
            }else
            {
                arrd[i*nOrder+j] = 0.0;
            }
        }
    }
}

/*****************************************************************************
 Prototype    : Algorithm.QuatDiff
 Description  : get the difference between two quaternions
 Input        : const double* qSrc1: First quaternion
                const double* qSrc2: Second quaternion
 Output       :
 Return Value : the difference between two quaternions
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/11/29
    Author       : lin.lin
    Modification : Created function
  note: recommended!
*****************************************************************************/
double Algorithm::QuatDiff(const double* qSrc1, const double* qSrc2)
{
    double q2inv[4]   = {0};
    double q2invq1[4] = {0};
    double norm_inv;                                        // get inverse
    double dNorm1Sqr, dNorm2Sqr, dNorm2;                    // restore normalized value
    double darrQuatSrc1[4], darrQuatSrc2[4];

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

    CQuat::QuatMul(q2inv, darrQuatSrc1, q2invq1);
    double delt[4] = {1,0,0,0};
    delt[0] = q2invq1[0] - delt[0];
    delt[1] = q2invq1[1] - delt[1];
    delt[2] = q2invq1[2] - delt[2];
    delt[3] = q2invq1[3] - delt[3];
    // get variance
    double variance = delt[0]*delt[0] + delt[1]*delt[1] + delt[2]*delt[2] + delt[3]*delt[3];
    return variance;
}

/*****************************************************************************
 Prototype    : Algorithm.QuatDiff
 Description  : get the difference between two quaternions
 Input        : CQuat q1: First quaternion
                CQuat q2: Second quaternion

 Output       : CQuat &q2invq1: (q2)^(-1) * q1
 Return Value : the difference between two quaternions
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/17
    Author       : lin.lin
    Modification : Created function
  note: old version before optimized!
*****************************************************************************/
double Algorithm::QuatDiff(const CQuat &qSrc1, const CQuat &qSrc2, CQuat &q2invq1)
{
    CQuat q1 = qSrc1;
    CQuat q2 = qSrc2;

    // normalize
    q1.Normalize();
    q2.Normalize();
    CQuat q2inv = q2.Inverse();
    q2invq1 = q2inv * q1;

    CQuat delt(1,0,0,0);
    delt = q2invq1 - delt;
    // get variance
    double variance = delt(0)*delt(0) + delt(1)*delt(1) + delt(2)*delt(2) + delt(3)*delt(3);
    return variance;
}

/*****************************************************************************
 Prototype    : Algorithm.QuatDiff
 Description  : overload QuatDiff
 Input        : CQuat q1: First quaternion
                CQuat q2: Second quaternion

 Output       :
 Return Value : the difference between two quaternions
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/17
    Author       : lin.lin
    Modification : Created function
note: old version before optimized!
      run time: about 2 us, 20180103 version
*****************************************************************************/
double Algorithm::QuatDiff(const CQuat &qSrc1, const CQuat &qSrc2)
{
    CQuat q2invq1;
    return QuatDiff(qSrc1, qSrc2, q2invq1);
}

/*****************************************************************************
 Prototype    : Algorithm.sort
 Description  : Sort in ascending (default) or descending (Not yet designed) order
 Input        : pdSrcData:     selects data along which to sort.
                nTotalNumber:  data numbers
 Output       :
                pnSortIndex :  the same size as pdSrcData and describes the arrangement of the elements of pdSrcData into pdSortData along the sorted dimension.
                pdSortData  :  the sorted elements of pdSrcData along dimension nTotalNumber
                iFlag       :  ALG_SORT_ASCEND -  ascend order (default); ALG_SORT_DESCEND - descend order
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/8/23
    Author       : lin.lin
    Modification : Created function
note: 1, reference the help doc of matlab command "sort"
      2, depend on class classStlSort
*****************************************************************************/
void Algorithm::sort(const double* pdSrcData, int nTotalNumber, int* pnSortIndex, double* pdSortData, int iFlag)
{
    vector< classStlSort > vect;

    for (int i = 0; i < nTotalNumber; i++)
    {
        classStlSort myData(i + 1, pdSrcData[i]);
        vect.push_back(myData);
    }

    if  ( 0 == iFlag )
    {
        stable_sort(vect.begin(), vect.end());// Ensure that the original relative order of equal elements remains the same after sorting
    }else
    {
        stable_sort(vect.begin(), vect.end(), Algorithm::cmpDescend);
    }

    for (int i = 0; i < vect.size(); i++)
    {
        pnSortIndex[i] = vect[i].index;
        pdSortData[i]  = vect[i].dFuncValue;
    }
}

 bool Algorithm::cmpDescend(const classStlSort & m1, const classStlSort & m2)
 {
    return m1.dFuncValue > m2.dFuncValue;
 }

 //overload sort
void Algorithm::sort(vector< classStlSort > &vect)
{

}

/* END:   Added by lin.lin, 2017/8/14   PN: */

int Algorithm::eign4x4(double* pMatrix, double* pOutVec)
{
    int i, j, p, q, u, w, t, s, l ;
    double fm, cn, sn, omega, x, y, d ;
    l = 1 ;
    const int     N        = 4 ;
    const double  EPSILON  = 1e-10 ;
    const int     MAX_ITER = 1000 ;

    for (i = 0; i <= N - 1; i++)
    {
        pOutVec[i*N + i] = 1.0;
        for (j = 0; j <= N - 1; j++)
        {
            if (i != j)
            {
                pOutVec[i*N + j] = 0.0;
            }
        }
    }
    while (1)
    {
        fm = 0.0;
        for (i = 1; i <= N - 1; i++)
        {
            for (j = 0; j <= i - 1; j++)
            {
                d = fabs(pMatrix[i*N + j]);
                if ((i != j) && (d>fm))
                {
                    fm = d;
                    p = i;
                    q = j;
                }
            }
        }
        if (fm < EPSILON)
        {
            return 0 ;
        }
        if (l > MAX_ITER)
        {
            return -1 ;
        }
        l = l + 1;
        u = p * N + q;
        w = p * N + p;
        t = q * N + p;
        s = q * N + q;
        x = -pMatrix[u];
        y = (pMatrix[s] - pMatrix[w]) / 2.0;
        omega = x / sqrt(x*x + y*y);
        if (y < 0.0)
        {
            omega = -omega;
        }
        sn = 1.0 + sqrt(1.0 - omega * omega);
        sn = omega / sqrt(2.0 * sn);
        cn = sqrt(1.0 - sn * sn);
        fm = pMatrix[w];
        pMatrix[w] = fm * cn * cn + pMatrix[s] * sn * sn + pMatrix[u] * omega ;
        pMatrix[s] = fm * sn * sn + pMatrix[s] * cn * cn - pMatrix[u] * omega ;
        pMatrix[u] = 0.0;
        pMatrix[t] = 0.0;
        for (j = 0; j <= N - 1; j++)
        {
            if ((j != p) && (j != q))
            {
                u = p * N + j;
                w = q * N + j;
                fm = pMatrix[u];
                pMatrix[u] = fm * cn + pMatrix[w] * sn;
                pMatrix[w] = -fm * sn + pMatrix[w] * cn;
            }
        }
        for (i = 0; i <= N - 1; i++)
        {
            if ((i != p) && (i != q))
            {
                u = i * N + p;
                w = i * N + q;
                fm = pMatrix[u];
                pMatrix[u] = fm * cn + pMatrix[w] * sn;
                pMatrix[w] = -fm * sn + pMatrix[w] * cn;
            }
        }
        for (i = 0; i <= N - 1; i++)
        {
            u = i * N + p;
            w = i * N + q;
            fm = pOutVec[u];
            pOutVec[u] = fm * cn + pOutVec[w] * sn;
            pOutVec[w] = -fm * sn + pOutVec[w] * cn;
        }
    }

    return 0 ;
}

int Algorithm::eign4x4MaxVec(double* pMatrix, double* pOutVec, double* pMaxVec)
{
    int i    = 0 ;
    int iRes = eign4x4(pMatrix, pOutVec) ;
    if (0 == iRes)
    {
        double dMax     = fabs(pMatrix[0]) ;
        int    iMaxIdx  = 0 ;
        double dCur     = fabs(pMatrix[0]) ;

        for (i = 1; i < 4; i++)
        {
            dCur = fabs(pMatrix[i * 4 + i]) ;
            if (dCur > dMax)
            {
                dMax    = dCur ;
                iMaxIdx = i ;
            }
        }

        pMaxVec[0] = pOutVec[iMaxIdx] ;
        pMaxVec[1] = pOutVec[4 + iMaxIdx] ;
        pMaxVec[2] = pOutVec[8 + iMaxIdx] ;
        pMaxVec[3] = pOutVec[12 + iMaxIdx] ;
    }

    return iRes ;
}

int Algorithm::suppressFrame(const Frame& frmSrc, const Frame& frmDst, double dMaxPos, double dMaxOrtTheta, double dMaxThetaZ, Frame& frmOut)
{
    double  dHomoSrc[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT] = { 0.0 } ;
    double  dHomoDst[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT] = { 0.0 } ;
    double  dQuatSrc[4]                                  = { 0.0 } ;
    double  dQuatDst[7]                                  = { 0.0 } ;
    frmSrc.toHomo(dHomoSrc);
    frmDst.toHomo(dHomoDst);

    double   dOffSetX = dHomoDst[3] - dHomoSrc[3] ;
    double   dOffSetY = dHomoDst[7] - dHomoSrc[7] ;
    double   dOffSetZ = dHomoDst[11] - dHomoSrc[11] ;

    if (dOffSetX < -dMaxPos)
        dOffSetX = -dMaxPos ;
    else if (dOffSetX > dMaxPos)
        dOffSetX = dMaxPos ;

    if (dOffSetY < -dMaxPos)
        dOffSetY = -dMaxPos ;
    else if (dOffSetY > dMaxPos)
        dOffSetY = dMaxPos;

    if (dOffSetZ < -dMaxPos)
        dOffSetZ = -dMaxPos ;
    else if (dOffSetZ > dMaxPos)
        dOffSetZ = dMaxPos ;

    dHomoDst[3]  = dHomoSrc[3]  + dOffSetX ;
    dHomoDst[7]  = dHomoSrc[7]  + dOffSetY ;
    dHomoDst[11] = dHomoSrc[11] + dOffSetZ ;

    homo2Quat(dHomoSrc, dQuatSrc) ;
    homo2SevenData(dHomoDst, dQuatDst) ;

    double dTheta        = 0.0 ;
    double dThetaReal    = 0.0 ;
    double dCosTheta     = dQuatSrc[0] * dQuatDst[3] + dQuatSrc[1] * dQuatDst[4] + dQuatSrc[2] * dQuatDst[5] + dQuatSrc[3] * dQuatDst[6] ;
    double dSinTheta     = 0.0 ;
    double dInvSinTheta  = 0.0 ;
    if (dCosTheta < 0.0)
    {
        dQuatDst[3] = -dQuatDst[3] ;
        dQuatDst[4] = -dQuatDst[4] ;
        dQuatDst[5] = -dQuatDst[5] ;
        dQuatDst[6] = -dQuatDst[6] ;
        dCosTheta   = -dCosTheta ;
    }

    if (dCosTheta > 0.9999999999)
    {
        ;
    }
    else
    {
        dSinTheta     = sqrt(1.0 - dCosTheta * dCosTheta) ;
        dTheta        = atan2(dSinTheta, dCosTheta) ;
        dThetaReal    = dTheta ;
        dInvSinTheta  = 1.0 / dSinTheta ;
        if (dThetaReal > dMaxOrtTheta)
            dThetaReal = dMaxOrtTheta ;

        double dK0 = sin(dTheta - dThetaReal) * dInvSinTheta ;
        double dK1 = sin(dThetaReal) * dInvSinTheta ;

        dQuatDst[3] = dQuatSrc[0] * dK0 + dQuatDst[3] * dK1 ;
        dQuatDst[4] = dQuatSrc[1] * dK0 + dQuatDst[4] * dK1 ;
        dQuatDst[5] = dQuatSrc[2] * dK0 + dQuatDst[5] * dK1 ;
        dQuatDst[6] = dQuatSrc[3] * dK0 + dQuatDst[6] * dK1 ;

        sevenData2Homo(dQuatDst, dHomoDst) ;

        if (0.0 != dMaxThetaZ)
        {
            double dHomoSrcInv[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT] = { 0.0 } ;
            double dHomoTrans[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT]  = { 0.0 } ;
            double dRoll = 0.0, dPitch = 0.0, dYaw = 0.0 ;
            dOffSetX     = dHomoDst[3] ;
            dOffSetY     = dHomoDst[7] ;
            dOffSetZ     = dHomoDst[11] ;
            dHomoSrc[3]  = dHomoSrc[7] = dHomoSrc[11] = 0.0 ;
            dHomoDst[3]  = dHomoDst[7] = dHomoDst[11] = 0.0 ;
            dHomoSrc[15] = 1.0 ;
            dHomoDst[15] = 1.0 ;
            homogeneous4x4Invers(dHomoSrc, dHomoSrcInv) ;
            matrix4x4Multiply(dHomoSrcInv, dHomoDst, dHomoTrans) ;
            //homo2RPY(dHomoTrans, &dRoll, &dPitch, &dYaw) ;
            homo2RPY(dHomoTrans, dRoll, dPitch, dYaw);

            if (dRoll > dMaxThetaZ || dRoll < -dMaxThetaZ)
            {
                //printf("RPY: %.10f, %.10f, %.10f\n", dRoll, dPitch, dYaw) ;
                if (dRoll > dMaxThetaZ)
                    dRoll = dMaxThetaZ ;
                else if (dRoll < -dMaxThetaZ)
                    dRoll = -dMaxThetaZ ;

                rpy2Homo(dRoll, dPitch, dYaw, dHomoTrans) ;
                matrix4x4Multiply(dHomoSrc, dHomoTrans, dHomoDst) ;
            }
            dHomoDst[3]  = dOffSetX ;
            dHomoDst[7]  = dOffSetY ;
            dHomoDst[11] = dOffSetZ ;

            /*
            static unsigned int uCnt = 0;
            if (uCnt < 100)
            {
                printf("RPY: %.10f, %.10f, %.10f\n", dRoll, dPitch, dYaw) ;
                printf("%.10f, %.10f, %.10f, %.10f\n", dHomoDst[0], dHomoDst[1], dHomoDst[2], dHomoDst[3]) ;
                printf("%.10f, %.10f, %.10f, %.10f\n", dHomoDst[4], dHomoDst[5], dHomoDst[6], dHomoDst[7]) ;
                printf("%.10f, %.10f, %.10f, %.10f\n", dHomoDst[8], dHomoDst[9], dHomoDst[10], dHomoDst[11]) ;
                printf("%.10f, %.10f, %.10f, %.10f\n", dHomoDst[12], dHomoDst[13], dHomoDst[14], dHomoDst[15]) ;
            }
            uCnt++ ;
            */
        }
    }

    frmOut.fromHomo(dHomoDst) ;

    return 0 ;
}

int Algorithm::homo2RPY(const double* pHomo, double* pRoll, double* pPitch, double* pYaw)
{
    if (pHomo[8] < 1.0)
    {
        if (pHomo[8] > -1.0)
        {
            *pPitch = asin(-pHomo[8]) ;
            *pRoll  = atan2(pHomo[4], pHomo[0]) ;
            *pYaw   = atan2(pHomo[9], pHomo[10]) ;
        }
        else
        {
            *pPitch = DEG2RAD_90;
            *pRoll  = -atan2(-pHomo[6], pHomo[5]) ;
            *pYaw   = 0.0 ;
        }
    }
    else
    {
        *pPitch  = -DEG2RAD_90;
        *pRoll   = atan2(-pHomo[6], pHomo[5]) ;
        *pYaw    = 0.0 ;
    }

    return 0 ;
}

/*****************************************************************************
 Prototype    : GeometryUtils.homo2RPY
 Description  : get roll, pitch and yaw from homo matrix.
 Input        : const double* pHomo: source homo matrix
 Output       : roll  : roll(z); (unit: rad); [-PI, PI]
                pitch : pitch(y);(unit: rad); [-PI/2, PI/2]
                yaw   : yaw(x);  (unit: rad); [-PI, PI]
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/9/15
    Author       : lin.lin
    Modification : Created function
 note:
 1, reference: GeometryUtils::getRPYFromFrame;
 2, reference: matlab function: rotm2eul;
*****************************************************************************/
void Algorithm::homo2RPY(const double* pHomo, double& dRoll, double& dPitch, double& dYaw)
{
    //double thetaX, thetaY, thetaZ;
    double angle = pHomo[8];
    //if( angle < 1.0)
    if(1.0 - angle > (ALGORITHM_HOMO_EULER_EPS))
    {
        //if( angle > -1.0)
        if(angle + 1.0 > ALGORITHM_HOMO_EULER_EPS)
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
    if ( (dYaw+PI) < ALGORITHM_PI_EPS )
    {
        dYaw = PI;
    }
    if ( (dRoll+PI) < ALGORITHM_PI_EPS )
    {
        dRoll = PI;
    }
}

/*****************************************************************************
 Prototype    : Algorithm.rpy2Homo
 Description  : Convert Euler angles (by zyx sequences) to homogeneous matrix
 Input        : double dRoll     : Euler angles Roll (unit: rad)
                double dPitch    : Euler angles Pitch (unit: rad)
                double dYaw      : Euler angles Yaw (unit: rad)

 Output       : double* pHomoDst : homogeneous matrix converted from given Euler angles.
 Return Value : void
 Calls        :
 Called By    :

  History        :
  1.Date         : 2017/10/9
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
void Algorithm::rpy2Homo(double dRoll, double dPitch, double dYaw, double* pHomoDst)
{
    double dCx = cos(dYaw);
    double dCy = cos(dPitch);
    double dCz = cos(dRoll);
    double dSx = sin(dYaw);
    double dSy = sin(dPitch);
    double dSz = sin(dRoll);

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

inline void Algorithm::DHfromPreposition2Homo(const double alpha, const double a, const double d, const double theta, double* pHomoDst)
{
  double ct, st, ca, sa;
  ct = cos(theta);
  st = sin(theta);
  sa = sin(alpha);
  ca = cos(alpha);

  pHomoDst[0]   = ct ;
  pHomoDst[1]   = -st ;
  pHomoDst[2]   = 0.0 ;
  pHomoDst[3]   = a ;

  pHomoDst[4]   = st * ca ;
  pHomoDst[5]   = ct *ca ;
  pHomoDst[6]   = -sa ;
  pHomoDst[7]   = -sa * d ;

  pHomoDst[8]   = st * sa ;
  pHomoDst[9]   = ct * sa ;
  pHomoDst[10]  = ca ;
  pHomoDst[11]  = ca*d ;

  pHomoDst[12] = 0.0 ;
  pHomoDst[13] = 0.0 ;
  pHomoDst[14] = 0.0 ;
  pHomoDst[15] = 1.0 ;
}

/* BEGIN: Added by lin.lin, 2018/10/17 */
// called by svd
void Algorithm::ppp(double a[], double e[], double s[], double v[], int m, int n)
{
    int i,j,p,q;
    double d;
    if (m>=n) i=n;
    else i=m;
    for (j=1; j<=i-1; j++)
      { a[(j-1)*n+j-1]=s[j-1];
        a[(j-1)*n+j]=e[j-1];
      }
    a[(i-1)*n+i-1]=s[i-1];
    if (m<n) a[(i-1)*n+i]=e[i-1];
    for (i=1; i<=n-1; i++)
    for (j=i+1; j<=n; j++)
      { p=(i-1)*n+j-1; q=(j-1)*n+i-1;
        d=v[p]; v[p]=v[q]; v[q]=d;
      }
    return;
}
// called by svd
void Algorithm::sss(double fg[2], double cs[2])
{
    double r,d;
    if ((fabs(fg[0])+fabs(fg[1]))==0.0)
      { cs[0]=1.0; cs[1]=0.0; d=0.0;}
    else
      { d=sqrt(fg[0]*fg[0]+fg[1]*fg[1]);
        if (fabs(fg[0])>fabs(fg[1]))
          { d=fabs(d);
            if (fg[0]<0.0) d=-d;
          }
        if (fabs(fg[1])>=fabs(fg[0]))
          { d=fabs(d);
            if (fg[1]<0.0) d=-d;
          }
        cs[0]=fg[0]/d; cs[1]=fg[1]/d;
      }
    r=1.0;
    if (fabs(fg[0])>fabs(fg[1])) r=cs[1];
    else
      if (cs[0]!=0.0) r=1.0/cs[0];
    fg[0]=d; fg[1]=r;
    return;
}

/*****************************************************************************
 Prototype    : Algorithm.svd
 Description  : Singular value decomposition of general real matrices
 Input        : double* a : store real matrix A (m*n), and return a diagonal matrix with singular value in the diagonal position(Non-ascending order).
                int m:      the row number of A
                int n:      the column number of A
                double* u:  get the left singular vector U. (m*m)
                double* v:  get the right singular vector V'. (n*n)
                double eps: Given precision requirements.
                int ka:     max(m, n) + 1
 Output       : None
 Return Value : int:        <0: fail; >0: sucess.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/10/17
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
int Algorithm::svd(double* a, const int m, const int n, double* u, double* v, const double eps, const int ka)
{
    int i, j, k, l, it, ll, kk, ix, iy, mm, nn, iz, m1, ks;
    double d, dd, t, sm, sm1, em1, sk, ek, b, c, shh, fg[2], cs[2];
    double *s, *e, *w;

    s = (double *)malloc(ka * sizeof(double));
    e = (double *)malloc(ka * sizeof(double));
    w = (double *)malloc(ka * sizeof(double));
    it = 60; k = n;
    if (m - 1 < n)
        k = m - 1;
    l = m;
    if (n - 2 < m)
        l = n - 2;
    if (l < 0)
        l = 0;
    ll = k;
    if (l > k)
        ll = l;
    if (ll >= 1)
    {
        for (kk=1; kk<=ll; kk++)
        {
            if (kk <= k)
            {
                d = 0.0;
                for (i=kk; i<=m; i++)
                {
                    ix = (i-1)*n + kk - 1;
                    d  = d + a[ix] * a[ix];
                }
                s[kk-1] = sqrt(d);
                if (s[kk-1] != 0.0)
                {
                    ix = (kk-1) * n + kk - 1;
                    if (a[ix] != 0.0)
                    {
                        s[kk-1] = fabs(s[kk-1]);
                        if (a[ix]<0.0)
                            s[kk-1]=-s[kk-1];
                    }
                    for (i=kk; i<=m; i++)
                    {
                        iy    = (i-1)*n + kk - 1;
                        a[iy] = a[iy] / s[kk-1];
                    }
                    a[ix] = 1.0 + a[ix];
                }
                s[kk-1] = -s[kk-1];
            }
            if (n >= kk+1)
            {
                for (j=kk+1; j<=n; j++)
                {
                    if ((kk <= k)  &&  (s[kk-1] != 0.0))
                    {
                        d = 0.0;
                        for (i=kk; i<=m; i++)
                        {
                            ix = (i-1)*n + kk - 1;
                            iy = (i-1)*n + j - 1;
                            d  = d + a[ix] * a[iy];
                        }
                        d = -d  /  a[(kk-1) * n + kk - 1];
                        for (i=kk; i<=m; i++)
                        {
                            ix    = (i-1) * n + j  - 1;
                            iy    = (i-1) * n + kk - 1;
                            a[ix] = a[ix] + d * a[iy];
                        }
                    }
                    e[j-1] = a[(kk-1) * n + j - 1];
                }
            }
            if (kk <= k)
            {
                for (i=kk; i<=m; i++)
                {
                    ix = (i-1) * m + kk - 1;
                    iy = (i-1) * n + kk - 1;
                    u[ix] = a[iy];
                }
            }
            if (kk <= l)
            {
                d = 0.0;
                for (i=kk+1; i<=n; i++)
                  d = d + e[i-1] * e[i-1];
                e[kk-1] = sqrt(d);
                if (e[kk-1] != 0.0)
                {
                    if (e[kk] != 0.0)
                    {
                        e[kk-1] = fabs(e[kk-1]);
                        if (e[kk] < 0.0)
                            e[kk-1] = -e[kk-1];
                    }
                    for (i=kk+1; i<=n; i++)
                      e[i-1] = e[i-1] / e[kk-1];
                    e[kk] = 1.0 + e[kk];
                }
                e[kk-1] = -e[kk-1];
                if ((kk+1<=m) && (e[kk-1]!=0.0))
                {
                    for (i=kk+1; i<=m; i++)
                        w[i-1] = 0.0;
                    for (j=kk+1; j<=n; j++)
                      for (i=kk+1; i<=m; i++)
                        w[i-1] = w[i-1] + e[j-1]  *  a[(i-1) * n + j - 1];
                    for (j=kk+1; j<=n; j++)
                      for (i=kk+1; i<=m; i++)
                      {
                          ix    = (i-1)*n + j - 1;
                          a[ix] = a[ix] - w[i-1] * e[j-1] / e[kk];
                      }
                }
                for (i=kk+1; i<=n; i++)
                  v[(i-1) * n + kk - 1] = e[i-1];
            }
        }
    }
    mm = n;
    if (m+1 < n)
        mm = m + 1;
    if (k < n)
        s[k] = a[k * n + k];
    if (m < mm)
        s[mm-1] = 0.0;
    if (l+1 < mm)
        e[l] = a[l * n + mm - 1];
    e[mm-1] = 0.0;
    nn = m;
    if (m>n)
        nn = n;
    if (nn >= k+1)
    {
        for (j=k+1; j<=nn; j++)
        {
            for (i=1; i<=m; i++)
              u[(i-1)*m+j-1] = 0.0;
            u[(j-1)*m+j-1] = 1.0;
        }
    }
    if (k >= 1)
    {
        for (ll=1; ll<=k; ll++)
        {
            kk = k - ll + 1;
            iz = (kk-1) * m + kk - 1;
            if (s[kk-1] != 0.0)
            {
                if (nn >= kk+1)
                  for (j=kk+1; j<=nn; j++)
                  {
                      d = 0.0;
                      for (i=kk; i<=m; i++)
                      {
                          ix = (i-1) * m + kk - 1;
                          iy = (i-1) * m + j  - 1;
                          d  = d + u[ix] * u[iy] / u[iz];
                      }
                      d = -d;
                      for (i=kk; i<=m; i++)
                      {
                          ix    = (i-1) * m + j  - 1;
                          iy    = (i-1) * m + kk - 1;
                          u[ix] = u[ix] + d * u[iy];
                      }
                  }
                  for (i=kk; i<=m; i++)
                  {
                      ix    = (i-1) * m + kk - 1;
                      u[ix] = -u[ix];
                  }
                  u[iz] = 1.0 + u[iz];
                  if (kk-1 >= 1)
                    for (i=1; i<=kk-1; i++)
                      u[(i-1) * m + kk - 1] = 0.0;
            }
            else
            {
                for (i=1; i<=m; i++)
                  u[(i-1) * m + kk - 1] = 0.0;
                u[(kk-1) * m + kk - 1] = 1.0;
            }
        }
    }
    for (ll=1; ll<=n; ll++)
    {
        kk = n  - ll + 1;
        iz = kk * n  + kk - 1;
        if ((kk <= l)  &&  (e[kk - 1] != 0.0))
        {
            for (j=kk+1; j<=n; j++)
            {
                d = 0.0;
                for (i=kk+1; i<=n; i++)
                {
                    ix = (i-1) * n + kk - 1;
                    iy = (i-1) * n + j  - 1;
                    d  = d + v[ix] * v[iy] / v[iz];
                }
                d = -d;
                for (i=kk+1; i<=n; i++)
                {
                    ix    = (i-1) * n + j  - 1;
                    iy    = (i-1) * n + kk - 1;
                    v[ix] = v[ix] + d * v[iy];
                }
            }
        }
        for (i=1; i<=n; i++)
            v[(i-1) * n + kk - 1] = 0.0;
        v[iz-n] = 1.0;
    }
    for (i=1; i<=m; i++)
        for (j=1; j<=n; j++)
            a[(i-1) * n + j - 1] = 0.0;
    m1 = mm; it = 60;
    while (1 == 1)
    {
        if (mm == 0)
        {
            ppp(a, e, s, v, m, n);
            free(s); free(e); free(w);
            return(1);
        }
        if (it == 0)
        {
            ppp(a, e, s, v, m, n);
            free(s); free(e); free(w);
            return(-1);
        }
        kk = mm - 1;
        while ((kk!=0) && (fabs(e[kk-1])!=0.0))
        {
            d  = fabs(s[kk-1]) + fabs(s[kk]);
            dd = fabs(e[kk-1]);
            if (dd > eps*d)
                kk = kk - 1;
            else
                e[kk-1] = 0.0;
        }
        if (kk == mm-1)
        {
            kk = kk + 1;
            if (s[kk-1]<0.0)
            {
                s[kk-1] = -s[kk-1];
                for (i=1; i<=n; i++)
                {
                    ix   = (i-1) * n + kk - 1;
                    v[ix] = -v[ix];
                }
            }
            while ((kk!=m1) && (s[kk-1]<s[kk]))
            {
                d = s[kk-1];
                s[kk-1] = s[kk];
                s[kk] = d;
                if (kk < n)
                    for (i=1; i<=n; i++)
                    {
                        ix    = (i-1) * n + kk - 1;
                        iy    = (i-1) * n + kk;

                        d     = v[ix];
                        v[ix] = v[iy];
                        v[iy] = d;
                    }
                if (kk < m)
                    for (i=1; i<=m; i++)
                    {
                        ix = (i-1) * m + kk - 1;
                        iy = (i-1) * m + kk;

                        d     = u[ix];
                        u[ix] = u[iy];
                        u[iy] = d;
                    }
                kk = kk + 1;
            }
            it = 60;
            mm = mm - 1;
        }
        else
        {
            ks = mm;
            while ((ks>kk) && (fabs(s[ks-1])!=0.0))
            {
                d = 0.0;
                if (ks != mm)
                    d = d + fabs(e[ks-1]);
                if (ks != kk + 1)
                    d = d + fabs(e[ks-2]);
                dd = fabs(s[ks-1]);
                if (dd > eps*d)
                    ks = ks - 1;
                else
                    s[ks-1] = 0.0;
            }
            if (ks == kk)
            {
                kk = kk + 1;
                d = fabs(s[mm-1]);
                t = fabs(s[mm-2]);
                if (t > d)
                    d = t;
                t = fabs(e[mm-2]);
                if (t > d)
                    d = t;
                t = fabs(s[kk-1]);
                if (t > d)
                    d = t;
                t = fabs(e[kk-1]);
                if (t > d)
                    d = t;
                sm  = s[mm-1] / d;
                sm1 = s[mm-2] / d;

                em1 = e[mm-2] / d;

                sk = s[kk-1] / d;
                ek = e[kk-1] / d;

                b = ((sm1+sm)*(sm1-sm)+em1*em1) / 2.0;
                c   = sm * em1;
                c   = c * c;
                shh = 0.0;
                if ((b!=0.0) || (c!=0.0))
                {
                    shh = sqrt(b*b + c);
                    if (b < 0.0)
                        shh = -shh;
                    shh = c / (b + shh);
                }
                fg[0] = (sk+sm) * (sk-sm) - shh;
                fg[1] = sk * ek;
                for (i=kk; i<=mm-1; i++)
                  {
                    sss(fg, cs);
                    if (i != kk)
                        e[i-2] = fg[0];
                    fg[0]  = cs[0] * s[i-1] + cs[1] * e[i-1];
                    e[i-1] = cs[0] * e[i-1] - cs[1] * s[i-1];
                    fg[1]  = cs[1] * s[i];
                    s[i]   = cs[0] * s[i];
                    if ((cs[0]!=1.0) || (cs[1]!=0.0))
                        for (j=1; j<=n; j++)
                        {
                            ix    = (j-1) * n + i - 1;
                            iy    = (j-1) * n + i;
                            d     = cs[0] * v[ix] + cs[1] * v[iy];
                            v[iy] = -cs[1]* v[ix] + cs[0] * v[iy];
                            v[ix] = d;
                        }
                    sss(fg, cs);
                    s[i-1] =  fg[0];
                    fg[0]  =  cs[0] * e[i-1] + cs[1] * s[i];
                    s[i]   = -cs[1] * e[i-1] + cs[0] * s[i];
                    fg[1]  =  cs[1] * e[i];
                    e[i]   =  cs[0] * e[i];
                    if (i < m)
                        if ((cs[0]!=1.0) || (cs[1]!=0.0))
                            for (j=1; j<=m; j++)
                            {
                                ix = (j-1) * m + i - 1;
                                iy = (j-1) * m + i;
                                d  = cs[0] * u[ix] + cs[1] * u[iy];
                                u[iy] = -cs[1] * u[ix] + cs[0] * u[iy];
                                u[ix] = d;
                            }
                }
                e[mm-2] = fg[0];
                it      = it - 1;
            }
            else
            {
                if (ks == mm)
                {
                    kk      = kk + 1;
                    fg[1]   = e[mm - 2];
                    e[mm-2] = 0.0;
                    for (ll = kk; ll <= mm - 1; ll++)
                    {
                        i      = mm + kk - ll - 1;
                        fg[0]  = s[i-1];
                        sss(fg, cs);
                        s[i-1] = fg[0];
                        if (i != kk)
                        {
                            fg[1]  = -cs[1] * e[i - 2];
                            e[i-2] =  cs[0] * e[i - 2];
                        }
                        if ((cs[0] != 1.0)  ||  (cs[1] != 0.0))
                        {
                            for (j=1; j<=n; j++)
                            {
                                ix    =  (j-1) * n     + i     - 1;
                                iy    =  (j-1) * n     + mm    - 1;
                                d     =  cs[0] * v[ix] + cs[1] * v[iy];
                                v[iy] = -cs[1] * v[ix] + cs[0] * v[iy];
                                v[ix] = d;
                            }
                        }
                    }
                }
                else
                {
                    kk      = ks + 1;
                    fg[1]   = e[kk - 2];
                    e[kk-2] = 0.0;
                    for (i = kk; i <= mm; i++)
                    {
                        fg[0]  = s[i - 1];
                        sss(fg, cs);
                        s[i-1] =  fg[0];
                        fg[1]  = -cs[1] * e[i - 1];
                        e[i-1] =  cs[0] * e[i - 1];
                        if ((cs[0] != 1.0)  ||  (cs[1] != 0.0))
                        {
                            for (j=1; j<=m; j++)
                            {
                                ix    =  (j - 1) * m     + i     - 1;
                                iy    =  (j - 1) * m     + kk    - 2;
                                d     =  cs[0]   * u[ix] + cs[1] * u[iy];
                                u[iy] = -cs[1]   * u[ix] + cs[0] * u[iy];
                                u[ix] =  d;
                            }
                        }
                    }
                }
            }
        }
    }
    return(1);
}
/*****************************************************************************
 Prototype    : Algorithm.pinv
 Description  : get pseudoinverse of matrix(pinv) by svd.
 Input        : [in][out]double a[]:  store real matrix A, and return a diagonal matrix with singular value in the diagonal position(Non-ascending order). (m*n)
                int m:       the row number of A
                int n:       the column number of A
                double eps : Given precision requirements.
                int ka     : max(m, n) + 1

 Output       : double aa[]: get A+ (get the pseudoinverse of matrix of A). (n*m)
                double u[] : get the left singular vector U. (m*m)
                double v[] : get the right singular vector V'. (n*n)

 Return Value : int        : <0: fail; >0: sucess.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/10/17
    Author       : lin.lin
    Modification : Created function
  note: p99
*****************************************************************************/
int Algorithm::pinv(double a[], int m, int n, double aa[], double eps, double u[], double v[], int ka)
{
    typedef double T;
    int i, j, k, l, t, p, q, f;
    i = svd(a, m, n, u, v, eps, ka);
    if (i < 0)
    {
        return(-1);
    }
    j = n;
    if (m < n)
    {
        j = m;
    }
    j = j - 1;
    k = 0;
    while ((k<=j) && (a[k*n+k] != 0.0))
    {
        k = k + 1;
    }
    k = k - 1;
    for (i=0; i<=n-1; i++)
    {
        for (j=0; j<=m-1; j++)
        {
            t = i * m + j;
            aa[t] = 0.0;
            for (l=0; l<=k; l++)
            {
                  f = l * n + i;
                  p = j * m + l;
                  q = l * n + l;
                  aa[t] = aa[t] + v[f] * u[p] / a[q];
            }
        }
    }
    return(1);
}

/* END:   Added by lin.lin, 2018/10/17   PN: */

/*****************************************************************************
 Prototype    : Algorithm.cond
 Description  : Condition number with respect to inversion
 Input        : double* ptA: point to the matrix for measures
                const int m: the row number of A
                const int n  the column number of A
                const int p: the matrix condition number in p-norm (So far it's just 2)
 Output       : None
 Return Value : double:  returns the 2-norm condition number, the ratio of the largest singular value of X to the smallest.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2018/11/23
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
double Algorithm::cond(const double* ptA, const int m, const int n, const int p)
{
    typedef double T;

    // initial value
    const T eps    = 0.000001;
    int ka         = 0; // ka:     max(m, n) + 1
    int iDiagMaxId = 0;
    int i          = 0;
    T tCond        = 0.0;

    T* ptU = NULL;
    T* ptV = NULL;
    T* ptS = NULL;
    ptU = (T *)malloc(m * m * sizeof(T));
    ptV = (T *)malloc(n * n * sizeof(T));
    ptS = (T *)malloc(m * n * sizeof(T));
    memcpy(ptS, ptA, sizeof(T)*m*n);

    if ( m>=n )
    {
        ka         = m + 1;
        iDiagMaxId = n;
    }else
    {
        ka         = n + 1;
        iDiagMaxId = m;
    }

    i = Algorithm::svd((T *)ptS, m, n, ptU, ptV, eps, ka);
    if ( 2 == p )
    {
        tCond = ptS[0] / ptS[(iDiagMaxId-1)*n + iDiagMaxId - 1];
    }else
    {
        tCond = -1.0;  // reserved
    }

    free(ptU);
    free(ptV);
    free(ptS);

    return tCond;
}
void Algorithm::fk_dh_6joint(const double* theta, double* homo)
{
  double q4 = theta[3];

  double s1 = sin(theta[0]);
  double s2 = sin(theta[1]);
  double s3 = sin(theta[2]);
  double s5 = sin(theta[4]);
  double s6 = sin(theta[5]);

  double c1 = cos(theta[0]);
  double c2 = cos(theta[1]);
  double c3 = cos(theta[2]);
  double c5 = cos(theta[4]);
  double c6 = cos(theta[5]);

  double homoTmp[16] = {
   s6*(c3*s1 + c1*c2*s3) - 1.0*c6*(c5*(s1*s3 - 1.0*c1*c2*c3) + c1*s2*s5),               s6*(c5*(s1*s3 - 1.0*c1*c2*c3) + c1*s2*s5) + c6*(c3*s1 + c1*c2*s3), c1*c5*s2 - 1.0*s5*(s1*s3 - 1.0*c1*c2*c3), 0.00952*s6*(c3*s1 + c1*c2*s3) - 0.00952*c6*(c5*(s1*s3 - 1.0*c1*c2*c3) + c1*s2*s5) - 0.01125*c5*(s1*s3 - 1.0*c1*c2*c3) - 1.0*c1*q4*s2 - 0.01125*c1*s2*s5,
c6*(c5*(c1*s3 + c2*c3*s1) - 1.0*s1*s2*s5) - 1.0*s6*(c1*c3 - 1.0*c2*s1*s3), - 1.0*s6*(c5*(c1*s3 + c2*c3*s1) - 1.0*s1*s2*s5) - 1.0*c6*(c1*c3 - 1.0*c2*s1*s3),         s5*(c1*s3 + c2*c3*s1) + c5*s1*s2, 0.00952*c6*(c5*(c1*s3 + c2*c3*s1) - 1.0*s1*s2*s5) - 0.00952*s6*(c1*c3 - 1.0*c2*s1*s3) + 0.01125*c5*(c1*s3 + c2*c3*s1) - 1.0*q4*s1*s2 - 0.01125*s1*s2*s5,
                                         c6*(c2*s5 + c3*c5*s2) + s2*s3*s6,                                            c6*s2*s3 - 1.0*s6*(c2*s5 + c3*c5*s2),                     c3*s2*s5 - 1.0*c2*c5,                                                             c2*q4 + 0.01125*c2*s5 + 0.00952*c6*(c2*s5 + c3*c5*s2) + 0.01125*c3*c5*s2 + 0.00952*s2*s3*s6,
                                                                        0,                                                                               0,                                        0,                                                                                                                                                     1.0
  };
  memcpy(homo, homoTmp, sizeof(double)*16);

}
void Algorithm::fk_dh_7joint(const double* theta, const double dft, double* homo)
{
    double s1 = sin(theta[0]);
    double s2 = sin(theta[1]);
    double s3 = sin(theta[2]);
    double s4 = sin(theta[3]);
    double s5 = sin(theta[4]);
    double s6 = sin(theta[5]);
    double s7 = sin(theta[6]);
    double c1 = cos(theta[0]);
    double c2 = cos(theta[1]);
    double c3 = cos(theta[2]);
    double c4 = cos(theta[3]);
    double c5 = cos(theta[4]);
    double c6 = cos(theta[5]);
    double c7 = cos(theta[6]);

    double c12 = c1*c2;
    double c23 = c2*c3;
    double s12 = s1*s2;
    double s13 = s1*s3;
    double s23 = s2*s3;
    double c1s2 = c1*s2;
    double c3s2 = c3*s2;
    double c2s3 = c2*s3;
    double c12c3 = c12*c3;
    double c12s3 = c12*s3;
    double c23s1 = c23*s1;
    double c2s13 = c2*s13;

    double A = (0.9999993453*s1+0.001144255061*c1s2);
    double B = (0.9999993453*c1-0.001144255061*s12);
    double C = (0.001144255061*c2s3+c3s2);
    double D = (0.9999991589*c2-0.0000006986660608*c23+0.0006105859476*s23);

    double AA = (0.001144254848*s1-0.9999991589*c1s2+0.0006105859476*c3*A+0.0006105859476*c12s3);
    double AB = (0.001144254848*c1+0.9999991589*s12+0.0006105859476*c3*B-0.0006105859476*c2s13);
    double AC = (s3*B+c23s1);
    double AD = (s3*A-c12c3);
    double AE = (0.0006105851103*c2+0.001144254028*c23+0.001197173408*s4*C-0.001197173408*c4*D-0.999999097*s23);
    double AF = (c4*C+s4*D);

    double BA  = (s4*AB-c4*AC);
    double BB = (s4*AA-c4*AD);
    double BC = (s5*AE+c5*AF);
    double BD = (0.0000007309758867*c2+0.0000013698698*c23-0.9999980598*s4*C+0.9999980598*c4*D-0.00119717172*s23+0.001564355424*s5*AF-0.001564355424*c5*AE);
    double BE = (0.0000006986655601*c1-0.001197173408*c4*AB+0.0006105851103*s12-0.001197173408*s4*AC-0.999999097*c3*B+0.999999097*c2s13);
    double BF = (0.001197173408*c4*AA-0.0000006986655601*s1+0.0006105851103*c1s2+0.001197173408*s4*AD+0.999999097*c3*A+0.999999097*c12s3);

    double CA = (c5*BA+s5*BE);
    double CB = (s6*BD+c6*BC);
    double CC = (0.0000000008364234055*c1+0.9999980598*c4*AB+0.0000007309758867*s12+0.001564355424*s5*BA+0.9999980598*s4*AC-0.001564355424*c5*BE-0.00119717172*c3*B+0.00119717172*c2s13);
    double CD = (0.000000001143494912*c2+0.000000002142942297*c23-0.001564337092*s4*C+0.001564337092*c4*D-0.000001872783761*s23-0.004690729696*s6*BC-0.9999877749*s5*AF+0.004690729696*c6*BD+0.9999877749*c5*AE);
    double CE = (c5*BB-s5*BF);
    double CF = (0.0000000008364234055*s1+0.9999980598*c4*AA-0.0000007309758867*c1s2+0.001564355424*s5*BB+0.001564355424*c5*BF+0.9999980598*s4*AD-0.00119717172*c3*A-0.00119717172*c12s3);
    double CG = 0.000007337963183*s4*C-1.005207366e-11*c23-5.363884551e-12*c2-0.000007337963183*c4*D+0.000000008784819048*s23-0.9999889985*s6*BC+0.004690723956*s5*AF+0.9999889985*c6*BD-0.004690723956*c5*AE;

    double DA = (1.308450697e-12*c1+0.001564337092*c4*AB+0.000000001143494912*s12-0.9999877749*s5*BA+0.001564337092*s4*AC+0.9999877749*c5*BE-0.004690729696*s6*CA-0.000001872783761*c3*B+0.004690729696*c6*CC+0.000001872783761*c2s13);
    double DB = (s6*CC+c6*CA);
    double DC = (s6*CF+c6*CE);
    double DD = (0.000000001143494912*c1s2-0.001564337092*c4*AA-1.308450697e-12*s1+0.9999877749*s5*BB+0.004690729696*s6*CE+0.9999877749*c5*BF-0.001564337092*s4*AD+0.000001872783761*c3*A-0.004690729696*c6*CF+0.000001872783761*c12s3);
    double DE = 0.004690723956*s5*BA-0.000007337963183*c4*AB-5.363884551e-12*s12-6.137656063e-15*c1-0.000007337963183*s4*AC-0.004690723956*c5*BE-0.9999889985*s6*CA+0.000000008784819048*c3*B+0.9999889985*c6*CC-0.000000008784819048*c2s13;
    double DF = 6.137656063e-15*s1+0.000007337963183*c4*AA-5.363884551e-12*c1s2-0.004690723956*s5*BB+0.9999889985*s6*CE-0.004690723956*c5*BF+0.000007337963183*s4*AD-0.000000008784819048*c3*A-0.9999889985*c6*CF-0.000000008784819048*c12s3;
    double DG = 0.0004710323391*s1-0.3361991645*c4*AA-0.00007237*c12-0.070086011*s6*CF-0.09935784*s4*AA+0.3360242154*c1s2+0.00056353*c5*BB+0.00000984076331*s5*BB-0.070086011*c6*CE+0.07019922769*s6*CE+0.00000984076331*c5*BF+0.09935784*c4*AD-0.00056353*s5*BF-0.3361991645*s4*AD-0.001285010773*c3*A-0.09926035*s3*A-0.07019922769*c6*CF+0.09926035*c12c3-0.001285010773*c12s3;
    double DH = 0.070086011*s6*CC-0.0004710323391*c1+0.3361991645*c4*AB-0.00007237*c2*s1+0.09935784*s4*AB+0.3360242154*s12-0.00056353*c5*BA-0.09935784*c4*AC-0.00000984076331*s5*BA+0.3361991645*s4*AC+0.00000984076331*c5*BE-0.00056353*s5*BE+0.070086011*c6*CA-0.07019922769*s6*CA+0.001285010773*c3*B+0.09926035*s3*B+0.07019922769*c6*CC+0.09926035*c23s1-0.001285010773*c2s13;
    double DI = 0.3360242154*c2+0.00007237*s2+0.070086011*s6*BD-0.00056353*s5*AE-0.00000147038008*c23+0.09935784*c4*C-0.0001135791579*c2s3-0.09926035*c3s2-0.3361991645*s4*C+0.3361991645*c4*D+0.001285010773*s23+0.09935784*s4*D+0.070086011*c6*BC-0.07019922769*s6*BC-0.00056353*c5*AF-0.00000984076331*s5*AF+0.07019922769*c6*BD+0.00000984076331*c5*AE+0.3975;

    double FlangeHomo[16] = {
                 c7*DC-s7*DD,  -c7*DD-s7*DC,  DF,   DG,
                -c7*DB-s7*DA,   s7*DB-c7*DA,  DE,   DH,
                -s7*CD-c7*CB,   s7*CB-c7*CD,  CG,   DI,
                           0,             0,   0,   1};
    double homoBase[16] = {
                    1,0,0,0,
                    0,1,0,0,
                    0,0,1,dft,
                    0,0,0,1};
    //homo = FlangeHomo * homoBase;
    GeometryUtils::MatrixMultiply(FlangeHomo, homoBase, 4, 4, 4, homo);
}

void Algorithm::getJacobianEx_6joint(const double* theta, double* homo)
{
  double q4  = theta[3];
  double st1 = sin(theta[0]);
  double st2 = sin(theta[1]);
  double st3 = sin(theta[2]);

  double st5 = sin(theta[4]);
  double st6 = sin(theta[5]);

  double ct1 = cos(theta[0]);
  double ct2 = cos(theta[1]);
  double ct3 = cos(theta[2]);

  double ct5 = cos(theta[4]);
  double ct6 = cos(theta[5]);

  double JacobMatrix[6*6] = {
     ct1*(0.00952*ct3*st6 - 0.00001*ct5*st3*(952.0*ct6 + 1125.0)) - st1*(1.0*ct2*(0.00952*st3*st6 + 0.00001*ct3*ct5*(952.0*ct6 + 1125.0)) - st2*(q4 + 0.00001*st5*(952.0*ct6 + 1125.0))), -0.00001*ct1*(100000.0*ct2*q4 + 1125.0*ct2*st5 + 1125.0*ct3*ct5*st2 + 952.0*ct2*ct6*st5 + 952.0*st2*st3*st6 + 952.0*ct3*ct5*ct6*st2), 0.00952*ct1*ct2*ct3*st6 - 0.00952*st1*st3*st6 - 0.01125*ct1*ct2*ct5*st3 - 0.01125*ct3*ct5*st1 - 0.00952*ct3*ct5*ct6*st1 - 0.00952*ct1*ct2*ct5*ct6*st3, -1.0*ct1*st2, -0.00001*(952.0*ct6 + 1125.0)*(ct1*ct5*st2 - 1.0*st1*st3*st5 + ct1*ct2*ct3*st5),     (st5*(ct1*st3 + ct2*ct3*st1) + ct5*st1*st2)*(0.00952*ct6*(ct2*st5 + ct3*ct5*st2) + 0.00952*st2*st3*st6) + 1.0*(0.00952*ct6*(ct5*(ct1*st3 + ct2*ct3*st1) - 1.0*st1*st2*st5) - 0.00952*st6*(ct1*ct3 - 1.0*ct2*st1*st3))*(ct2*ct5 - 1.0*ct3*st2*st5),
 1.0*ct1*(1.0*ct2*(0.00952*st3*st6 + 0.00001*ct3*ct5*(952.0*ct6 + 1125.0)) - st2*(q4 + 0.00001*st5*(952.0*ct6 + 1125.0))) + st1*(0.00952*ct3*st6 - 0.00001*ct5*st3*(952.0*ct6 + 1125.0)), -0.00001*st1*(100000.0*ct2*q4 + 1125.0*ct2*st5 + 1125.0*ct3*ct5*st2 + 952.0*ct2*ct6*st5 + 952.0*st2*st3*st6 + 952.0*ct3*ct5*ct6*st2), 0.01125*ct1*ct3*ct5 + 0.00952*ct1*st3*st6 - 0.01125*ct2*ct5*st1*st3 + 0.00952*ct2*ct3*st1*st6 + 0.00952*ct1*ct3*ct5*ct6 - 0.00952*ct2*ct5*ct6*st1*st3, -1.0*st1*st2,     -0.00001*(952.0*ct6 + 1125.0)*(ct5*st1*st2 + ct1*st3*st5 + ct2*ct3*st1*st5), (st5*(st1*st3 - 1.0*ct1*ct2*ct3) - 1.0*ct1*ct5*st2)*(0.00952*ct6*(ct2*st5 + ct3*ct5*st2) + 0.00952*st2*st3*st6) - 1.0*(ct2*ct5 - 1.0*ct3*st2*st5)*(0.00952*st6*(ct3*st1 + ct1*ct2*st3) - 0.00952*ct6*(ct5*(st1*st3 - 1.0*ct1*ct2*ct3) + ct1*st2*st5)),
                                                                                                                                                                                       0,             0.01125*ct2*ct3*ct5 - 0.01125*st2*st5 - 1.0*q4*st2 + 0.00952*ct2*st3*st6 - 0.00952*ct6*st2*st5 + 0.00952*ct2*ct3*ct5*ct6,                                                                                     -0.00001*st2*(1125.0*ct5*st3 - 952.0*ct3*st6 + 952.0*ct5*ct6*st3),          ct2,                        0.00001*(ct2*ct5 - 1.0*ct3*st2*st5)*(952.0*ct6 + 1125.0),                                                                                                                                                                                   0.00952*ct6*st2*st3 - 0.00952*ct2*st5*st6 - 0.00952*ct3*ct5*st2*st6,
                                                                                                                                                                                       0,                                                                                                                                  st1,                                                                                                                                          -1.0*ct1*st2,            0,                                                           ct3*st1 + ct1*ct2*st3,                                                                                                                                                                                                     ct1*ct5*st2 - 1.0*st5*(st1*st3 - 1.0*ct1*ct2*ct3),
                                                                                                                                                                                       0,                                                                                                                             -1.0*ct1,                                                                                                                                          -1.0*st1*st2,            0,                                                       ct2*st1*st3 - 1.0*ct1*ct3,                                                                                                                                                                                                             st5*(ct1*st3 + ct2*ct3*st1) + ct5*st1*st2,
                                                                                                                                                                                     1.0,                                                                                                                                    0,                                                                                                                                                   ct2,            0,                                                                         st2*st3,                                                                                                                                                                                                                             ct3*st2*st5 - 1.0*ct2*ct5
  };
  memcpy(homo, JacobMatrix, sizeof(double)*6*6);
}

void Algorithm::getJacobianEx_7joint(const double* theta, const double dft, double* homo)
{
  double st1 = sin(theta[0]);
  double st2 = sin(theta[1]);
  double st3 = sin(theta[2]);
  double st4 = sin(theta[3]);
  double st5 = sin(theta[4]);
  double st6 = sin(theta[5]);
  double st7 = sin(theta[6]);

  double ct1 = cos(theta[0]);
  double ct2 = cos(theta[1]);
  double ct3 = cos(theta[2]);
  double ct4 = cos(theta[3]);
  double ct5 = cos(theta[4]);
  double ct6 = cos(theta[5]);
  double ct7 = cos(theta[6]);

  double JacobMatrix[6 * 7] = {
     ct1*(st3*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) - 0.1*st3 + ct3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) - 1.0*st1*(ct2*(0.1*ct3 - 1.0*ct3*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702)) + 0.33541)), 0.00001*ct1*(33541.0*ct2 + 33541.0*ct2*ct4 - 10000.0*ct3*st2 + 10000.0*ct2*st4 + 7020.0*ct2*ct4*ct6 + 10000.0*ct3*ct4*st2 + 7000.0*ct2*ct4*st6 - 33541.0*ct3*st2*st4 - 7020.0*ct3*ct6*st2*st4 - 7020.0*ct2*ct5*st4*st6 - 7000.0*ct3*st2*st4*st6 - 7000.0*ct6*st2*st3*st5 + 7020.0*st2*st3*st5*st6 + 100000.0*ct2*ct4*ct6*dft + 7000.0*ct2*ct5*ct6*st4 + 7000.0*ct3*ct4*ct5*ct6*st2 - 7020.0*ct3*ct4*ct5*st2*st6 - 100000.0*ct3*ct6*dft*st2*st4 - 100000.0*ct2*ct5*dft*st4*st6 + 100000.0*dft*st2*st3*st5*st6 - 100000.0*ct3*ct4*ct5*dft*st2*st6), -1.0*ct2*(st1*st2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702)) + 0.33541) - 1.0*ct1*(st3*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) - 0.1*st3 + ct3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + ct2*st1*(0.1*ct3 - 1.0*ct3*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702)))) - 1.0*st1*st2*(st2*(0.1*ct3 - 1.0*ct3*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) - 1.0*ct2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702)) + 0.33541)),                                                         (ct1*ct3 - 1.0*ct2*st1*st3)*(ct2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + ct3*st2*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) - 1.0*st2*st3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st2*st3*((ct1*st3 + ct2*ct3*st1)*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st5*(ct1*ct3 - 1.0*ct2*st1*st3)*(0.07*ct6 - 1.0*st6*(dft + 0.0702)) - 1.0*st1*st2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702)))), -0.0002*(351.0*st6 - 350.0*ct6 + 5000.0*dft*st6)*(ct3*ct5*st1 - 1.0*ct1*st2*st4*st5 - 1.0*ct4*st1*st3*st5 + ct1*ct2*ct5*st3 + ct1*ct2*ct3*ct4*st5),                                                                                                           1.0*(st5*(ct2*st4 + ct3*ct4*st2) + ct5*st2*st3)*(1.0*(0.07*ct6 - 1.0*st6*(dft + 0.0702))*(ct5*(ct4*(ct1*st3 + ct2*ct3*st1) - 1.0*st1*st2*st4) + st5*(ct1*ct3 - 1.0*ct2*st1*st3)) - (st4*(ct1*st3 + ct2*ct3*st1) + ct4*st1*st2)*(0.07*st6 + ct6*(dft + 0.0702))) - 1.0*((0.07*st6 + ct6*(dft + 0.0702))*(ct2*ct4 - 1.0*ct3*st2*st4) + (ct5*(ct2*st4 + ct3*ct4*st2) - 1.0*st2*st3*st5)*(0.07*ct6 - 1.0*st6*(dft + 0.0702)))*(st5*(ct4*(ct1*st3 + ct2*ct3*st1) - 1.0*st1*st2*st4) - 1.0*ct5*(ct1*ct3 - 1.0*ct2*st1*st3)),                                                                                                                                                          0,
      ct1*(ct2*(0.1*ct3 - 1.0*ct3*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702)) + 0.33541)) + st1*(st3*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) - 0.1*st3 + ct3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))), 0.00001*st1*(33541.0*ct2 + 33541.0*ct2*ct4 - 10000.0*ct3*st2 + 10000.0*ct2*st4 + 7020.0*ct2*ct4*ct6 + 10000.0*ct3*ct4*st2 + 7000.0*ct2*ct4*st6 - 33541.0*ct3*st2*st4 - 7020.0*ct3*ct6*st2*st4 - 7020.0*ct2*ct5*st4*st6 - 7000.0*ct3*st2*st4*st6 - 7000.0*ct6*st2*st3*st5 + 7020.0*st2*st3*st5*st6 + 100000.0*ct2*ct4*ct6*dft + 7000.0*ct2*ct5*ct6*st4 + 7000.0*ct3*ct4*ct5*ct6*st2 - 7020.0*ct3*ct4*ct5*st2*st6 - 100000.0*ct3*ct6*dft*st2*st4 - 100000.0*ct2*ct5*dft*st4*st6 + 100000.0*dft*st2*st3*st5*st6 - 100000.0*ct3*ct4*ct5*dft*st2*st6),               ct2*(st1*(st3*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) - 0.1*st3 + ct3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + ct1*st2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702)) + 0.33541) + ct1*ct2*(0.1*ct3 - 1.0*ct3*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702)))) + ct1*st2*(st2*(0.1*ct3 - 1.0*ct3*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) - 1.0*ct2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702)) + 0.33541)),                                                                 (ct3*st1 + ct1*ct2*st3)*(ct2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + ct3*st2*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) - 1.0*st2*st3*st5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st2*st3*((st1*st3 - 1.0*ct1*ct2*ct3)*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st5*(ct3*st1 + ct1*ct2*st3)*(0.07*ct6 - 1.0*st6*(dft + 0.0702)) + ct1*st2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702)))), -0.0002*(351.0*st6 - 350.0*ct6 + 5000.0*dft*st6)*(ct2*ct5*st1*st3 - 1.0*ct1*ct3*ct5 + ct1*ct4*st3*st5 - 1.0*st1*st2*st4*st5 + ct2*ct3*ct4*st1*st5),                                                                                                         -1.0*((0.07*st6 + ct6*(dft + 0.0702))*(st4*(st1*st3 - 1.0*ct1*ct2*ct3) - 1.0*ct1*ct4*st2) - 1.0*(st5*(ct3*st1 + ct1*ct2*st3) + ct5*(ct4*(st1*st3 - 1.0*ct1*ct2*ct3) + ct1*st2*st4))*(0.07*ct6 - 1.0*st6*(dft + 0.0702)))*(st5*(ct2*st4 + ct3*ct4*st2) + ct5*st2*st3) - 1.0*(st5*(ct4*(st1*st3 - 1.0*ct1*ct2*ct3) + ct1*st2*st4) - 1.0*ct5*(ct3*st1 + ct1*ct2*st3))*((0.07*st6 + ct6*(dft + 0.0702))*(ct2*ct4 - 1.0*ct3*st2*st4) + (ct5*(ct2*st4 + ct3*ct4*st2) - 1.0*st2*st3*st5)*(0.07*ct6 - 1.0*st6*(dft + 0.0702))),                                                                                                                                                           0,
      0,                                                                      0.1*ct2*ct3*ct4 - 0.1*ct2*ct3 - 0.33541*ct4*st2 - 0.1*st2*st4 - 0.33541*st2 - 0.33541*ct2*ct3*st4 - 0.0702*ct4*ct6*st2 - 0.07*ct4*st2*st6 - 0.07*ct2*ct3*st4*st6 - 0.07*ct2*ct6*st3*st5 - 0.07*ct5*ct6*st2*st4 + 0.0702*ct2*st3*st5*st6 + 0.0702*ct5*st2*st4*st6 - 0.0702*ct2*ct3*ct6*st4 - 1.0*ct4*ct6*dft*st2 + 0.07*ct2*ct3*ct4*ct5*ct6 - 0.0702*ct2*ct3*ct4*ct5*st6 - 1.0*ct2*ct3*ct6*dft*st4 + ct2*dft*st3*st5*st6 + ct5*dft*st2*st4*st6 - 1.0*ct2*ct3*ct4*ct5*dft*st6,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0.00001*st2*(10000.0*st3 - 10000.0*ct4*st3 + 33541.0*st3*st4 - 7000.0*ct3*ct6*st5 + 7020.0*ct6*st3*st4 + 7020.0*ct3*st5*st6 + 7000.0*st3*st4*st6 + 7020.0*ct4*ct5*st3*st6 + 100000.0*ct6*dft*st3*st4 + 100000.0*ct3*dft*st5*st6 - 7000.0*ct4*ct5*ct6*st3 + 100000.0*ct4*ct5*dft*st3*st6), (ct3*st1 + ct1*ct2*st3)*((ct1*st3 + ct2*ct3*st1)*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st5*(ct1*ct3 - 1.0*ct2*st1*st3)*(0.07*ct6 - 1.0*st6*(dft + 0.0702)) - 1.0*st1*st2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702)))) - 1.0*(ct1*ct3 - 1.0*ct2*st1*st3)*((st1*st3 - 1.0*ct1*ct2*ct3)*(0.1*ct4 - 1.0*st4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct4*ct5*(0.07*ct6 - 1.0*st6*(dft + 0.0702))) + st5*(ct3*st1 + ct1*ct2*st3)*(0.07*ct6 - 1.0*st6*(dft + 0.0702)) + ct1*st2*(0.1*st4 + ct4*(0.07*st6 + ct6*(dft + 0.0702) + 0.33541) + ct5*st4*(0.07*ct6 - 1.0*st6*(dft + 0.0702)))),                                                      0.0002*(351.0*st6 - 350.0*ct6 + 5000.0*dft*st6)*(ct5*st2*st3 + ct2*st4*st5 + ct3*ct4*st2*st5), -1.0*((0.07*st6 + ct6*(dft + 0.0702))*(st4*(st1*st3 - 1.0*ct1*ct2*ct3) - 1.0*ct1*ct4*st2) - 1.0*(st5*(ct3*st1 + ct1*ct2*st3) + ct5*(ct4*(st1*st3 - 1.0*ct1*ct2*ct3) + ct1*st2*st4))*(0.07*ct6 - 1.0*st6*(dft + 0.0702)))*(st5*(ct4*(ct1*st3 + ct2*ct3*st1) - 1.0*st1*st2*st4) - 1.0*ct5*(ct1*ct3 - 1.0*ct2*st1*st3)) - (st5*(ct4*(st1*st3 - 1.0*ct1*ct2*ct3) + ct1*st2*st4) - 1.0*ct5*(ct3*st1 + ct1*ct2*st3))*(1.0*(0.07*ct6 - 1.0*st6*(dft + 0.0702))*(ct5*(ct4*(ct1*st3 + ct2*ct3*st1) - 1.0*st1*st2*st4) + st5*(ct1*ct3 - 1.0*ct2*st1*st3)) - (st4*(ct1*st3 + ct2*ct3*st1) + ct4*st1*st2)*(0.07*st6 + ct6*(dft + 0.0702))),                                                                                                                                                           0,
      0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -1.0*st1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         ct1*st2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           -1.0*ct3*st1 - 1.0*ct1*ct2*st3,                                                                                                  ct1*ct4*st2 - 1.0*st4*(st1*st3 - 1.0*ct1*ct2*ct3),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           st5*(ct4*(st1*st3 - 1.0*ct1*ct2*ct3) + ct1*st2*st4) - 1.0*ct5*(ct3*st1 + ct1*ct2*st3), -1.0*st6*(st5*(ct3*st1 + ct1*ct2*st3) + ct5*(ct4*(st1*st3 - 1.0*ct1*ct2*ct3) + ct1*st2*st4)) - 1.0*ct6*(st4*(st1*st3 - 1.0*ct1*ct2*ct3) - 1.0*ct1*ct4*st2),
      0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              ct1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         st1*st2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 ct1*ct3 - 1.0*ct2*st1*st3,                                                                                                          st4*(ct1*st3 + ct2*ct3*st1) + ct4*st1*st2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       ct5*(ct1*ct3 - 1.0*ct2*st1*st3) - 1.0*st5*(ct4*(ct1*st3 + ct2*ct3*st1) - 1.0*st1*st2*st4),               st6*(ct5*(ct4*(ct1*st3 + ct2*ct3*st1) - 1.0*st1*st2*st4) + st5*(ct1*ct3 - 1.0*ct2*st1*st3)) + ct6*(st4*(ct1*st3 + ct2*ct3*st1) + ct4*st1*st2),
      1.0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             ct2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   st2*st3,                                                                                                                          ct2*ct4 - 1.0*ct3*st2*st4,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       st5*(ct2*st4 + ct3*ct4*st2) + ct5*st2*st3,                                                                   ct6*(ct2*ct4 - 1.0*ct3*st2*st4) - 1.0*st6*(ct5*(ct2*st4 + ct3*ct4*st2) - 1.0*st2*st3*st5) };
  memcpy(homo, JacobMatrix, sizeof(double)*6*7);
}

double Algorithm::getGainOfJacobTranspose(const double* J, const int iDof, const int iJointNum, const double* e)
{
    double gain = 0;
    double JT[iJointNum*iDof] = {0};
    double JTXe[iDof] = {0};
    double JJTe[iDof] = {0};
    int i = 0;
    GeometryUtils::MatrixTranspose(J, iDof, iJointNum, JT);
    // get JJTe = J * JT * e;
    GeometryUtils::MatrixMultiply(JT , e, iJointNum, iDof, 1, JTXe);
    GeometryUtils::MatrixMultiply(J , JTXe, iDof, iJointNum, 1, JJTe);
    // get gain
    double dot1 = 0, dot2 = 0;
    for ( i = 0 ; i <iDof ; ++i )
    {
        dot1 += e[i]*JJTe[i];
        dot2 += JJTe[i]*JJTe[i];
    }
    gain = dot1 / dot2;
    return gain;
}

/*****************************************************************************
 Prototype    : Algorithm.getJointVelocityByJT
 Description  : get joint velocity by Jacobian transpose method
 Input        : const double* desiredCartesianVector:desired position and pose in Cartesian space.
                double* q: actual q (unit: radian)
                double* dq: joint velocity (unit: radian/s)
 Output       : None
 Return Value : iInCounter, success; -1 failed.
 Calls        :
 Called By    :

  History        :
  1.Date         : 2020/1/17
    Author       : lin.lin
    Modification : Created function
  note: 7 joint case.
*****************************************************************************/
int Algorithm::getJointVelocityByJT(const double* desiredCartesianVector, double* q, double* dq)
{
  const int iDof = 6;
  const int iJointNum = 7;
  double xe[iDof] = {0};
  double eps[2] = {1e-4, 1};  // m, degree
  eps[1] *= PI/180.0;
  // local init
  int jMaxNum  = 20000;
  double dft = 0;
  int iInCounter = 0;
  double Tact[16] = {
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1};
  double Tbase[16] = {
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1};
  int ret = -1;

  double qOld[iJointNum] = {0};
  double dqTmp[iJointNum]   = {0};
  double tarrEndRot[9] = {0};
  double tarrErr[6] = {0};
  double tarrErrTmp[6] = { 0 };
  double EndEffectorJacobian[iDof*iJointNum] = {0}; // 6*7
  double JT[iJointNum*iDof] = {0}; // 7*6
  double gain = 0;
  int i = 0, j = 0, k = 0;
  double qeRadNew[iJointNum] = {0};
  double tarrQd[4] = {0};
  double tarrQe[4] = {0};

  memcpy(qOld, q, sizeof(double)*iJointNum);
  for ( j = 1; j <=jMaxNum ; ++j )
  {
      iInCounter++;

      // get new e
      fk_dh_7joint(q, dft, Tact);
      xe[0] = Tact[3];
      xe[1] = Tact[7];
      xe[2] = Tact[11];
      tarrErr[0] = desiredCartesianVector[0] - xe[0];
      tarrErr[1] = desiredCartesianVector[1] - xe[1];
      tarrErr[2] = desiredCartesianVector[2] - xe[2];

      eul2rotm((double*)desiredCartesianVector+3, tarrEndRot);
      rotation2Quat(tarrEndRot, tarrQd);
      t2r(Tact, tarrEndRot);
      rotation2Quat(tarrEndRot, tarrQe);
      getErrO(tarrQd, tarrQe, tarrErr+3);

      // get new dq
      getJacobianEx_7joint(q, dft, EndEffectorJacobian);
      gain = getGainOfJacobTranspose(EndEffectorJacobian, iDof, iJointNum, tarrErr);
      for ( i = 0 ; i < iDof ; ++i )
      {
          tarrErrTmp[i] = tarrErr[i] * gain;
          if ( i>2 )
          {
              tarrErrTmp[i] = tarrErrTmp[i] / 5.0;
          }
      }
      GeometryUtils::MatrixTranspose(EndEffectorJacobian, iDof, iJointNum, JT);
      GeometryUtils::MatrixMultiply(JT, tarrErrTmp, iJointNum, iDof, 1, dqTmp);

      // get new q
      for (k = 0 ; k <iJointNum ; ++k )
      {
        q[k] += dqTmp[k];
      }

      // Iteration termination condition
      double dNormP = Norm(tarrErr);
      double dNormO = Norm(tarrErr+3);
      if ( dNormP < eps[0] && dNormO < eps[1] )
      {
        ret = iInCounter;
        break;
      }
  }
  for ( i = 0 ; i <iJointNum ; ++i )
  {
      dq[i] = q[i] - qOld[i];
  }

  return ret;
}
/*****************************************************************************
 Prototype    : getGainOfJacobTranspose
 Description  :
 Input        : J: the Jacobian Matrix (6*6)
                e: the error of position and gesture, in Cartesian space (6*1)
 Output       : None
 Return Value : the gain (6*1)
 Calls        :
 Called By    :

  History        :
  1.Date         : 2019/12/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
Matrix<double, 6, 1> getGainOfJacobTranspose(Matrix<double, 6, 6> J, Matrix<double, 6, 1> e)
{
  Matrix<double, 6, 1> output;

  return output;
}



/*****************************************************************************
 Prototype    : getDqByJacobTranspose
 Description  : get dot q by Jacobian transpose method
 Input        : K: gain of Jacobian transpose mathod(6*1)
                J: the Jacobian Matrix (6*6)
                e: the error of position and gesture, in Cartesian space (6*1)
 Output       : None
 Return Value : dot q (6*1)
 Calls        :
 Called By    :

  History        :
  1.Date         : 2019/12/16
    Author       : lin.lin
    Modification : Created function

*****************************************************************************/
Matrix<double, 6, 1> getDqByJacobTranspose(Matrix<double, 6, 1> K, Matrix<double, 6, 6> J, Matrix<double, 6, 1> e)
{
  Matrix<double, 6, 1> dq;

  return dq;
}

#ifndef  HOMO_2_QUAT
#define  HOMO_2_QUAT(pSrcHomoData, pDstQuatData)                                     \
    double  dTrace = pSrcHomoData[0] + pSrcHomoData[5] + pSrcHomoData[10] ;          \
    dTrace = (dTrace < -1.0 ? -1.0 : dTrace) ;                                       \
    double  dTraceSqrt = 0.5 * sqrt(dTrace + 1.0) ;                                  \
    double  dKx = pSrcHomoData[9] - pSrcHomoData[6] ;                                \
    double  dKy = pSrcHomoData[2] - pSrcHomoData[8] ;                                \
    double  dKz = pSrcHomoData[4] - pSrcHomoData[1] ;                                \
    double  dKx1 = 0.0, dKy1 = 0.0, dKz1 = 0.0 ;                                     \
    bool    bAdd = true;                                                             \
    double  dNorm = 0.0;                                                             \
    double  dScalar = 0.0;                                                           \
    if (pSrcHomoData[0] >= pSrcHomoData[5] && pSrcHomoData[0] >= pSrcHomoData[10])   \
    {                                                                                \
        dKx1 = pSrcHomoData[0] - pSrcHomoData[5] - pSrcHomoData[10] + 1.0;           \
        dKy1 = pSrcHomoData[4] + pSrcHomoData[1];                                    \
        dKz1 = pSrcHomoData[8] + pSrcHomoData[2];                                    \
        bAdd = (dKx >= 0.0);                                                         \
    }                                                                                \
    else if (pSrcHomoData[5] >= pSrcHomoData[10])                                    \
    {                                                                                \
        dKx1 = pSrcHomoData[4] + pSrcHomoData[1];                                    \
        dKy1 = pSrcHomoData[5] - pSrcHomoData[0] - pSrcHomoData[10] + 1.0;           \
        dKz1 = pSrcHomoData[9] + pSrcHomoData[6];                                    \
        bAdd = (dKy >= 0.0);                                                         \
    }                                                                                \
    else                                                                             \
    {                                                                                \
        dKx1 = pSrcHomoData[8] + pSrcHomoData[2];                                    \
        dKy1 = pSrcHomoData[9] + pSrcHomoData[6];                                    \
        dKz1 = pSrcHomoData[10] - pSrcHomoData[0] - pSrcHomoData[5] + 1.0;           \
        bAdd = (dKz >= 0.0);                                                         \
    }                                                                                \
    if (true == bAdd)                                                                \
    {                                                                                \
        dKx += dKx1;                                                                 \
        dKy += dKy1;                                                                 \
        dKz += dKz1;                                                                 \
    }                                                                                \
    else                                                                             \
    {                                                                                \
        dKx -= dKx1;                                                                 \
        dKy -= dKy1;                                                                 \
        dKz -= dKz1;                                                                 \
    }                                                                                \
    dNorm = sqrt(dKx * dKx + dKy * dKy + dKz * dKz);                                 \
    if (dNorm <= 1e-15)                                                              \
    {                                                                                \
        pDstQuatData[0] = 0.0f;                                                      \
        pDstQuatData[1] = 0.0f;                                                      \
        pDstQuatData[2] = 0.0f;                                                      \
        pDstQuatData[3] = 1.0f;                                                      \
    }                                                                                \
    else                                                                             \
    {                                                                                \
        if (dTraceSqrt > 1.0)                                                        \
            dTraceSqrt = 1.0;                                                        \
        dScalar = sqrt(1.0 - dTraceSqrt * dTraceSqrt) / dNorm;                       \
        pDstQuatData[0] = dScalar * dKx;                                             \
        pDstQuatData[1] = dScalar * dKy;                                             \
        pDstQuatData[2] = dScalar * dKz;                                             \
        pDstQuatData[3] = dTraceSqrt;                                                \
    }
#endif

void Algorithm::homo2Quat(const double* pSrcHomoData, double* pDstQuatData)
{
    HOMO_2_QUAT(pSrcHomoData, pDstQuatData) ;
}

void Algorithm::homo2SevenData(const double* pSrcHomoData, float* pDstSevenData)
{
    pDstSevenData[0] = pSrcHomoData[3] ;
    pDstSevenData[1] = pSrcHomoData[7] ;
    pDstSevenData[2] = pSrcHomoData[11] ;
    HOMO_2_QUAT(pSrcHomoData, (pDstSevenData + 3)) ;
}

void Algorithm::homo2SevenData(const double* pSrcHomoData, double* pDstSevenData)
{
    pDstSevenData[0] = pSrcHomoData[3] ;
    pDstSevenData[1] = pSrcHomoData[7] ;
    pDstSevenData[2] = pSrcHomoData[11] ;
    HOMO_2_QUAT(pSrcHomoData, (pDstSevenData + 3)) ;
}

#ifndef  SEVEN_DATA_2_HOMO
#define  SEVEN_DATA_2_HOMO(pSrcSevenData, pDstHomoData)                                                                                         \
    double     dQ1Sq = pSrcSevenData[3] * pSrcSevenData[3];                                                                                     \
    double     dQ2Sq = pSrcSevenData[4] * pSrcSevenData[4];                                                                                     \
    double     dQ3Sq = pSrcSevenData[5] * pSrcSevenData[5];                                                                                     \
                                                                                                                                                \
    double     dQ0Q1 = pSrcSevenData[6] * pSrcSevenData[3];                                                                                     \
    double     dQ1Q2 = pSrcSevenData[3] * pSrcSevenData[4];                                                                                     \
    double     dQ2Q3 = pSrcSevenData[4] * pSrcSevenData[5];                                                                                     \
    double     dQ0Q2 = pSrcSevenData[6] * pSrcSevenData[4];                                                                                     \
    double     dQ0Q3 = pSrcSevenData[6] * pSrcSevenData[5];                                                                                     \
    double     dQ1Q3 = pSrcSevenData[3] * pSrcSevenData[5];                                                                                     \
                                                                                                                                                \
    pDstHomoData[0] = 1 - 2 * (dQ2Sq + dQ3Sq);                                                                                                  \
    pDstHomoData[1] = 2 * (dQ1Q2 - dQ0Q3);                                                                                                      \
    pDstHomoData[2] = 2 * (dQ1Q3 + dQ0Q2);                                                                                                      \
    pDstHomoData[3] = pSrcSevenData[0];                                                                                                         \
    pDstHomoData[4] = 2 * (dQ1Q2 + dQ0Q3);                                                                                                      \
    pDstHomoData[5] = 1 - 2 * (dQ1Sq + dQ3Sq);                                                                                                  \
    pDstHomoData[6] = 2 * (dQ2Q3 - dQ0Q1);                                                                                                      \
    pDstHomoData[7] = pSrcSevenData[1];                                                                                                         \
    pDstHomoData[8] = 2 * (dQ1Q3 - dQ0Q2);                                                                                                      \
    pDstHomoData[9] = 2 * (dQ2Q3 + dQ0Q1);                                                                                                      \
    pDstHomoData[10] = 1 - 2 * (dQ1Sq + dQ2Sq);                                                                                                 \
    pDstHomoData[11] = pSrcSevenData[2];                                                                                                        \
    pDstHomoData[12] = 0.0;                                                                                                                     \
    pDstHomoData[13] = 0.0;                                                                                                                     \
    pDstHomoData[14] = 0.0;                                                                                                                     \
    pDstHomoData[15] = 1.0;                                                                                                                     \
                                                                                                                                                \
    double     dProdV1V0 = pDstHomoData[0] * pDstHomoData[1] + pDstHomoData[4] * pDstHomoData[5] + pDstHomoData[8] * pDstHomoData[9];           \
    double     dProdV2V0 = pDstHomoData[0] * pDstHomoData[2] + pDstHomoData[4] * pDstHomoData[6] + pDstHomoData[8] * pDstHomoData[10];          \
    double     dLenV0Sqr = pDstHomoData[0] * pDstHomoData[0] + pDstHomoData[4] * pDstHomoData[4] + pDstHomoData[8] * pDstHomoData[8];           \
    double     dSlope0 = dProdV1V0 / dLenV0Sqr;                                                                                                 \
    double     dSlope1 = dProdV2V0 / dLenV0Sqr;                                                                                                 \
                                                                                                                                                \
    pDstHomoData[1] = pDstHomoData[1] - dSlope0 * pDstHomoData[0] ;                                                                             \
    pDstHomoData[5] = pDstHomoData[5] - dSlope0 * pDstHomoData[4] ;                                                                             \
    pDstHomoData[9] = pDstHomoData[9] - dSlope0 * pDstHomoData[8] ;                                                                             \
                                                                                                                                                \
    double     dProdV1V2 = pDstHomoData[2] * pDstHomoData[1] + pDstHomoData[6] * pDstHomoData[5] + pDstHomoData[10] * pDstHomoData[9] ;         \
    double     dLenV1Sqr = pDstHomoData[1] * pDstHomoData[1] + pDstHomoData[5] * pDstHomoData[5] + pDstHomoData[9] * pDstHomoData[9] ;          \
    double     dSlope2   = dProdV1V2 / dLenV1Sqr ;                                                                                              \
                                                                                                                                                \
    pDstHomoData[2] = pDstHomoData[2] - dSlope1 * pDstHomoData[0] - dSlope2 * pDstHomoData[1] ;                                                 \
    pDstHomoData[6] = pDstHomoData[6] - dSlope1 * pDstHomoData[4] - dSlope2 * pDstHomoData[5] ;                                                 \
    pDstHomoData[10] = pDstHomoData[10] - dSlope1 * pDstHomoData[8] - dSlope2 * pDstHomoData[9] ;                                               \
                                                                                                                                                \
                                                                                                                                                \
    double     dLenV2Sqr = pDstHomoData[2] * pDstHomoData[2] + pDstHomoData[6] * pDstHomoData[6] + pDstHomoData[10] * pDstHomoData[10] ;        \
                                                                                                                                                \
    dLenV0Sqr = sqrt(dLenV0Sqr) ;                                                                                                               \
    dLenV1Sqr = sqrt(dLenV1Sqr) ;                                                                                                               \
    dLenV2Sqr = sqrt(dLenV2Sqr) ;                                                                                                               \
                                                                                                                                                \
    pDstHomoData[0]  = pDstHomoData[0] / dLenV0Sqr ;                                                                                            \
    pDstHomoData[1]  = pDstHomoData[1] / dLenV1Sqr ;                                                                                            \
    pDstHomoData[2]  = pDstHomoData[2] / dLenV2Sqr ;                                                                                            \
    pDstHomoData[4]  = pDstHomoData[4] / dLenV0Sqr ;                                                                                            \
    pDstHomoData[5]  = pDstHomoData[5] / dLenV1Sqr ;                                                                                            \
    pDstHomoData[6]  = pDstHomoData[6] / dLenV2Sqr ;                                                                                            \
    pDstHomoData[8]  = pDstHomoData[8] / dLenV0Sqr ;                                                                                            \
    pDstHomoData[9]  = pDstHomoData[9] / dLenV1Sqr ;                                                                                            \
    pDstHomoData[10] = pDstHomoData[10] / dLenV2Sqr;
#endif

void Algorithm::sevenData2Homo(const float* pSrcSevenData, double* pDstHomoData)
{
    SEVEN_DATA_2_HOMO(pSrcSevenData, pDstHomoData) ;
}

void Algorithm::sevenData2Homo(const double* pSrcSevenData, double* pDstHomoData)
{
    SEVEN_DATA_2_HOMO(pSrcSevenData, pDstHomoData) ;
}
#ifndef QUATXYZW_MATRIX_SUM
#define QUATXYZW_MATRIX_SUM(dMatrixSum, pWeight, dQuat) \
        dMatrixSum[0]  += pWeight[i] * dQuat[3] * dQuat[3] ;  \
        dMatrixSum[1]  += pWeight[i] * dQuat[3] * dQuat[0];   \
        dMatrixSum[2]  += pWeight[i] * dQuat[3] * dQuat[1];   \
        dMatrixSum[3]  += pWeight[i] * dQuat[3] * dQuat[2];   \
        dMatrixSum[4]  += pWeight[i] * dQuat[0] * dQuat[3];   \
        dMatrixSum[5]  += pWeight[i] * dQuat[0] * dQuat[0];   \
        dMatrixSum[6]  += pWeight[i] * dQuat[0] * dQuat[1];   \
        dMatrixSum[7]  += pWeight[i] * dQuat[0] * dQuat[2];   \
        dMatrixSum[8]  += pWeight[i] * dQuat[1] * dQuat[3];   \
        dMatrixSum[9]  += pWeight[i] * dQuat[1] * dQuat[0];   \
        dMatrixSum[10] += pWeight[i] * dQuat[1] * dQuat[1];   \
        dMatrixSum[11] += pWeight[i] * dQuat[1] * dQuat[2];   \
        dMatrixSum[12] += pWeight[i] * dQuat[2] * dQuat[3];   \
        dMatrixSum[13] += pWeight[i] * dQuat[2] * dQuat[0];   \
        dMatrixSum[14] += pWeight[i] * dQuat[2] * dQuat[1];   \
        dMatrixSum[15] += pWeight[i] * dQuat[2] * dQuat[2];
#endif

#ifndef  NORMLIZE_QUATERNION_DOUBLE
#define  NORMLIZE_QUATERNION_DOUBLE(pQuat)                                                                \
    dQuatNorm = pQuat[0] * pQuat[0] + pQuat[1] * pQuat[1] + pQuat[2] * pQuat[2] + pQuat[3] * pQuat[3] ;   \
    dQuatNorm = sqrt(dQuatNorm) ;                                                                         \
    pQuat[0]  = pQuat[0] / dQuatNorm ;                                                                    \
    pQuat[1]  = pQuat[1] / dQuatNorm ;                                                                    \
    pQuat[2]  = pQuat[2] / dQuatNorm ;                                                                    \
    pQuat[3]  = pQuat[3] / dQuatNorm ;
#endif

int Algorithm::avgHomo(const double* pHomoSrc, const double* pWeight, int iHomoCnt, double* pHomoDst)
{
    int     i                = 0 ;
    double  dQuat[4]         = { 0.0, 0.0, 0.0, 0.0 } ;
    double  dMatrixSum[16]   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
    double  dOutVec[16]      = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double  dWeightSum       = 0.0 ;
    double  dVal             = 0.0 ;
    double  dDstSevenData[7] = { 0.0 } ;
    double  dQuatQ0Q3[4]     = { 0.0 } ;
    double  dQuatNorm        = 0.0 ;
    int     iRes             = 0 ;
    dDstSevenData[0]         = 0.0 ;
    dDstSevenData[1]         = 0.0 ;
    dDstSevenData[2]         = 0.0 ;
    for (i = 0; i < iHomoCnt; i++)
    {
        dDstSevenData[0] += pWeight[i] * pHomoSrc[i * HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT + 3] ;
        dDstSevenData[1] += pWeight[i] * pHomoSrc[i * HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT + 7] ;
        dDstSevenData[2] += pWeight[i] * pHomoSrc[i * HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT + 11] ;
        dWeightSum       += pWeight[i] ;
    }
    dDstSevenData[0] = dDstSevenData[0] / dWeightSum ;
    dDstSevenData[1] = dDstSevenData[1] / dWeightSum ;
    dDstSevenData[2] = dDstSevenData[2] / dWeightSum ;

    for (i = 0; i < iHomoCnt; i++)
    {
        homo2Quat(pHomoSrc + i * HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT, dQuat) ;
        QUATXYZW_MATRIX_SUM(dMatrixSum, pWeight, dQuat)
    }

    for (i = 0; i < 16; i++)
        dMatrixSum[i] = dMatrixSum[i] / dWeightSum ;

    iRes = eign4x4MaxVec(dMatrixSum, dOutVec, dQuatQ0Q3) ;
    if (0 == iRes)
    {
        NORMLIZE_QUATERNION_DOUBLE(dQuatQ0Q3) ;
        dDstSevenData[3] = dQuatQ0Q3[1] ;
        dDstSevenData[4] = dQuatQ0Q3[2] ;
        dDstSevenData[5] = dQuatQ0Q3[3] ;
        dDstSevenData[6] = dQuatQ0Q3[0] ;
    }
    else
    {
        dDstSevenData[3] = dQuat[0] ;
        dDstSevenData[4] = dQuat[1] ;
        dDstSevenData[5] = dQuat[2] ;
        dDstSevenData[6] = dQuat[3] ;
    }
    sevenData2Homo(dDstSevenData, pHomoDst) ;

    return 0 ;
}

int Algorithm::avgFrame(const Frame* pFrameArray, const double* pWeight, int iFrameCnt, Frame& frmDst)
{
    int     i                                            = 0 ;
    double  dQuat[4]                                     = { 0.0, 0.0, 0.0, 0.0 } ;
    double  dHomoCur[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT] = { 0.0 } ;
    double  dMatrixSum[16]                               = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double  dWeightSum                                   = 0.0 ;
    double  dVal                                         = 0.0 ;
    double  dDstSevenData[7]                             = { 0.0 } ;
    double  dQuatQ0Q3[4]                                 = { 0.0 } ;
    double  dQuatNorm                                    = 0.0 ;
    int     iRes                                         = 0 ;
    dDstSevenData[0] = 0.0 ;
    dDstSevenData[1] = 0.0 ;
    dDstSevenData[2] = 0.0 ;
    for (i = 0; i < iFrameCnt; i++)
    {
        dDstSevenData[0] += pWeight[i] * pFrameArray[i].mOrigine.mData[0] ;
        dDstSevenData[1] += pWeight[i] * pFrameArray[i].mOrigine.mData[1] ;
        dDstSevenData[2] += pWeight[i] * pFrameArray[i].mOrigine.mData[2] ;
        dWeightSum       += pWeight[i] ;
    }

    dDstSevenData[0] = dDstSevenData[0] / dWeightSum ;
    dDstSevenData[1] = dDstSevenData[1] / dWeightSum ;
    dDstSevenData[2] = dDstSevenData[2] / dWeightSum ;

    for (i = 0; i < iFrameCnt ; i++)
    {
        pFrameArray[i].toHomo(dHomoCur) ;
        homo2Quat(dHomoCur, dQuat) ;
        QUATXYZW_MATRIX_SUM(dMatrixSum, pWeight, dQuat) ;
    }

    for (i = 0; i < 16; i++)
        dMatrixSum[i] = dMatrixSum[i] / dWeightSum;

    iRes = eign4x4MaxVec(dMatrixSum, dHomoCur, dQuatQ0Q3) ;
    if (0 == iRes)
    {
        NORMLIZE_QUATERNION_DOUBLE(dQuatQ0Q3) ;
        dDstSevenData[3] = dQuatQ0Q3[1];
        dDstSevenData[4] = dQuatQ0Q3[2];
        dDstSevenData[5] = dQuatQ0Q3[3];
        dDstSevenData[6] = dQuatQ0Q3[0];
    }
    else
    {
        dDstSevenData[3] = dQuat[0] ;
        dDstSevenData[4] = dQuat[1] ;
        dDstSevenData[5] = dQuat[2] ;
        dDstSevenData[6] = dQuat[3] ;
    }

    sevenData2Homo(dDstSevenData, dHomoCur) ;
    frmDst.fromHomo(dHomoCur) ;

    return 0 ;
}

int Algorithm::avgQuaternion(const double* pQuaternionSrc, const double* pWeight, int iQuatCnt, double* pQuaternionDst)
{
    int     i               = 0 ;
    double  dMatrixSum[16]  = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double  dWeightSum      = 0.0 ;
    double  dVal            = 0.0 ;
    double  dQuatNorm       = 0.0 ;
    int     iRes            = 0 ;
    double  dOutVec[16]     = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
    for (i = 0; i < iQuatCnt; i++)
    {
        dMatrixSum[0]  += pWeight[i] * pQuaternionSrc[i * 4 + 0] * pQuaternionSrc[i * 4 + 0] ;
        dMatrixSum[1]  += pWeight[i] * pQuaternionSrc[i * 4 + 0] * pQuaternionSrc[i * 4 + 1] ;
        dMatrixSum[2]  += pWeight[i] * pQuaternionSrc[i * 4 + 0] * pQuaternionSrc[i * 4 + 2] ;
        dMatrixSum[3]  += pWeight[i] * pQuaternionSrc[i * 4 + 0] * pQuaternionSrc[i * 4 + 3] ;
        dMatrixSum[4]  += pWeight[i] * pQuaternionSrc[i * 4 + 1] * pQuaternionSrc[i * 4 + 0] ;
        dMatrixSum[5]  += pWeight[i] * pQuaternionSrc[i * 4 + 1] * pQuaternionSrc[i * 4 + 1] ;
        dMatrixSum[6]  += pWeight[i] * pQuaternionSrc[i * 4 + 1] * pQuaternionSrc[i * 4 + 2] ;
        dMatrixSum[7]  += pWeight[i] * pQuaternionSrc[i * 4 + 1] * pQuaternionSrc[i * 4 + 3] ;
        dMatrixSum[8]  += pWeight[i] * pQuaternionSrc[i * 4 + 2] * pQuaternionSrc[i * 4 + 0] ;
        dMatrixSum[9]  += pWeight[i] * pQuaternionSrc[i * 4 + 2] * pQuaternionSrc[i * 4 + 1] ;
        dMatrixSum[10] += pWeight[i] * pQuaternionSrc[i * 4 + 2] * pQuaternionSrc[i * 4 + 2] ;
        dMatrixSum[11] += pWeight[i] * pQuaternionSrc[i * 4 + 2] * pQuaternionSrc[i * 4 + 3] ;
        dMatrixSum[12] += pWeight[i] * pQuaternionSrc[i * 4 + 3] * pQuaternionSrc[i * 4 + 0] ;
        dMatrixSum[13] += pWeight[i] * pQuaternionSrc[i * 4 + 3] * pQuaternionSrc[i * 4 + 1] ;
        dMatrixSum[14] += pWeight[i] * pQuaternionSrc[i * 4 + 3] * pQuaternionSrc[i * 4 + 2] ;
        dMatrixSum[15] += pWeight[i] * pQuaternionSrc[i * 4 + 3] * pQuaternionSrc[i * 4 + 3] ;
        dWeightSum     += pWeight[i] ;
    }

    for (i = 0; i < 16; i++)
        dMatrixSum[i] = dMatrixSum[i] / dWeightSum ;

    iRes = eign4x4MaxVec(dMatrixSum, dOutVec, pQuaternionDst) ;
    if (0 == iRes)
    {
        NORMLIZE_QUATERNION_DOUBLE(pQuaternionDst) ;
    }
    else
    {
        pQuaternionDst[0] = pQuaternionSrc[0] ;
        pQuaternionDst[1] = pQuaternionSrc[1] ;
        pQuaternionDst[2] = pQuaternionSrc[2] ;
        pQuaternionDst[3] = pQuaternionSrc[3] ;
    }
    return 0;
}

int Algorithm::avgQuaternionXYZW(const double* pQuaternionSrc, const double* pWeight, int iQuatCnt, double* pQuaternionDst)
{
    int     i                   = 0 ;
    double  dMatrixSum[16]      = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
    double  dEignOut[16]        = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
    double  dQuatQ0Q3[4]        = { 0.0 } ;
    double  dWeightSum          = 0.0 ;
    double  dVal                = 0.0 ;
    double  dQuatNorm           = 0.0 ;
    int     iRes                = 0 ;
    for (i = 0; i < iQuatCnt ; i++)
    {
        dMatrixSum[0]  += pWeight[i] * pQuaternionSrc[i * 4 + 3] * pQuaternionSrc[i * 4 + 3] ;
        dMatrixSum[1]  += pWeight[i] * pQuaternionSrc[i * 4 + 3] * pQuaternionSrc[i * 4 + 0] ;
        dMatrixSum[2]  += pWeight[i] * pQuaternionSrc[i * 4 + 3] * pQuaternionSrc[i * 4 + 1] ;
        dMatrixSum[3]  += pWeight[i] * pQuaternionSrc[i * 4 + 3] * pQuaternionSrc[i * 4 + 2] ;
        dMatrixSum[4]  += pWeight[i] * pQuaternionSrc[i * 4 + 0] * pQuaternionSrc[i * 4 + 3] ;
        dMatrixSum[5]  += pWeight[i] * pQuaternionSrc[i * 4 + 0] * pQuaternionSrc[i * 4 + 0] ;
        dMatrixSum[6]  += pWeight[i] * pQuaternionSrc[i * 4 + 0] * pQuaternionSrc[i * 4 + 1] ;
        dMatrixSum[7]  += pWeight[i] * pQuaternionSrc[i * 4 + 0] * pQuaternionSrc[i * 4 + 2] ;
        dMatrixSum[8]  += pWeight[i] * pQuaternionSrc[i * 4 + 1] * pQuaternionSrc[i * 4 + 3] ;
        dMatrixSum[9]  += pWeight[i] * pQuaternionSrc[i * 4 + 1] * pQuaternionSrc[i * 4 + 0] ;
        dMatrixSum[10] += pWeight[i] * pQuaternionSrc[i * 4 + 1] * pQuaternionSrc[i * 4 + 1] ;
        dMatrixSum[11] += pWeight[i] * pQuaternionSrc[i * 4 + 1] * pQuaternionSrc[i * 4 + 2] ;
        dMatrixSum[12] += pWeight[i] * pQuaternionSrc[i * 4 + 2] * pQuaternionSrc[i * 4 + 3] ;
        dMatrixSum[13] += pWeight[i] * pQuaternionSrc[i * 4 + 2] * pQuaternionSrc[i * 4 + 0] ;
        dMatrixSum[14] += pWeight[i] * pQuaternionSrc[i * 4 + 2] * pQuaternionSrc[i * 4 + 1] ;
        dMatrixSum[15] += pWeight[i] * pQuaternionSrc[i * 4 + 2] * pQuaternionSrc[i * 4 + 2] ;
        dWeightSum     += pWeight[i] ;
    }
    for (i = 0; i < 16; i++)
        dMatrixSum[i] = dMatrixSum[i] / dWeightSum;

    iRes = eign4x4MaxVec(dMatrixSum, dEignOut, dQuatQ0Q3) ;
    if (0 == iRes)
    {
        pQuaternionDst[0] = dQuatQ0Q3[1] ;
        pQuaternionDst[1] = dQuatQ0Q3[2] ;
        pQuaternionDst[2] = dQuatQ0Q3[3] ;
        pQuaternionDst[3] = dQuatQ0Q3[0] ;
        NORMLIZE_QUATERNION_DOUBLE(pQuaternionDst) ;
    }
    else
    {
        pQuaternionDst[0] = pQuaternionSrc[0] ;
        pQuaternionDst[1] = pQuaternionSrc[1] ;
        pQuaternionDst[2] = pQuaternionSrc[2] ;
        pQuaternionDst[3] = pQuaternionSrc[3] ;
    }

    return 0 ;
}

#define MAX_J4   (110 * PI / 180)
#define MIN_J4   (-110 * PI / 180)
#define MAX_J6   (110 * PI / 180)
#define MIN_J6   (-110 * PI / 180)
bool Algorithm::checkJointLimitSuccess(const Joint &joint)
{
    double j1 = joint.get(0);
    double j2 = joint.get(1);
    double j3 = joint.get(2);
    double j4 = joint.get(3);
    double j5 = joint.get(4);
    double j6 = joint.get(5);
    double j7 = joint.get(6);
    if (j6 > MAX_J6 || j6 < MIN_J6) {
        return false;
    }
    return true;
}

// the functions used in UnitTest
// this function is used to measure the runtime of the tested funtion.
int time_substract(struct timeval *result, struct timeval *begin, struct timeval *end)
{
    if(begin->tv_sec > end->tv_sec)
    {
        return -1;
    }
    if((begin->tv_sec == end->tv_sec) && (begin->tv_usec > end->tv_usec))
    {
        return -2;
    }
    result->tv_sec  = (end->tv_sec  - begin->tv_sec);
    result->tv_usec = (end->tv_usec - begin->tv_usec);
    if(result->tv_usec < 0)
    {
        result->tv_sec--;
        result->tv_usec += 1000000;
    }
    return 0;
}

#ifndef   QUAT_MULTIPLY
#define   QUAT_MULTIPLY(pQuatFirst, pQuatSecond, pQuatResult)                                                                                                            \
    pQuatResult[3] = (pQuatFirst[3] * pQuatSecond[3]) - (pQuatFirst[0] * pQuatSecond[0]) - (pQuatFirst[1] * pQuatSecond[1]) - (pQuatFirst[2] * pQuatSecond[2]) ;         \
    pQuatResult[0] = (pQuatFirst[3] * pQuatSecond[0]) + (pQuatFirst[0] * pQuatSecond[3]) + (pQuatFirst[1] * pQuatSecond[2]) - (pQuatFirst[2] * pQuatSecond[1]) ;         \
    pQuatResult[1] = (pQuatFirst[3] * pQuatSecond[1]) + (pQuatFirst[1] * pQuatSecond[3]) + (pQuatFirst[2] * pQuatSecond[0]) - (pQuatFirst[0] * pQuatSecond[2]) ;         \
    pQuatResult[2] = (pQuatFirst[3] * pQuatSecond[2]) + (pQuatFirst[2] * pQuatSecond[3]) + (pQuatFirst[0] * pQuatSecond[1]) - (pQuatFirst[1] * pQuatSecond[0]) ;
#endif

void Algorithm::QuatMultiply(const float* pQuatFirst, const float* pQuatSecond, float* pQuatResult)
{
    QUAT_MULTIPLY(pQuatFirst, pQuatSecond, pQuatResult) ;
}

void Algorithm::QuatMultiply(const double* pQuatFirst, const double* pQuatSecond, double* pQuatResult)
{
    QUAT_MULTIPLY(pQuatFirst, pQuatSecond, pQuatResult) ;
}

#ifndef   QUAT_CONJ
#define   QUAT_CONJ(pQuatSrc, pQuatConj)              \
    pQuatConj[0]  = -pQuatSrc[0] ;                    \
    pQuatConj[1]  = -pQuatSrc[1] ;                    \
    pQuatConj[2]  = -pQuatSrc[2] ;                    \
    pQuatConj[3]  =  pQuatSrc[3] ;
#endif

void Algorithm::QuatConj(const float* pQuatSrc, float* pQuatConj)
{
    QUAT_CONJ(pQuatSrc, pQuatConj) ;
}

void Algorithm::QuatConj(const double* pQuatSrc, double* pQuatConj)
{
    QUAT_CONJ(pQuatSrc, pQuatConj) ;
}

#ifndef   QUAT_DIFF
#define   QUAT_DIFF(pQuatFirst, pQuatFirstConj, pQuatSecond, pQuatResult)                        \
    QUAT_CONJ(pQuatFirst, pQuatFirstConj)                                                        \
    QUAT_MULTIPLY(pQuatFirstConj, pQuatSecond, pQuatResult)
#endif

void Algorithm::QuatDiff(const float* pQuatFirst, const float* pQuatSecond, float* pQuatDiff)
{
    float    fFirstConj[4] ;
    QUAT_DIFF(pQuatFirst, fFirstConj, pQuatSecond, pQuatDiff) ;
}

void Algorithm::QuatDiff(const double* pQuatFirst, const double* pQuatSecond, double* pQuatDiff)
{
    double   dFirstConj[4] ;
    QUAT_DIFF(pQuatFirst, dFirstConj, pQuatSecond, pQuatDiff) ;
}

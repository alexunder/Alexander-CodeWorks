#include "Rotation.h"
#include "GeometryUtils.h"
/*****************************************************************************
 Prototype    : Rotation.fromRPY
 Description  : get rotation matrix from roll, pitch and yaw
 Input        : double roll
                double pitch
                double yaw
 Output       : None
 Return Value : Rotation
 Calls        :
 Called By    :

  History        :
  1.Date         :
    Author       :
    Modification : Created function
note:
     1, roll :rotate around x axis;
        pitch:rotate around y axis;
        yaw  :rotate around z axis;
     2, see 1, Euler Angle Formulas, 2.6 Factor as RzRyRx
            2, Computing Euler angles from a rotation matrix, Gregory G. Slabaugh
*****************************************************************************/
Rotation Rotation::fromRPY(double roll,double pitch,double yaw)
{
    double ca1,cb1,cc1,sa1,sb1,sc1;
    ca1 = cos(yaw); sa1 = sin(yaw);                         //cz, sz
    cb1 = cos(pitch);sb1 = sin(pitch);                      //cy, sy
    cc1 = cos(roll);sc1 = sin(roll);                        //cx, sx
    return Rotation(ca1*cb1, ca1*sb1*sc1 - sa1*cc1, ca1*sb1*cc1 + sa1*sc1,
                    sa1*cb1, sa1*sb1*sc1 + ca1*cc1, sa1*sb1*cc1 - ca1*sc1,
                    -sb1,    cb1*sc1,               cb1*cc1);
}

// Gives back a vector in RPY coordinates
/*****************************************************************************
 Prototype    : Rotation.getRPY
 Description  : get frame from Euler angles (roll, pitch, yaw)
 Input        : Rotation::mData

 Output :       roll  : roll angle
                pitch : pitch angle
                yaw   : yaw angle
 Return Value : void
 Calls        :
 Called By    :
  History        :
  1.Date         :
    Author       :
    Modification : Created function
note:
     1, roll :rotate around x axis;
        pitch:rotate around y axis;
        yaw  :rotate around z axis;
     2, see 1, Euler Angle Formulas, 2.6 Factor as RzRyRx
            2, Computing Euler angles from a rotation matrix, Gregory G. Slabaugh
*****************************************************************************/
void Rotation::getRPY(double& roll,double& pitch,double& yaw) const
{
    double epsilon=1E-12;
    pitch = atan2(-mData[6], sqrt( sqr(mData[0]) +sqr(mData[3]) )  );
    if ( fabs(pitch) > (PI/2.0-epsilon) ) {
        yaw = atan2(-mData[1], mData[4]);
        roll  = 0.0 ;
    } else {
        roll  = atan2(mData[7], mData[8]);
        yaw   = atan2(mData[3], mData[0]);
    }
}

Rotation operator *(const Rotation& lhs,const Rotation& rhs)
{
    return Rotation(
                    lhs.mData[0]*rhs.mData[0]+lhs.mData[1]*rhs.mData[3]+lhs.mData[2]*rhs.mData[6],
                    lhs.mData[0]*rhs.mData[1]+lhs.mData[1]*rhs.mData[4]+lhs.mData[2]*rhs.mData[7],
                    lhs.mData[0]*rhs.mData[2]+lhs.mData[1]*rhs.mData[5]+lhs.mData[2]*rhs.mData[8],
                    lhs.mData[3]*rhs.mData[0]+lhs.mData[4]*rhs.mData[3]+lhs.mData[5]*rhs.mData[6],
                    lhs.mData[3]*rhs.mData[1]+lhs.mData[4]*rhs.mData[4]+lhs.mData[5]*rhs.mData[7],
                    lhs.mData[3]*rhs.mData[2]+lhs.mData[4]*rhs.mData[5]+lhs.mData[5]*rhs.mData[8],
                    lhs.mData[6]*rhs.mData[0]+lhs.mData[7]*rhs.mData[3]+lhs.mData[8]*rhs.mData[6],
                    lhs.mData[6]*rhs.mData[1]+lhs.mData[7]*rhs.mData[4]+lhs.mData[8]*rhs.mData[7],
                    lhs.mData[6]*rhs.mData[2]+lhs.mData[7]*rhs.mData[5]+lhs.mData[8]*rhs.mData[8]
                );
}


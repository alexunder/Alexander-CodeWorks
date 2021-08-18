#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include <string.h>

#include "Vector.h"
#include "Frame.h"
#include "Joint.h"
#include "Algorithm/Algorithm.h"

/*********************
*
* create by deming.qiao at 2017/01/25
*
* enum :Message
* used to parase protocol or use data to generate protocol
*                   Protocol from arm
* ------------------------------------------------
* |              messageId(4 bytes)               |
* ________________________________________________
* |           dataCount(unit by byte)(4 bytes)    |
* ________________________________________________
* |                      *                       |
* |                      *                       |
* |              data(dataCount*8 bits)          |
* |                      *                       |
* |                      *                       |
* ________________________________________________
*
***********************/


enum class MessageId : uint32_t {
    END_FRI_SESSION = 0x1,    // controller -> kuka
    TROCAR_POSITION = 0x2,    // kuka -> controller
    FLANGE_FRAME = 0x3,       // kuka -> controller
    JOINT_POS = 0x4,          // kuka -> controller
    INIT_FINISHED = 0x5,      // controller -> kuka
    ADJUST_BEGIN = 0x6,       // controller -> kuka
    ADJUST_END = 0x7,         // controller -> kuka
    HOLD_BEGIN = 0x8,         // controller -> kuka
    HOLD_END = 0x9,           // controller -> kuka
    FRI_CONNECTED = 0xa,      // kuka -> controller
    //both used message
    ERROR_STATE = 1,//kuka error and controller error
    //kuka used message
    BUILD_WORLD_COOR = 100,//touch three points to build the world cooridate
    FIND_TROCAR_POSITION,//find trocar position 
    MOVE_AND_RCM,//move to rcm position
    ATTACH_INSTRUMENTS,//attache instruments on kuka
    MOVE_TO_BELLY,//move the instrument near belly
    READY,//finish all preparing and ready to surgery
    MOVE_ARRAY_FROM_BELLY, //move away from belly after surgerying
    DETACH_INSTRUMENTS,//detach instruments
    INIT_POSITION,//move to init position
    //controller used message
    RE_INIT = 200,//if all the operation has finished or an error occured
    NEXT,//to do next 
    REDO // do current operation again
};

class MessageParser {
public:
    static inline Vector vectorFromMsg(void *data)
    {
        double x, y, z;
        memcpy(&x, &static_cast<char *>(data)[0], 8);
        memcpy(&y, &static_cast<char *>(data)[8], 8);
        memcpy(&z, &static_cast<char *>(data)[16], 8);
        return Vector(ntohd(x), ntohd(y), ntohd(z));
    }
    static inline Frame frameFromMsg(void *data)
    {
        double x, y, z;
        double a, b, c;
        double r00, r01, r02;
        double r10, r11, r12;
        double r20, r21, r22;
        memcpy(&x, &static_cast<char *>(data)[0], 8);
        memcpy(&y, &static_cast<char *>(data)[8], 8);
        memcpy(&z, &static_cast<char *>(data)[16], 8);

        memcpy(&a, &static_cast<char *>(data)[24], 8);
        memcpy(&b, &static_cast<char *>(data)[32], 8);
        memcpy(&c, &static_cast<char *>(data)[40], 8);

        memcpy(&r00, &static_cast<char *>(data)[48], 8);
        memcpy(&r01, &static_cast<char *>(data)[56], 8);
        memcpy(&r02, &static_cast<char *>(data)[64], 8);

        memcpy(&r10, &static_cast<char *>(data)[72], 8);
        memcpy(&r11, &static_cast<char *>(data)[80], 8);
        memcpy(&r12, &static_cast<char *>(data)[88], 8);

        memcpy(&r20, &static_cast<char *>(data)[96], 8);
        memcpy(&r21, &static_cast<char *>(data)[104], 8);
        memcpy(&r22, &static_cast<char *>(data)[112], 8);

        Rotation r = Rotation(
                       ntohd(r00), ntohd(r01), ntohd(r02),
                       ntohd(r10), ntohd(r11), ntohd(r12),
                       ntohd(r20), ntohd(r21), ntohd(r22)
                     );

        double flangeTrans[3][4];
        Algorithm::kukaTransFromEuler(ntohd(x), ntohd(y), ntohd(z), ntohd(a) * 180 / PI, ntohd(b) * 180 / PI, ntohd(c) * 180 / PI, flangeTrans);
        return Frame(flangeTrans);

/*
        return Frame(Rotation(
                       ntohd(r00), ntohd(r01), ntohd(r02),
                       ntohd(r10), ntohd(r11), ntohd(r12),
                       ntohd(r20), ntohd(r21), ntohd(r22)
                     ), 
                     Vector(ntohd(x), ntohd(y), ntohd(z))
                    );
*/
    }

    static inline Joint jointFromMsg(void *data)
    {
        double j1, j2, j3, j4, j5, j6, j7;
        memcpy(&j1, &static_cast<char *>(data)[0], 8);
        memcpy(&j2, &static_cast<char *>(data)[8], 8);
        memcpy(&j3, &static_cast<char *>(data)[16], 8);
        memcpy(&j4, &static_cast<char *>(data)[24], 8);
        memcpy(&j5, &static_cast<char *>(data)[32], 8);
        memcpy(&j6, &static_cast<char *>(data)[40], 8);
        memcpy(&j7, &static_cast<char *>(data)[48], 8);
        return Joint(ntohd(j1), ntohd(j2), ntohd(j3), ntohd(j4), ntohd(j5), ntohd(j6), ntohd(j7));
    }

    static inline double ntohd(double netdouble)
    {
        double rst;
        char tmp1[8];
        char tmp2[8];
        memcpy(tmp1, &netdouble, 8);
        for (int i = 0; i < 8; i++) {
            tmp2[i] = tmp1[8 - i - 1];
        }
        memcpy(&rst, tmp2, 8);
        return rst;
    }

    static inline double htond(double hostdouble)
    {
        double rst;
        char tmp1[8];
        char tmp2[8];
        memcpy(tmp1, &hostdouble, 8);
        for (int i = 0; i < 8; i++) {
            tmp2[i] = tmp1[8 - i - 1];
        }
        memcpy(&rst, tmp2, 8);
        return rst;
    }
};

#endif

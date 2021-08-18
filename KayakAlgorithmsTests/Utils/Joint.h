#ifndef Joint_h_
#define Joint_h_

class Joint {
public:
    double mData[7];
    inline Joint() {mData[0]=mData[1]=mData[2]=mData[3]=mData[4]=mData[5]=mData[6] = 0.0;}
    inline Joint(const Joint& otherJoint)
    {
      mData[0] = otherJoint.mData[0] ;
      mData[1] = otherJoint.mData[1] ;
      mData[2] = otherJoint.mData[2] ;
      mData[3] = otherJoint.mData[3] ;
      mData[4] = otherJoint.mData[4] ;
      mData[5] = otherJoint.mData[5] ;
      mData[6] = otherJoint.mData[6] ;
    }
    inline Joint(double j1, double j2, double j3, double j4, double j5, double j6, double j7)
    {
        mData[0] = j1;
        mData[1] = j2;
        mData[2] = j3;
        mData[3] = j4;
        mData[4] = j5;
        mData[5] = j6;
        mData[6] = j7;
    }
    inline double get(int idx) const { return mData[idx]; }
    inline void set(int idx, double j) { mData[idx] = j; }
    inline std::string toString() 
    {
        std::stringstream s;
        s<<"|"<<mData[0]<<"|";
        s<<"|"<<mData[1]<<"|";
        s<<"|"<<mData[2]<<"|";
        s<<"|"<<mData[3]<<"|";
        s<<"|"<<mData[4]<<"|";
        s<<"|"<<mData[5]<<"|";
        s<<"|"<<mData[6]<<"|";
        return s.str(); 
    }

    inline Joint& operator=(const Joint& jointOther)
    {
        if (this == &jointOther)
            return *this ;
        
        mData[0] = jointOther.mData[0] ;
        mData[1] = jointOther.mData[1] ;
        mData[2] = jointOther.mData[2] ;
        mData[3] = jointOther.mData[3] ;
        mData[4] = jointOther.mData[4] ;
        mData[5] = jointOther.mData[5] ;
        mData[6] = jointOther.mData[6] ;

        return *this ;
    }
};

#define   JointBlending               Joint

#endif


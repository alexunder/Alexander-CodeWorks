#ifndef    __KY_COM_DEF_H__
#define    __KY_COM_DEF_H__


#define   HAPTION_POS_VAL_CNT                              7
#define   HAPTION_SPEED_VAL_CNT                            6
#define   HAPTION_FORCE_VAL_CNT                            6
#define   HAPTION_ARTICULAR_POS_VAL_CNT                    6
#define   HAPTION_ARTICULAR_POS_PINCH_VAL_CNT              (HAPTION_ARTICULAR_POS_VAL_CNT + 1)
#define   HAPTION_ARTICULAR_SPEED_VAL_CNT                  6
#define   HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT               16
#define   VECTOR_CNT                                       3 
#define   HAPTION_ARTICULAR_PD_PARAM_CNT                   3
#define   CUBIC_TRAJ_PLAN_PARAM_CNT                        4
#define   TOTAL_KUKA_COUNT                                 0 
#define   CLI_TITLE_COUNT                                  4 
#define   CLI_INFO_COUNT                                   5 
#define   MAX_LOGS                                         1024

#define   KAYAK_DEVICE_CNT                                 2

typedef enum tag_ky_err_code {
    KY_OK = 0,
    KY_HAPTION_MOVE_POSITION_ERR,
    KY_HAPTION_MOVE_ARTICULAR_ERR,
    KY_HAPTION_MOVE_COMPENSATE_POS_ERR,
    KY_HAPTION_ROT_ORIENTATION_ERR,
    KY_PROXIMITY_INIT_ERR,
    KY_PEDAL_INIT_ERR,
    KY_EYEPIECE_INIT_ERR,
    KY_HAPTION_INIT_ERR,
    KY_ESD_INIT_ERR,
    KY_MOTOR_INIT_ERR,
    KY_MSG_TIME_OUT,
    KY_MSG_PENDING_CANCEL,
    KY_TEAR_DOWN,
    KY_HAPTION_BACK_MAPPING_ERR,
    KY_PROXIMIT_FAR,
    KY_ERR_SUM
}KY_ERR_CODE ;

enum class CLI_USER_TITLE{NONE = 0,WAIT,PREPARE,SETUP};
enum class CLI_USER_INFO{NONE = 0,WAITINT,BUILD_COORD,TROCAR_POSITION,PREPARE_FINISHED};

#define   KY_ERR_BITS        8

#define  NSEC_PER_SECOND                           1000000000

#ifndef ALIGNED_SECOND
#define  ALIGNED_SECOND(dstTime)                   \
while (dstTime.tv_nsec >= NSEC_PER_SECOND)         \
{                                                  \
    dstTime.tv_nsec -= NSEC_PER_SECOND;            \
    dstTime.tv_sec++;                              \
}
#endif

const double IDENTITY_HOMO[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT] = {
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1
};

const int gNanoSecPerSec = 1000000000 ;

typedef struct tag_device_data {
    unsigned long long        ullTimeStamp ;
    unsigned long long        ullTimeStampEnd ;
    double                    dHomo[HOMOGENEOUS_TRANSFORM_4X4_ELEM_CNT] ;
    double                    dSpeed[6] ;
    double                    dArticularSpeed[6] ;
    double                    dArticularPosition[6] ;
    double                    dClip ;
}DEVICE_DATA, *PDEVICE_DATA;

#define KY_PI                   3.141592653589793238462643383279
#define KY_DEG2RAD_PIDIV180     0.01745329251994329576923690768488

#define MAPPING_TASK_AFFINITY_LEFT                  (1 << 2)
#define MAPPING_TASK_AFFINITY_RIGHT                 (1 << 3)
#define ELMO_ESD_TASK_AFFINITY_LEFT                 (1 << 4)
#define ELMO_ESD_TASK_AFFINITY_RIGHT                (1 << 5)
#define FRI_STEP_TASK_AFFINITY_LEFT                 (1 << 6)
#define FRI_STEP_TASK_AFFINITY_RIGHT                (1 << 7)

typedef int(*LOG_PRINT_CB)(void* ptr_cb_usr, int log_level, const char* str_fmt, ...) ;

#endif

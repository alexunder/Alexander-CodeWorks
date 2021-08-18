#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <pthread.h>
#include <cstring>
#include <cstdlib>
#include "LogInter.h"

class Logger : public LogInter
{
public:
    static Logger* instance()
    {
        return m_pInstance;
    }   
    /*
    add log into queue
    if add success return true
    else return false
    */
    bool log(const LogLevel& level,const char* msg, ...);
    /*
    get logs from start to end
    if end equals -1 it will return all the left log from start 
    return value 
    n count how many logs returned
    */
    bool getLog(LogText* log) ;

private:
    Logger();
    LogText* m_pLogs;
    int m_iStart;
    int m_iEnd;
    static Logger* m_pInstance; 
    pthread_mutex_t         m_pLogMtx ;
};
#endif

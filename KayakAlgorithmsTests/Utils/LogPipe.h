#ifndef _LOG_PIPE_H_
#define _LOG_PIPE_H_

#include "LogInter.h"

#include <pthread.h>
#include <alchemy/pipe.h>

class LogPipe : public LogInter
{
public:
  static LogPipe* instance()
  {
    return m_pInstance;
  }

  // call from xenomai domain
  bool log(const LogLevel& level,const char* msg, ...);

  // call from linux domain
  bool getLog(LogText* logs);

private:
  LogPipe();
  ~LogPipe();
  static LogPipe* m_pInstance; 
   
  RT_PIPE mPipe;
  int mHandle;     // linux domain access handle
};


#endif


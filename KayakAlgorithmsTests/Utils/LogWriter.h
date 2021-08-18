#ifndef _LOG_WRITER_H_
#define _LOG_WRITER_H_

#include "LogInter.h"

class LogWriter
{
public:
  static LogWriter* instance()
  {
    return m_pInstance;
  }

  bool write(const char *filename, const LogText& logText);

  char * getLogPath()
  {
    return m_logdir;
  }
private:
  LogWriter();
  ~LogWriter();
  static LogWriter* m_pInstance; 

  void prepareDir();
  char  m_logdir[256] ;
};



#endif


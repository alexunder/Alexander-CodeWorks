#ifndef _LOGINTER_H_
#define _LOGINTER_H_

#define MAX_LOGS   1024

enum class LogLevel {NONE, DEBUG , INFO, WARN, ERROR, FATAL };

struct LogText
{  
  char m_pStr[1024];
  LogLevel m_eLevel;
  long int m_tTime; 
};

class LogInter
{
public:
  virtual bool log(const LogLevel& level,const char* msg, ...) = 0;
  virtual bool getLog(LogText* logs) = 0;
};

#endif

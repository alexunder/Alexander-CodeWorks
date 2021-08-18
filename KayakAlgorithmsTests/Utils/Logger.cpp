#include "Logger.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "UtilFunc.h"

Logger* Logger::m_pInstance = new Logger();

Logger::Logger()
{
    m_pLogs      = new LogText[MAX_LOGS] ;  
    m_pLogMtx    = PTHREAD_MUTEX_INITIALIZER ;
    m_iStart     = 0 ;
    m_iEnd       = 0 ;
}

bool Logger::log(const LogLevel& level,const char* msg, ...)
{
    pthread_mutex_lock(&m_pLogMtx); 

    char log_buffer[1024];
    va_list ap;
    va_start(ap, msg);
    vsnprintf(log_buffer, 1024 , msg, ap);
    va_end(ap);

    m_iEnd++;
    if (m_iEnd == m_iStart - 1 || m_iStart == 0 && m_iEnd == MAX_LOGS - 1) {
        m_iStart++;
    }
    if (m_iStart == MAX_LOGS) m_iStart = 0;
    if (m_iEnd == MAX_LOGS) {
        m_iEnd = 0;
        m_pLogs[MAX_LOGS - 1].m_eLevel = level;
        strncpy(m_pLogs[MAX_LOGS - 1].m_pStr, log_buffer, 1024);
        time(&m_pLogs[MAX_LOGS - 1].m_tTime);
    } else {
        m_pLogs[m_iEnd - 1].m_eLevel = level;
        strncpy(m_pLogs[m_iEnd - 1].m_pStr, log_buffer, 1024);
        time(&m_pLogs[m_iEnd - 1].m_tTime);   
    }
    pthread_mutex_unlock(&m_pLogMtx); 
    return true;
}

bool Logger::getLog(LogText* log) 
{
  if (m_iStart == m_iEnd) return 0;
  pthread_mutex_lock(&m_pLogMtx);
  if (m_iStart == m_iEnd) {
    pthread_mutex_unlock(&m_pLogMtx); 
    return false;
  }
  if(m_iStart == MAX_LOGS)
    m_iStart = 0;
  strcpy(log->m_pStr, m_pLogs[m_iStart].m_pStr);
  log->m_eLevel = m_pLogs[m_iStart].m_eLevel;
  log->m_tTime = m_pLogs[m_iStart].m_tTime;
  m_iStart++;
  pthread_mutex_unlock(&m_pLogMtx); 

  return true;
} 


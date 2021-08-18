#include "LogPipe.h"
#include "UtilFunc.h"

#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

LogPipe* LogPipe::m_pInstance = new LogPipe();

LogPipe::LogPipe()
{
  int conn = rt_pipe_create(&mPipe, NULL, P_MINOR_AUTO, 0);
  if (conn >= 0) {
    char devname[128];
    sprintf(devname, "/dev/rtp%d", conn);
    mHandle = open(devname, O_RDONLY);
  }
}

LogPipe::~LogPipe()
{
  if (mHandle >= 0) {
    close(mHandle);
  }
  rt_pipe_delete(&mPipe);
}

bool LogPipe::getLog(LogText* log)
{
  ssize_t rst = read(mHandle, log, sizeof(LogText));
  if (rst != sizeof(LogText)) {
    perror("read");
    return false;
  }
  return true;
}

bool LogPipe::log(const LogLevel& level, const char* msg, ...)
{
  char log_buffer[1024];
  va_list ap;
  va_start(ap, msg);
  vsnprintf(log_buffer, 1024 , msg, ap);
  va_end(ap);

  struct LogText logText;
  logText.m_eLevel = level;
  strncpy(logText.m_pStr, log_buffer, 1024);
  time(&logText.m_tTime);
  ssize_t s = rt_pipe_write(&mPipe, &logText, sizeof(logText), P_NORMAL);
  if (s != sizeof(logText)) {
    return false;
  } else {
    return true;
  }
}


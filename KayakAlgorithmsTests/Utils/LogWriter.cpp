#include "LogWriter.h"
#include "UtilFunc.h"

#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include <stdlib.h>


LogWriter* LogWriter::m_pInstance = new LogWriter();

LogWriter::LogWriter()
{
    memset(m_logdir, 0, sizeof(m_logdir));
    prepareDir();
}

LogWriter::~LogWriter()
{
}

bool LogWriter::write(const char* filename, const LogText& logText)
{
  char fullname[256];

  sprintf(fullname, "%s/%s", m_logdir, filename);
  FILE *fp = fopen(fullname, "a+");
  if (fp != NULL) {
    char stmp[128];
    strcpy(stmp, ctime(&logText.m_tTime));
    stmp[strlen(stmp) - 1] = '\0';
    switch(logText.m_eLevel) {
      case LogLevel::NONE:
          fprintf(fp, "NONE: %s %s\n", stmp, logText.m_pStr);
          break;
      case LogLevel::DEBUG:
          fprintf(fp, "DEBUG: %s %s\n", stmp, logText.m_pStr);
          break;
      case LogLevel::INFO:
          fprintf(fp, "INFO: %s %s\n", stmp, logText.m_pStr);
          break;
      case LogLevel::WARN:
          fprintf(fp, "WARN: %s %s\n", stmp, logText.m_pStr);
          break;
      case LogLevel::ERROR:
          fprintf(fp, "ERROR: %s %s\n", stmp, logText.m_pStr);
          break;
      case LogLevel::FATAL:
          fprintf(fp, "FATAL: %s %s\n", stmp, logText.m_pStr);
          break;
      default:
          break;
    }
    fflush(fp);
    fclose(fp);
  }
}

void LogWriter::prepareDir()
{
    // create log dir
    const char * LogPath = "log";
    char szDateTime[64] = { 0 };
    GetCurDateTime(szDateTime);

    sprintf(m_logdir, "%s/%s_log", LogPath, szDateTime);

    struct stat st;
    if (stat(LogPath, &st) == 0) {
        if (st.st_mode & S_IFDIR != 0) {
        }
        else {
            printf("log exist but not a directory !\n");
            exit(-1);
        }
    }
    else {
        if (mkdir(LogPath, 0755) != 0) {
            perror("mkdir");
            exit(-1);
        }
    }

    if (mkdir(m_logdir, 0755) != 0) {
        perror("mkdir");
        exit(-1);
    }
}


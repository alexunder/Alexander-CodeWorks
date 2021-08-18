#include "CLIWindow.h"
#include <cstdio>
#include <cstring>

CLIWindow* CLIWindow::m_pInstance = new CLIWindow();
CLIWindow::CLIWindow():CLIInter()
{
    m_bHasColor = false; 
    m_bWindowOpen = false;
    m_pLogs = new LogText[MAX_LOGS];
    m_bRun = true;
    m_iUserInfoIndex = CLI_USER_INFO::NONE;
    m_iUserTitleIndex = CLI_USER_TITLE::NONE;
    m_pOpenMtx = PTHREAD_MUTEX_INITIALIZER;
    m_pRunMtx = PTHREAD_MUTEX_INITIALIZER;
}
void* CLIWindow::key(void*)
{   
    CLIWindow::getInstance()->run(); 
}
const char* CLIWindow::getUserInput()
{
    if(m_iPoolStart == m_iPoolEnd)
        return nullptr;
    const char* result = m_pUserInputPool[m_iPoolStart];    
    m_iPoolStart++;
    return result; 
}
bool CLIWindow::windowExist()
{
    bool result;
    pthread_mutex_lock(&m_pOpenMtx);
    result = m_bWindowOpen;
    pthread_mutex_unlock(&m_pOpenMtx);
    return result;
}
void CLIWindow::run()
{
    int count = 0;
    int m_iKey = -1;
    memset(m_pUserInput,'\0',sizeof(char) * 256);
    while(m_bRun)
    {
        m_iKey=wgetch(m_pUserWin);
        if(m_iKey == -1)
            continue;
        else if(m_iKey == KEY_UP)
        {
            if(m_uiHighLight > m_uiIndex)
                 --m_uiHighLight;
            else
            {
                if(m_uiIndex == 0)
                     m_uiHighLight = m_uiIndex;
                else
                {
                     --m_uiIndex;
                     m_uiHighLight = m_uiIndex;
                     --m_uiTempLastIndex;
                }
            }
            refreshLog();
        }
        else if(m_iKey == KEY_DOWN)
        {   
            if(m_uiHighLight < ( m_uiTempLastIndex - 1) )
                ++m_uiHighLight;
            else
            {
                if(m_uiHighLight == (m_uiLastIndex - 1))
                     m_uiHighLight = m_uiLastIndex - 1;
                else
                {
                    ++m_uiIndex;
                    ++m_uiTempLastIndex;
                    ++m_uiHighLight;
                }
            }
            refreshLog();
        }
        else if(m_iKey == KEY_CONFIRM)
        {
            m_pUserInput[count] = '\0';
            count = 0;
            if(((m_iPoolEnd + 1) % 256) == m_iPoolStart)
            {
                continue;
            }
            memcpy(m_pUserInputPool[m_iPoolEnd],m_pUserInput,sizeof(char) * 256);
            m_iPoolEnd = (m_iPoolEnd + 1) % 256;
            memset(m_pUserInput,'\0',sizeof(char) * 256);
            refreshUser();  
            memset(m_pUserInput,'\0',sizeof(char) * 256);
        }
        else if(m_iKey == KEY_ESC)
        {
            closeWindow();
            break; 
        }
        else
        {   
            m_pUserInput[count] = m_iKey;
            count++; 
            wprintw(m_pUserWin,"%c",m_iKey);
        }
    }
}
void CLIWindow::log(const LogText& logDetail)
{
    if(m_uiLastIndex >= MAX_LOGS)
    {
        return ;
    }
    
    if(m_uiLastIndex >= m_uiMaxShowLines)
        m_uiIndex = m_uiLastIndex - m_uiMaxShowLines + 1;

    strcpy(m_pLogs[m_uiLastIndex].m_pStr, logDetail.m_pStr);
    m_pLogs[m_uiLastIndex].m_eLevel= logDetail.m_eLevel;
    m_pLogs[m_uiLastIndex].m_tTime = logDetail.m_tTime;
    m_uiHighLight = m_uiLastIndex;
    m_uiLastIndex++;
    m_uiTempLastIndex = m_uiLastIndex;
}
void CLIWindow::refreshLog()
{
    pthread_mutex_lock(&m_pOpenMtx);
    if(m_bWindowOpen == false)
    {
        pthread_mutex_unlock(&m_pOpenMtx);
        return ;
    }
    pthread_mutex_unlock(&m_pOpenMtx);

    if(m_uiIndex == m_uiTempLastIndex)
    {
        wrefresh(m_pLogWin);
        return;
    }
    int start = 1;
    for(int i = m_uiIndex;i<m_uiTempLastIndex;++i)
    {
        LogLevel level = m_pLogs[i].m_eLevel;
        const char* detail = m_pLogs[i].m_pStr;
        
        const char* s;
        switch(level)
        {
        case LogLevel::DEBUG:s = "DEBUG :\0";break;
        case LogLevel::INFO:s = "INFO  :\0";break;
        case LogLevel::WARN:s = "WARN  :\0";break;
        case LogLevel::ERROR:s = "ERROR :\0:";break;
        case LogLevel::FATAL:s = "FATAL :\0";break;
        }
        
        mvwprintw(m_pLogWin,start,0,"%s",BLANK); 
        unsigned int sLen = strlen(s) + 1;
        wattron(m_pLogWin,COLOR_PAIR(level) | A_BOLD | A_UNDERLINE);
        mvwprintw(m_pLogWin,start,1,"%s",s);
        wattroff(m_pLogWin,COLOR_PAIR(level) | A_BOLD | A_UNDERLINE);
        
        if(i == m_uiHighLight)
        {
            wattron(m_pLogWin,COLOR_PAIR(6));
        }
        else
            wattron(m_pLogWin, A_BOLD | A_UNDERLINE);
        struct tm *timenow;   
        timenow = localtime(&m_pLogs[i].m_tTime); 
        char* timeStr = asctime(timenow); 
        mvwprintw(m_pLogWin,start,sLen + 1,"%s",timeStr);
        sLen += strlen(timeStr) + 1;
        mvwprintw(m_pLogWin,start,sLen + 1,"%s",detail);
        sLen += strlen(timeStr) + 1;
    
        if(i == m_uiHighLight)
        {
            wattroff(m_pLogWin,COLOR_PAIR(6));
        }
        else
            wattroff(m_pLogWin, A_BOLD | A_UNDERLINE);
    
        wmove(m_pUserWin,3,strlen(m_pUserInput)+1);
        wattroff(m_pLogWin, A_BOLD | A_UNDERLINE);
        ++start;
    }
    wrefresh(m_pLogWin);
}

void CLIWindow::refreshUser()
{
    pthread_mutex_lock(&m_pOpenMtx);
    if(m_bWindowOpen == false)
    {
        pthread_mutex_unlock(&m_pRunMtx);
        return ;
    }
    pthread_mutex_unlock(&m_pOpenMtx);

    wattron(m_pUserWin,A_BOLD);
    mvwprintw(m_pUserWin,1,1,TITILELIST[(int)m_iUserTitleIndex]); 
    mvwprintw(m_pUserWin,2,1,INFOLIST[(int)m_iUserInfoIndex]);
    wattroff(m_pUserWin,A_BOLD);
    
    mvwprintw(m_pUserWin,3,0,"%s",BLANK);
    wmove(m_pUserWin,3,strlen(m_pUserInput)+1);
    wrefresh(m_pUserWin);
}

void CLIWindow::inputHint(const char* hintMsg)
{
    
}
int CLIWindow::getInput()
{
}

void CLIWindow::openWindow()
{
    pthread_mutex_lock(&m_pOpenMtx);  
    if(m_bWindowOpen)
    {
        pthread_mutex_unlock(&m_pOpenMtx);  
        return ; 
    }
    m_bWindowOpen = true;
    pthread_mutex_unlock(&m_pOpenMtx);  
    //init curses
    initscr();
    //get cols and rows of screen
    getmaxyx(stdscr,m_iHeight,m_iWidth);
    
    BLANK = new char[m_iWidth];
    memset(BLANK,' ',sizeof(char) * (m_iWidth));
    m_uiMaxShowLines = m_iHeight - WINDOW_HEIGHT - 2;
    m_bHasColor = has_colors();
    m_bRun = true;
    if(m_bHasColor)
    {
        start_color();
        INITCOLOR(INFO);
        INITCOLOR(DEBUG);
        INITCOLOR(WARNING);
        INITCOLOR(ERROR);
        INITCOLOR(FATAL);
        init_pair(6,COLOR_WHITE,COLOR_GREEN);
    }
    raw();  
    noecho();
    //set window position ,width,height
    m_pLogWin = newwin(m_iHeight - WINDOW_HEIGHT,m_iWidth,0,0);
    m_pUserWin = newwin(WINDOW_HEIGHT,m_iWidth,m_iHeight - WINDOW_HEIGHT,0);
    keypad(m_pUserWin,true);
    if(m_bHasColor)
    {
        init_pair(9,COLOR_RED,COLOR_GREEN);
        wattron(m_pLogWin,COLOR_PAIR(9));
        wattron(m_pUserWin,COLOR_PAIR(9));
    } 
    mvwprintw(m_pLogWin,0,(m_iWidth - strlen(LOGBAR))/2,LOGBAR);
    mvwprintw(m_pUserWin,0,(m_iWidth - strlen(USERINFO))/2,USERINFO);
    if(m_bHasColor)
    {
        wattroff(m_pLogWin,COLOR_PAIR(9));
        wattroff(m_pUserWin,COLOR_PAIR(9));
    }
    pthread_create(&m_pKeyThread,nullptr,&(CLIWindow::key),nullptr);
    refresh();
    refreshAll();
}
void CLIWindow::closeWindow()
{
    pthread_mutex_lock(&m_pRunMtx);
    m_bRun = false;
    pthread_mutex_unlock(&m_pRunMtx);

    pthread_mutex_lock(&m_pOpenMtx);
    if(m_bWindowOpen)
    {
        delwin(m_pLogWin);
        delwin(m_pUserWin);
        endwin(); 
        clear();
        m_bWindowOpen = false;
    }
    pthread_mutex_unlock(&m_pOpenMtx);
} 

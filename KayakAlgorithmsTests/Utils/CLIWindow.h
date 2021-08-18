#ifndef _CLIWINDOW_H_ 
#define _CLIWINDOW_H_

#include "CLIInter.h"
#include "KyComDef.h"
#include <time.h>

#define INITCOLOR(LEVEL) init_pair(LEVEL,LEVEL##COLOR,COLOR_BLACK);
#define KEY_ESC (27)
#define KEY_TAB (9)
#define KEY_CONFIRM (10)

class CLIWindow : public CLIInter
{
public:

    // print log in the log section
    void log(const LogText& logDetail);

    // get input from user
    void inputHint(const char* hintMsg);
    int getInput();
    // open and close window
    void openWindow();
    void closeWindow();
    void refreshAll(){refreshLog();refreshUser();}  
    void refreshLog();
    void refreshUser();
    bool windowExist();
    // refresh information
    void nextTitle(CLI_USER_TITLE index){m_iUserTitleIndex = index;};
    void nextInfo(CLI_USER_INFO index){m_iUserInfoIndex = index;};
    void run();
    static void* key(void* );
    const char* getUserInput();
    static CLIWindow* getInstance()
    {
        if(m_pInstance == nullptr)
            m_pInstance = new CLIWindow();
        return m_pInstance;
    }
    ~CLIWindow()
    {
        closeWindow();
    }
    
private:    
    //can't create by itself
    CLIWindow();
   

    //height of user window   
    const static int WINDOW_HEIGHT = 7;


    //log level and colors
    const static int DEBUG= 1;
    const static int INFO= 2;
    const static int WARNING = 3;
    const static int ERROR  = 4;
    const static int FATAL= 5;
    const static int DEBUGCOLOR = COLOR_GREEN;
    const static int INFOCOLOR = COLOR_BLUE;
    const static int WARNINGCOLOR = COLOR_YELLOW;
    const static int ERRORCOLOR = COLOR_MAGENTA;
    const static int FATALCOLOR = COLOR_RED;
    const char* TITILELIST[CLI_TITLE_COUNT]={
        "",
        "wating kukas connect",
        "do some preparing...",
        "setup environment"
    };
    const char* INFOLIST[CLI_INFO_COUNT] = {
        "",
        "waiting . . .",
        "building coordinate",
        "get trocar position",
        "prepare finished"
    };
    char* BLANK;  
    //window title  
    const char* const LOGBAR = "L   O    G";
    const char* const USERINFO = "U   S   E   R";

    // window info
    static CLIWindow* m_pInstance;
    bool m_bWindowOpen;
    int m_iWidth;
    int m_iHeight;  
    bool m_bHasColor;
    WINDOW* m_pLogWin;
    WINDOW* m_pUserWin;
    pthread_mutex_t m_pOpenMtx;

    //log buffer   
    unsigned int m_uiIndex = 0;
    unsigned int m_uiLastIndex = 0;
    unsigned int m_uiTempLastIndex = 0;
    unsigned int m_uiHighLight = 0;
    unsigned int m_iCurx = 0;
    unsigned int m_iCury = 0;
    
     
    LogText*  m_pLogs; 
    unsigned int m_uiMaxShowLines;//the max shown lines of info
    bool m_bRun;    //flag if the window is still alive
    pthread_mutex_t m_pRunMtx;
    CLI_USER_INFO m_iUserInfoIndex;//which info to show
    CLI_USER_TITLE m_iUserTitleIndex;//which title to show
    char m_pUserInput[256];//temp memory used to get input(line) from user
    //use array as queue;
    char m_pUserInputPool[256][256];
    int m_iPoolStart= 0;
    int m_iPoolEnd= 0;
    pthread_t m_pKeyThread;//thread used to get user input
};
#endif

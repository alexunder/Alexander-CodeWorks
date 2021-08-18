#ifndef _CLIInter_H_
#define _CLIInter_H_

#include <string>
#include <ncurses.h>

#include "LogInter.h"

// we will devide the xterm into two sections:
// the above section will display the system log
// the below section will display the input hint and get input from user
class CLIInter
{
public:
    // print log in the log section
    virtual void log(const LogText& logDetail) = 0;

    // get input from user
    virtual void inputHint(const char* hintMsg) = 0;
    virtual int getInput() = 0;
};

#endif


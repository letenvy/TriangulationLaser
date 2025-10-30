#include "ConsoleInput.hpp"

#ifdef _WIN32
    #include <conio.h>
#else
    #include <termios.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <cstdio>
#endif

ConsoleInput::ConsoleInput(){
#ifndef _WIN32
    if(tcgetattr(STDIN_FILENO,&orig_termios_)!=0){
        //intentionally empty
    }
    struct termios raw = orig_termios_;
    raw.c_lflag&=~(ICANON|ECHO);
    raw.c_cc[VMIN]=0;
    raw.c_cc[VTIME]=0;

    if (tcsetattr(STDIN_FILENO,TCSAFLUSH,&raw)==0){
        int flags=fcntl(STDIN_FILENO,F_GETFL,0);
        fcntl(STDIN_FILENO,F_SETFL,flags|O_NONBLOCK);
    }
#endif
}

ConsoleInput::~ConsoleInput(){
#ifndef _WIN32
    tcsetattr(STDIN_FILENO,TCSAFLUSH,&orig_termios_);
    int flags=fcntl(STDIN_FILENO,F_GETFL,0);
    fcntl(STDIN_FILENO,F_SETFL,flags &~O_NONBLOCK);
#endif
}

bool ConsoleInput::isEscapePressed(){
#ifdef _WIN32
    if(_kbhit()){
        int ch=_getch();
        return ch==27; //[ESC]
    }
    return false;
#else
    char ch;
    ssize_t n=read(STDIN_FILENO,&ch,1);
    if(n==1){
        return ch==27;
    }
    return false;
#endif
}
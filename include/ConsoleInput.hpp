#pragma once

class ConsoleInput{
public:
    ConsoleInput();
    ~ConsoleInput();
    bool isEscapePressed();
private:
#ifndef _WIN32
    struct termios orig_termios_;
#endif
};
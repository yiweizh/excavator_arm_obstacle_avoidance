#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
// #include "kbhit.hpp"
#ifndef KBHITh
#define KBHITh




static struct termios initial_settings, new_settings;
static int peek_character = -1;

void init_keyboard()
{
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0,TCSANOW,&new_settings);
}

void close_keyboard()
{
    tcsetattr(0,TCSANOW,&initial_settings);
}

/* Le o teclado sem bloquear o programa */
unsigned char kbhit()
{
    unsigned char ch;
    int nread;

    new_settings.c_cc[VMIN]= 0;
    tcsetattr(0,TCSANOW,&new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    if (nread==1)
    {
    return ch;
    }
    return 0;
}


#endif
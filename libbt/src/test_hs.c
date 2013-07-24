/**********************************************************************
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 **********************************************************************/

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <stdlib.h>
#include <sys/ioctl.h>

int open_ftdi_port(char* device, char* speed)
{
    struct termios g_initialAtt, newAtt;
    int port = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

    int ispeed = atoi(speed); //convert string to int //

    if (port == -1)
    {
        printf("ERROR port -1");
        exit(0);
    }

    printf("User specified speed as: %d \n",ispeed);

    printf("+%s\n",__func__);
    tcgetattr(port,&g_initialAtt);// save this to restore later
    newAtt=g_initialAtt;

    if(ispeed == 921600)
    {
        newAtt.c_cflag = B921600 | CS8 | CLOCAL | CRTSCTS; // Enable RTS CTS flow control
    } else if(ispeed == 2000000)
    {
        newAtt.c_cflag = CS8 | CLOCAL | CRTSCTS; // Enable RTS CTS flow control
    } else if(ispeed == 115200)
    {
        newAtt.c_cflag = B115200 | CS8 | CLOCAL | CRTSCTS; // Enable RTS CTS flow control
    } else //default rate: 115200
    {
        newAtt.c_cflag = B115200 | CS8 | CLOCAL | CRTSCTS; // Enable RTS CTS flow control
    }
    cfmakeraw(&newAtt);
    tcsetattr(port,TCSANOW,&newAtt);
    return port;
}

unsigned char char_to_hex(char c)
{
    if (c == '0')
        return 0x00;
    else if (c == '1')
        return 0x01;
    else if (c == '2')
        return 0x02;
    else if (c == '3')
        return 0x03;
    else if (c == '4')
        return 0x04;
    else if (c == '5')
        return 0x05;
    else if (c == '6')
        return 0x06;
    else if (c == '7')
        return 0x07;
    else if (c == '8')
        return 0x08;
    else if (c == '9')
        return 0x09;
    else if (c == 'A' || c == 'a')
        return 0x0a;
    else if (c == 'B' || c == 'b')
        return 0x0b;
    else if (c == 'C' || c == 'c')
        return 0x0c;
    else if (c == 'D' || c == 'd')
        return 0x0d;
    else if (c == 'E' || c == 'e')
        return 0x0e;
    else if (c == 'F' || c == 'f')
        return 0x0f;
    else
        return -1;
}

int byte2int(unsigned char c)
{
    printf("byte2int: %c\n", c);
    switch (c)
    {
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '0':
            return c-'0';
        case 'a':
        case 'A':
            return 10;
        case 'b':
        case 'B':
            return 11;
        case 'c':
        case 'C':
            return 12;
        case 'd':
        case 'D':
            return 13;
        case 'e':
        case 'E':
            return 14;
        case 'f':
        case 'F':
            return 15;
        default:
            return -1;
    }
}
void parse_cmd(int fd, char* cmd_str)
{
    struct timeval timeout;
    char n;
    // initialise the timeout structure
    timeout.tv_sec = 1; // ten second timeout
    timeout.tv_usec = 0;

    size_t cmd_size = strlen(cmd_str);
    int i;
    unsigned char c;
    for(i=0; i<cmd_size; i++)
    {
        if (i%2 == 1)
        {
            c |= char_to_hex(cmd_str[i]);
            printf(" %x ",c);
            write(fd, &c, 1);
        }
        else
        {
            c = char_to_hex(cmd_str[i]) << 4;
        }
    }

//    printf("Event: \n");
    //unsigned char buf[40];
    unsigned char bc;
    int event_started = 0;
    unsigned int cc = 0, event_code = 0;
    unsigned int x = 0, len = 0;
    //bzero(buf,40);
    i = 0;
    //sleep(1);
    do{
        //n = read(fd, &buf[i], 1);
        n = read(fd, &bc, 1);
        //val = atoi(&buf[i]);
        sscanf(&bc, "%c", &x);

        if (x == 4 && event_started == 0)
        {
            printf("\n\t>> %x ",bc);
            event_started = 1;
            cc = 0;
        }
        else if (event_started == 1)
        {
            cc++;
            if (cc == 1)
            {
                event_code = x;
                if (x == 15) //Debug stratup event. Skip this event
                {
                    printf ("Debug Startup event\n");
                    break;
                }
                printf (" %x ", bc); //Event code
            }
            else if (cc == 2)
            {
                printf(" %x ",bc);
                len = x;
            }
            else if (cc >= 2) {
                printf(" %x ", bc);
                len--;
            }

            if (len <= 0 && cc >= 2)
                break;
        }
        i++;
    }while(1);

    if (event_code == 14)
        printf (" (Command complete)\n");
    // check if an error has occured
    if(n < 0)
    {
     perror("select failed\n");
    }
    else if (n == 0)
    {
     puts("Timeout!");
    }
    else
    {
     //printf("\nBytes detected on the port!\n");
     printf ("\n");
     return;
    }
}

void open_file(int fd, char* filename)
{
    FILE * fp;
    char line[1024];
    size_t len = 0;
    ssize_t read;
    char* ret = NULL;
    fp = fopen(filename, "r");
    if (fp == NULL)
       exit(1);

    while (1) {
       ret = fgets (line, sizeof (line), fp);
       if (ret == NULL || strcmp(ret,"") == 0 || strlen(line) <= 0)
               break;
       line[strlen(line)-1] = '\0';
       if (line[0] == '#') //comment line. Print it
       {
               printf("%s\n", line);
               continue;
        }
       //printf(":%s:\n", line);
       parse_cmd(fd, line);
       ret = NULL;
    }
    fclose(fp);
}

int main(int argc, char* argv[])
{
    unsigned char* cmd;
    printf("OPEN PORT\n");
    int fd = -1;

    //Check if user passes enough arguments //
    if (argc > 3)
    {
         if(fd == -1)
        {
            fd = open_ftdi_port(argv[1],argv[3]);
            printf("FILE:%s\n",argv[2]);
            open_file(fd, argv[2]);
        }
    }
    else
        printf(">>> Invalid args. Usage: exe <serial_port> <file_name> <speed> <<< \n");

    if(fd != -1)
      close(fd);

    return(0);

}

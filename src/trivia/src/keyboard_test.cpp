#include <sys/file.h>
#include <stdio.h>
#include <string.h>
#include <linux/input.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <music_player_client.h>



int main (int argc, char *argv[])
{
        ros::init(argc, argv, "keyboard_test");
        ros::NodeHandle n;
        struct input_event ev, ev1;
        musicClient player;
        int fd, rd;
        int fd1,rd1;
        int finish=0;
        //Open Device
        if ((fd = open ("/dev/input/event12", O_RDONLY|O_NONBLOCK)) == -1){
                printf ("not a valid device.\n");
                return -1;
        }
/*        if ((fd1 = open ("/dev/input/event12", O_RDONLY|O_NONBLOCK)) == -1){
                printf ("not a valid device.\n");
                return -1;
        }*/

        while (finish<20){

                memset((void*)&ev, 0, sizeof(ev));
                // memset((void*)&ev1, 0, sizeof(ev1));

                rd = read (fd, (void*)&ev, sizeof(ev));
                // rd1 = read (fd1, (void*)&ev1, sizeof(ev1));

                if(rd>0 && ev.value==1){
                        printf("ev.code = [%d]",ev.code);
                        if(ev.code==76){
                          printf("Response Detected!\n;");
                          player.beep();
                        }
                        finish++;
                }
                // if(rd1>0){
                //         printf("type: %d, code: %d, value: %d, rd: %d\n", ev1.type, ev1.code, ev1.value, rd1);
                //         finish++;
                // }
                
                
        }
        close(fd);
        // close(fd1);
        return 0;
}
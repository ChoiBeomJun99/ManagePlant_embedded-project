#include <stdio.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define DEV_NAME "proj_ioctl_dev"
#define IOCTL_START_NUM 0x80
#define IOCTL_NUM1 IOCTL_START_NUM+1
#define IOCTL_NUM2 IOCTL_START_NUM+2

#define PROJ_IOCTL_NUM 'z'
#define PROJ_IOCTL1 _IOWR(PROJ_IOCTL_NUM, IOCTL_NUM1, unsigned long)
#define PROJ_IOCTL2 _IOWR(PROJ_IOCTL_NUM, IOCTL_NUM2, unsigned long*)

void main(void){
    int dev;
    dev = open("/dev/proj_ioctl_dev", O_RDWR);
    
    ioctl(dev, PROJ_IOCTL1, 0); //그냥 물 주기
    ioctl(dev, PROJ_IOCTL2, 1); //1분 뒤에 작동하도록

    close(dev);
}

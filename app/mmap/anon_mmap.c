#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <string.h>
#include <fcntl.h>

int main(int argc,char *argv[])
{
    int len = 4096;

    //创建匿名内存映射区
    void *ptr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANON,-1, 0);
    if(ptr == MAP_FAILED) {
        perror("mmap error");
        exit(1);
    }
    printf("mmap: 0x%x\n", ptr);

    //创建子进程
    pid_t pid = fork();
    if(pid == -1) {
        perror("fork error");
        exit(1);
    } else if(pid > 0) {
        //写数据
        strcpy((char *)ptr,"菜虚坤打篮球");
        //回收
        wait(NULL);
    } else if(pid == 0) {
        //读数据
        printf("%s\n",(char *)ptr);
    }
    //释放映射区内存
    int ret = munmap(ptr,len);
    if(ret == -1) {
        perror("munmap error");
        exit(1);
    }

    return 0;
}
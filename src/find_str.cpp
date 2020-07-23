#include <stdio.h>
#include <libusb.h>
#include <malloc.h>
#include <unistd.h>
#include <pthread.h>
#include "find_str.h"

int find_str(const char *input_str,unsigned char *data,int len)
{
    const char *str = input_str;
    int str_len = strlen(input_str);
    int index = 0;
    for(int i=0;i<len;i++)
    {
        if(data[i] == str[index])
        {
            //DBG("find %c %d %d",str[index],index,this->len);
            index++;
            if(index >= str_len)
            {
                return i+1;
            }
        }else{
            index = 0;
            if(data[i] == str[index])
            {
                //DBG("find %c %d %d",str[index],index,this->len);
                index++;
                if(index >= str_len)
                {
                    return i+1;
                }
            }
        }
    }
    
    return 0;
}
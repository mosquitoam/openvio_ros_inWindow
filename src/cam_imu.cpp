//#include <ros/ros.h>
#include <stdio.h>
#include <libusb.h>
#include <malloc.h>
#include <unistd.h>
#include <pthread.h>
#include "cam_imu.h"

using namespace std;
// using namespace cv;
// using namespace ros;

#define CTRL_EPADDR     0x01
#define CAM_EPADDR      0x81
#define IMU_EPADDR      0x82

#define USB_TIMEOUT     1000 //传输数据的时间延迟

#define GX_OFFSET       0
#define GY_OFFSET       0
#define GZ_OFFSET       0

#define IMU_BUF_SIZE    14
#define IMG_BUF_SIZE    752 * 480
#define M_PI            3.14159265358979323846

#define REQUEST_CAMERA_START    0xA0
#define REQUEST_CAMERA_STOP     0xA1
#define REQUEST_IMU_START       0xB0
#define REQUEST_IMU_STOP        0xB1

unsigned char *imu_buf;
unsigned char *img_buf;

int img_cnt = 0, imu_cnt = 0;
float imu_buffer[6];
libusb_device_handle *dev_handle;

unsigned char imu_flag=0,img_flag=0;

Mat img_cam(Size(752, 480), CV_8UC1);

ros::Time imu_time;
ros::Time img_time;

Time img_get(Mat img_data)
{
    img_cnt++;
    memcpy(img_data.data, img_buf, IMG_BUF_SIZE);

    return img_time;
}

Time imu_get_data(float *buf)
{
    imu_cnt++;
    for (unsigned char i = 0; i < 6; i++)
    {
        buf[i] = imu_buffer[i];
    }
        printf("%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t\r\n",
                    imu_buffer[0],imu_buffer[1],imu_buffer[2],
                    imu_buffer[3],imu_buffer[4],imu_buffer[5]);

    return imu_time;
}

void *cam_catch_thread(void *)
{
    int ret = 0;
    static int img_recv_index = 0;
    int camRecvLen;

    printf("cam_catch_thread start\r\n");

    while (1)
    {
        ret = libusb_bulk_transfer(dev_handle, CAM_EPADDR, (unsigned char *)(img_buf + img_recv_index), IMG_BUF_SIZE, &camRecvLen, USB_TIMEOUT);
        if (ret < 0)
        {
            if (ret != -7)
            {
                printf("cam recv error %d\r\n", ret);
                break;
            }
        }
        else
        {
            img_recv_index += camRecvLen;

            if (img_recv_index >= IMG_BUF_SIZE)
            {
                img_time = ros::Time::now();
                img_cnt++;
                img_flag = 1;
                img_recv_index = 0;
            }
        }
    }

    printf("cam_catch_thread exit\r\n");
    pthread_exit(NULL);
}



float buf_data[3][10];
unsigned char lvbo_cnt=0;
float lvbo_data[3],lvbo_gyro_data[3];

void *imu_catch_thread(void *)
{
    int imuRecvLen;
    printf("imu_catch_thread start\r\n");

    while (1)
    {

        libusb_bulk_transfer(dev_handle, IMU_EPADDR, imu_buf, IMU_BUF_SIZE, &imuRecvLen, USB_TIMEOUT);

        imu_time = ros::Time::now();
        //imu_time_cnt = (unsigned int)(imu_buf[14] << (8 * 3) | imu_buf[15] << (8 * 2) | imu_buf[16] << (8 * 1) | imu_buf[17]);
        //img_time_cnt = (unsigned int)(imu_buf[19] << (8 * 3) | imu_buf[20] << (8 * 2) | imu_buf[21] << (8 * 1) | imu_buf[22]);

        // ros::Time ttime((double)imu_time_cnt / 1000);
        // imu_time = ttime;
        // ros::Time tttime((double)img_time_cnt / 1000);
        // img_time = tttime;

        int16_t gx = (((0xff & (char)imu_buf[4 * 2]) << 8) | 0xff & (char)imu_buf[4 * 2 + 1]);
        int16_t gy = (((0xff & (char)imu_buf[5 * 2]) << 8) | 0xff & (char)imu_buf[5 * 2 + 1]);
        int16_t gz = (((0xff & (char)imu_buf[6 * 2]) << 8) | 0xff & (char)imu_buf[6 * 2 + 1]);

        imu_buffer[3] = gx * (4000.0 / 65536.0) * (M_PI / 180.0) - GX_OFFSET;
        imu_buffer[4] = gy * (4000.0 / 65536.0) * (M_PI / 180.0) - GY_OFFSET;
        imu_buffer[5] = gz * (4000.0 / 65536.0) * (M_PI / 180.0) - GZ_OFFSET;

        // get acelerometer values
        int16_t ax = (((0xff & (char)imu_buf[0 * 2]) << 8) | 0xff & (char)imu_buf[0 * 2]);
        int16_t ay = (((0xff & (char)imu_buf[1 * 2]) << 8) | 0xff & (char)imu_buf[1 * 2]);
        int16_t az = (((0xff & (char)imu_buf[2 * 2]) << 8) | 0xff & (char)imu_buf[2 * 2]);

        // calculate accelerations in m/s²
        imu_buffer[0] = ax * (16.0 / 65536.0) * 9.8015f;
        imu_buffer[1] = ay * (16.0 / 65536.0) * 9.8015f;
        imu_buffer[2] = az * (16.0 / 65536.0) * 9.8015f;

        buf_data[0][lvbo_cnt]=imu_buffer[0];
        buf_data[1][lvbo_cnt]=imu_buffer[1];
        buf_data[2][lvbo_cnt]=imu_buffer[2];
        lvbo_data[0]=0.0;
        lvbo_data[1]=0.0;
        lvbo_data[2]=0.0;
        for(int ii=0;ii<10;ii++)
        {
            lvbo_data[0]+=buf_data[0][ii];
            lvbo_data[1]+=buf_data[1][ii];
            lvbo_data[2]+=buf_data[2][ii];
        }
        imu_buffer[0]=(lvbo_data[0]/10);
        imu_buffer[1]=(lvbo_data[1]/10);
        imu_buffer[2]=(lvbo_data[2]/10);
        lvbo_cnt++;
        if(lvbo_cnt==10)
            lvbo_cnt=0;

        imu_buffer[0] = 0.9953f * imu_buffer[0] - 0.3479f;
        imu_buffer[1] = 0.9952f * imu_buffer[1] + 0.1601f;
        imu_buffer[2] = 0.9868f * imu_buffer[2] - 0.0253f;
        imu_flag=1;

        imu_cnt++;
    }

    printf("imu_catch_thread exit\r\n");
    pthread_exit(NULL);
}
void *img_show_thread(void *)
{
    while (1)
    {

    }
    pthread_exit(NULL);
}

//int main()
int openvio_init(unsigned char flag)
{
    unsigned char ctrl_buffer[10];
    libusb_device **devs;
    libusb_context *ctx = NULL;
    int r;



    imu_buf = (unsigned char *)malloc(IMU_BUF_SIZE);
    img_buf = (unsigned char *)malloc(IMG_BUF_SIZE*2);


    r = libusb_init(&ctx);
    if (r < 0)
    {
        printf("Init Error\r\n");
        return 1;
    }

    dev_handle = libusb_open_device_with_vid_pid(ctx, 0x07DC, 0x07DC);
    if (dev_handle == NULL)
    {
        printf("Not found 0x07DC 0x07DC device,exit.\r\n");
        libusb_exit(ctx);
        return 1;
    }
    else
    {
        printf("found 0x07DC 0x07DC device.\r\n");
    }

    r = libusb_claim_interface(dev_handle, 0); //声明设备接口
    if (r < 0)
    {
        printf("Error:libusb_claim_interface\r\n");
        return 1;
    }

    r = libusb_control_transfer(dev_handle, LIBUSB_REQUEST_TYPE_VENDOR + +LIBUSB_ENDPOINT_IN, REQUEST_CAMERA_START, 0, 0, ctrl_buffer, 1, 1000);
    if (r < 0)
    {
        printf("cam libusb_control_transfer fail\r\n");
        return 1;
    }
    else
    {
        printf("cam libusb_control_transfer success %c\r\n", ctrl_buffer[0]);
    }

    libusb_control_transfer(dev_handle, LIBUSB_REQUEST_TYPE_VENDOR + +LIBUSB_ENDPOINT_IN, REQUEST_IMU_START, 0, 0, ctrl_buffer, 1, 1000);
    if (r < 0)
    {
        printf("imu libusb_control_transfer fail\r\n");
        return 1;
    }
    else
    {
        printf("imu libusb_control_transfer success %c\r\n", ctrl_buffer[0]);
    }

    pthread_t cam_thread;
    if (pthread_create(&cam_thread, NULL, cam_catch_thread, NULL))
        printf("Failed to create thread cam_catch_thread\r\n");

    pthread_t imu_thread;
    if (pthread_create(&imu_thread, NULL, imu_catch_thread, NULL))
        printf("Failed to create thread imu_catch_thread\r\n");

    //pthread_t img_s_thread;
    //if(pthread_create(&img_s_thread, NULL, img_show_thread, NULL))
    //	printf("Failed to create thread img_show_thread\r\n");

    // while (1)
    // {
    //     sleep(1);
    //     printf("%dfps\t%dHz\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t\r\n",
    //                 img_cnt, imu_cnt,
    //                 imu_buffer[0],imu_buffer[1],imu_buffer[2],
    //                 imu_buffer[3],imu_buffer[4],imu_buffer[5]);

    //     img_cnt = 0;
    //     imu_cnt = 0;
    // }

    //libusb_close(dev_handle);
    //libusb_exit(ctx);
    return 0;
}

//g++ cam_imu.cpp -o cam_imu -I /usr/include/libusb-1.0 /usr/local/lib/libusb-1.0.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_highgui.so -lpthread -I /opt/ros/kinetic/include/opencv-3.3.1-dev

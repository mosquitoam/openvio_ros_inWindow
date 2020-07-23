#ifndef __FIND_STR_H__
#define __FIND_STR_H__

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


int find_str(const char *input_str,unsigned char *data,int len);

#endif

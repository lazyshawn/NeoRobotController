/***********************************************************************
 * 控制卡接口文件                                                      *
 * 																	   *
 * 包含了跨平台编译需要的头文件，如控制卡头文件、MSCV头文件等		   *
 ***********************************************************************/
#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // !M_PI

#ifdef _MSC_VER
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#else
#include "zmcbuildin.h"
#define NULL ((void *)0)
#endif

#ifdef __cplusplus
extern "C" {
#else
typedef enum { false = 0, true = 1 } bool;
#endif

#ifdef __cplusplus
}
#endif

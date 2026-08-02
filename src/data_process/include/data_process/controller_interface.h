/***********************************************************************
 * 控制卡接口文件                                                      *
 * 																	   *
 * 包含了跨平台编译需要的头文件，如控制卡头文件、MSCV头文件等		   *
 ***********************************************************************/
#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // !M_PI

#if defined(_WIN32) && defined(_MSC_VER)
#include "common/ExportSharedAPI.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#elif defined(__linux__) && defined(__GNUC__)
#include "common/ExportSharedAPI.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#else
#include "zmcbuildin.h"
#define NULL ((void *)0)
#endif

#ifdef __cplusplus
extern "C" {
#elif __STDC_VERSION__ < 202311L
typedef enum { false = 0, true = 1 } bool;
#endif

#ifdef __cplusplus
}
#endif

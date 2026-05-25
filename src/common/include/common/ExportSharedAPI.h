#pragma once

#ifdef EXPORT_STATIC_LIBS
// =====================================================
// 1. 静态库模式：所有宏为空
// =====================================================
#define SHARE_API_
#define EXTERN_API_

#elif defined(_WIN32) || defined(__CYGWIN__)
// =====================================================
// 2. Windows 动态库
// =====================================================
#ifdef EXPORT_SHARED_LIBS
// 导出符号
#define SHARE_API_ __declspec(dllexport)
#else
// 导入符号
#define SHARE_API_ __declspec(dllimport)
#endif
#define EXTERN_API_ extern

#else
// =====================================================
// 3. Linux 动态库（已全局 -fvisibility=hidden）
// =====================================================
#ifdef EXPORT_SHARED_LIBS
#define SHARE_API_ __attribute__((visibility("default")))
#else
#define SHARE_API_
#endif
#define EXTERN_API_ extern
#endif

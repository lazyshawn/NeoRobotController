
#include <vector>
#include <Windows.h>
#include <DbgHelp.h>
#include <time.h>
#include <stdio.h>
#include <stdexcept>
#include <crtdbg.h>

LONG WINAPI CrashHandler(EXCEPTION_POINTERS* pException)
{
    SYSTEMTIME st;
    GetLocalTime(&st);
    
    char dumpFileName[MAX_PATH];
    sprintf_s(dumpFileName, MAX_PATH, "crash_dump_%04d%02d%02d_%02d%02d%02d.dmp",
        st.wYear, st.wMonth, st.wDay,
        st.wHour, st.wMinute, st.wSecond);

    HANDLE hFile = CreateFileA(dumpFileName, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hFile != INVALID_HANDLE_VALUE)
    {
        MINIDUMP_EXCEPTION_INFORMATION exceptionInfo;
        exceptionInfo.ThreadId = GetCurrentThreadId();
        exceptionInfo.ExceptionPointers = pException;
        exceptionInfo.ClientPointers = FALSE;

        BOOL success = MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(),
            hFile, MiniDumpNormal, &exceptionInfo, NULL, NULL);
        
        CloseHandle(hFile);
        
        if (success) {
            printf("Dump file created: %s\n", dumpFileName);
        } else {
            printf("Failed to create dump file. Error: %lu\n", GetLastError());
        }
    }
    else {
        printf("Failed to create file. Error: %lu\n", GetLastError());
    }

    return EXCEPTION_EXECUTE_HANDLER;
}

int main() {
    //printf("Registering crash handler...\n");
	SetUnhandledExceptionFilter(CrashHandler);
	
    //printf("Triggering crash...\n");
    //fflush(stdout);
    
    // 使用更直接的方式触发崩溃
    int* p = nullptr;
    *p = 0;

	//int a = 1;
	//int b = 0;
	//int c = a / b;

	//std::vector<double> x;
	//x[0] = 1;
	
	return 0;
}

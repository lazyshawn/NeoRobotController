@echo off

cd /d "%~dp0build_ninja"

echo Loading vcvars64.bat
call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\VC\Auxiliary\Build\vcvars64.bat"

echo Build using ninja...
cmake -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_MAKE_PROGRAM=ninja ..

echo Build finished.

if exist compile_commands.json (
    copy /y compile_commands.json ..\
    echo copy compile_commands.json.
) else (
    echo compile_commands.json not exist.
)
@echo off
echo ===================================================
echo   Building Multinet GDExtension (Ninja + MSVC)
echo ===================================================

call "D:\VS\2022ce\VC\Auxiliary\Build\vcvars64.bat"
if %errorlevel% neq 0 (
    echo Error initializing MSVC environment.
    exit /b %errorlevel%
)

cmake --build "%~dp0multinet_ext\build\windows-editor" --parallel 7
if %errorlevel% neq 0 (
    echo Build failed!
    exit /b %errorlevel%
)

echo.
echo ===================================================
echo   Multinet GDExtension built ^& deployed successfully!
echo ===================================================

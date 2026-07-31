@echo off
echo ===================================================
echo   Building Multinet GDExtension (Ninja + MSVC)
echo ===================================================

call "D:\VS\2022ce\VC\Auxiliary\Build\vcvars64.bat"
if %errorlevel% neq 0 (
    echo Error initializing MSVC environment.
    exit /b %errorlevel%
)

python "%~dp0multinet_ext\scripts\gate_rule3_allocations.py"
if %errorlevel% neq 0 (
    echo Rule 3 Gate failed! Halting build.
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

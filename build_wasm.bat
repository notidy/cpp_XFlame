@echo off
setlocal

if /I "%~1"=="-r" goto release
if /I "%~1"=="-d" goto debug
if "%~1"=="" goto debug

echo Unknown option: [%~1]
echo.
echo Usage:
echo   build_wasm.bat -d    Debug build (default)
echo   build_wasm.bat -r    Release build
exit /b 1

:debug
set "BUILD_MODE=debug"
goto build

:release
set "BUILD_MODE=release"
goto build

:build
echo.
echo ========================================
echo Building XFlame %BUILD_MODE%
echo ========================================
echo.

call D:\New\emsdk\emsdk_env.bat

if not exist dist mkdir dist

if "%BUILD_MODE%"=="debug" (
    set "OPT_FLAGS=-O0 -g3"
    set "DEBUG_FLAGS=-s ASSERTIONS=2 -s SAFE_HEAP=1 -s STACK_OVERFLOW_CHECK=2"
    set "OUTPUT=dist/xflame.js"
) else (
    set "OPT_FLAGS=-O3 -flto"
    set "DEBUG_FLAGS="
    set "OUTPUT=dist/xflame.js"
)

echo.

REM ========================================
REM Compile
REM ========================================

em++ ^
-I ./include ^
src/XFlame.cpp ^
src/XDataUtil.cpp ^
src/XFlameWeb.cpp ^
%OPT_FLAGS% ^
%DEBUG_FLAGS% ^
-msimd128 ^
-s WASM=1 ^
-s MODULARIZE=1 ^
-s EXPORT_ES6=1 ^
-s ENVIRONMENT=web ^
-s FILESYSTEM=0 ^
-s DISABLE_EXCEPTION_CATCHING=1 ^
-fno-exceptions ^
-fno-rtti ^
-s ALLOW_MEMORY_GROWTH=1 ^
-s EXPORTED_FUNCTIONS="['_malloc','_free','_XFlame_Create','_XFlame_Destroy','_XFlame_Init','_XFlame_ComputeFrame','_XFlame_GetDriveMeshVtsPtr','_XFlame_GetDriveMeshVtsCount','_XFlame_GetDriveMeshFacePtr','_XFlame_GetDriveMeshFaceCount','_XFlame_GetNFrames','_XFlame_GetDriveMeshRestNormalPtr','_XFlame_GetDriveMeshNormalPtr','_XFlame_GetCompParmPtr','_XFlame_GetCamParmPtr','_XFlame_GetRenderSizePtr']" ^
-s EXPORTED_RUNTIME_METHODS="['HEAP8','HEAPU8','HEAPU32','HEAP32','HEAPF32','wasmMemory']" ^
-o %OUTPUT%

echo.
if %ERRORLEVEL% neq 0 (
    echo Build FAILED!
    exit /b 1
) else (
    echo Build SUCCESS!
)

endlocal
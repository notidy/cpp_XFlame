@echo off

call D:\New\emsdk\emsdk_env.bat

if not exist dist mkdir dist

em++ ^
-I ./include ^
src/XFlame.cpp ^
src/XDataUtil.cpp ^
src/XFlameWeb.cpp ^
-O3 ^
-flto ^
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
-o dist/xflame.js

echo.
if %ERRORLEVEL% neq 0 (
    echo Build FAILED!
) else (
    echo Build SUCCESS!
)

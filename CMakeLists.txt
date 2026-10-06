cmake_minimum_required(VERSION 3.16)
project(AdasPipeline VERSION 0.1.0 LANGUAGES CXX)

# ---------- 1. C++ standard ----------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------- 2. Postavke kompajlera (upozorenja) ----------
add_library(project_options INTERFACE)
if(MSVC)
  target_compile_options(project_options INTERFACE /W4 /permissive- /utf-8)
else()
  target_compile_options(project_options INTERFACE -Wall -Wextra -Wpedantic)
endif()

# ---------- 3. OpenCV ----------
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc videoio highgui dnn)

# ---------- 4. ONNX Runtime ----------
set(ONNXRUNTIME_ROOT "C:/onnxruntime" CACHE PATH "Putanja do ONNX Runtime foldera")

find_path(ONNXRUNTIME_INCLUDE_DIR NAMES onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
  HINTS ${ONNXRUNTIME_ROOT}/lib)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime nije pronađen u ${ONNXRUNTIME_ROOT}")
endif()

add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION             "${ONNXRUNTIME_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}")

# ---------- 5. Naša biblioteka s logikom ----------
add_library(adas_core STATIC
  src/FrameGrabber.cpp
  src/Detector.cpp
  src/AdasLogic.cpp)

target_include_directories(adas_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(adas_core
  PUBLIC  ${OpenCV_LIBS} onnxruntime::onnxruntime
  PRIVATE project_options)

# ---------- 6. Izvršni program ----------
add_executable(adas_pipeline src/main.cpp)
target_link_libraries(adas_pipeline PRIVATE adas_core project_options)

# ---------- 7. Windows: kopiraj onnxruntime.dll pored .exe ----------
if(WIN32)
  add_custom_command(TARGET adas_pipeline POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${ONNXRUNTIME_ROOT}/lib/onnxruntime.dll"
            $<TARGET_FILE_DIR:adas_pipeline>)
endif()
cmake_minimum_required(VERSION 3.16)
project(AdasPipeline VERSION 0.1.0 LANGUAGES C CXX)

# ---------- 1. Standardi jezika ----------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------- 2. Postavke kompajlera (upozorenja) ----------
add_library(project_options INTERFACE)
if(MSVC)
  target_compile_options(project_options INTERFACE
    /W4 /utf-8 $<$<COMPILE_LANGUAGE:CXX>:/permissive->)
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

# ---------- 5. C biblioteka: CAN poruke (dijele je ADAS kamera i Brake ECU) ----------
add_library(adas_msg STATIC common/src/adas_msg.c)
target_include_directories(adas_msg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/common/include)
target_link_libraries(adas_msg PRIVATE project_options)

# ---------- 5b. OSAL: mreža i vrijeme za Windows i POSIX (C) ----------
add_library(osal STATIC common/src/osal.c)
target_include_directories(osal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/common/include)
target_link_libraries(osal PRIVATE project_options)
if(WIN32)
  target_link_libraries(osal PRIVATE ws2_32)   # Winsock biblioteka
endif()

# ---------- 6. C++ biblioteka s ADAS logikom ----------
add_library(adas_core STATIC
  src/FrameGrabber.cpp
  src/Detector.cpp
  src/AdasLogic.cpp
  src/Tracker.cpp)

target_include_directories(adas_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(adas_core
  PUBLIC  ${OpenCV_LIBS} onnxruntime::onnxruntime
  PRIVATE project_options)

# ---------- 7. ADAS kamera (izvršni program) ----------
add_executable(adas_pipeline src/main.cpp)
target_link_libraries(adas_pipeline PRIVATE adas_core project_options)

# ---------- 8. Windows: kopiraj onnxruntime.dll pored .exe ----------
if(WIN32)
  add_custom_command(TARGET adas_pipeline POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${ONNXRUNTIME_ROOT}/lib/onnxruntime.dll"
            $<TARGET_FILE_DIR:adas_pipeline>)
endif()

# ---------- 9. Testovi ----------
enable_testing()

add_executable(test_adas_msg tests/test_adas_msg.c)
target_link_libraries(test_adas_msg PRIVATE adas_msg project_options)
if(NOT MSVC)
  target_link_libraries(test_adas_msg PRIVATE m)   # matematička biblioteka (fabsf) na Linuxu
endif()

add_test(NAME adas_msg_test COMMAND test_adas_msg)

# ---------- 10. Brake ECU (C): prima CAN poruke preko UDP-a ----------
add_executable(brake_ecu ecu/brake_ecu.c ecu/brake_logic.c)
target_link_libraries(brake_ecu PRIVATE adas_msg osal project_options)

# ---------- 11. Fake camera (C): šalje testne poruke ----------
add_executable(fake_camera tools/fake_camera.c)
target_link_libraries(fake_camera PRIVATE adas_msg osal project_options)
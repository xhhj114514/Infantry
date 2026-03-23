cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Setup compiler settings
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_C_FLAGS   "" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "" CACHE STRING "" FORCE)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# # # 强制将 compile_commands.json 复制到项目根目录,需要ctrl+shift+p重启clangd语言服务器
# file(COPY "${CMAKE_BINARY_DIR}/compile_commands.json"
#      DESTINATION "${CMAKE_SOURCE_DIR}")


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Infantry)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# 编译选项：只给 GCC（不传给 ld，避免 unrecognized）
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wno-gnu-binary-literal  # 抑制二进制常量
    -Wno-gnu                 # 抑制其他 GCC 扩展
    -Wno-unused-variable     # 未用变量
    -Wno-unused-function     # 未用函数
    -Wno-unused-parameter    # 未用参数（已存在，可忽略）
    -Wno-pedantic            # 空文件和指针转换
    -Wno-misleading-indentation  # 缩进误导
    -Wno-switch              # switch 未覆盖
    -Wno-implicit-fallthrough    # 落通过
    -Wno-comment             # 注释问题
)

# 链接选项：纯 ld 选项，避免混入 gcc 警告抑制
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,--gc-sections        # 垃圾回收
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)


##############以下为修改过的cmakelist#################
function(USER_ADD_DIR dir)
    file(GLOB USER_SOURCE
        ${dir}/*.h
        ${dir}/*.c
        ${dir}/*.hpp
        ${dir}/*.cpp
        ${dir}/*.cc
    )
    target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${USER_SOURCE})
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${dir})
endfunction()
###############################在此处添加你的文件路径#########
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/adc)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/can)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/dwt)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/flash)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/gpio)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/iic)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/log)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/pwm)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/spi)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/usart)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/bsp/usb)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/alarm)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/algorithm)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/BMI088)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/daemon)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/imu)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/ist8310)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/lqr)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/master_machine)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/matrix)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/message_center)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/DJImotor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/DMmotor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/MImotor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/motor/servo_motor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/referee)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/remote)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/super_cap)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/Trajectory_planning)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/can_comm)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Modules/ui)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/chassis)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/cmd)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/gimbal)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/shoot)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/application/gimbal_algorithm)


# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wno-gnu
    -Wno-gnu-binary-literal
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)



# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

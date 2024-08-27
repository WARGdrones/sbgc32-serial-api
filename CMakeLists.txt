cmake_minimum_required(VERSION 3.5)
project(sbgc32-serial-api VERSION 0.1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_BUILD_TYPE Debug)

# disable optimizations to prevent stack smashing
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-stack-protector")
# set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-stack-protector")

# Set source files path
set(PATH_TO_SERIAL_API serialAPI)
set(PATH_TO_CORE serialAPI/core)
set(PATH_TO_DRIVERS serialAPI/drivers/driverLinux)
set(PATH_TO_MODULES serialAPI/modules)
set(PATH_TO_OS_GLUE serialAPI/os)


# Add source files
set(SOURCES
    ${PATH_TO_CORE}/debug/debug.c
    ${PATH_TO_CORE}/highLayer/highLayer.c
    ${PATH_TO_CORE}/parser/parser.c
    ${PATH_TO_CORE}/commandBuild.c
    ${PATH_TO_CORE}/lowLayer.c

    ${PATH_TO_DRIVERS}/driverLinux.c

    ${PATH_TO_MODULES}/adjvar/adjvar.c
    ${PATH_TO_MODULES}/calib/calib.c
    ${PATH_TO_MODULES}/eeprom/eeprom.c
    ${PATH_TO_MODULES}/gimbalControl/gimbalControl.c
    ${PATH_TO_MODULES}/imu/imu.c
    ${PATH_TO_MODULES}/profiles/profiles.c
    ${PATH_TO_MODULES}/realtime/realtime.c
    ${PATH_TO_MODULES}/service/service.c

    ${PATH_TO_OS_GLUE}/glueAzureRTOS/glueAzureRTOS.c
    ${PATH_TO_OS_GLUE}/glueFreeRTOS/glueFreeRTOS.c
    ${PATH_TO_OS_GLUE}/glueLinuxOS/glueLinuxOS.c

    ${PATH_TO_SERIAL_API}/sbgc32.c
)

# Add header files
set(HEADERS
    ${PATH_TO_CORE}/debug/debug.h
    ${PATH_TO_CORE}/highLayer/highLayer.h
    ${PATH_TO_CORE}/parser/parser.h
    ${PATH_TO_CORE}/core.h

    ${PATH_TO_DRIVERS}/driverLinux.h

    ${PATH_TO_MODULES}/adjvar/adjvar.h
    ${PATH_TO_MODULES}/calib/calib.h
    ${PATH_TO_MODULES}/eeprom/eeprom.h
    ${PATH_TO_MODULES}/gimbalControl/gimbalControl.h
    ${PATH_TO_MODULES}/imu/imu.h
    ${PATH_TO_MODULES}/profiles/profiles.h
    ${PATH_TO_MODULES}/realtime/realtime.h
    ${PATH_TO_MODULES}/service/service.h

    ${PATH_TO_OS_GLUE}/glueAzureRTOS/glueAzureRTOS.h
    ${PATH_TO_OS_GLUE}/glueFreeRTOS/glueFreeRTOS.h
    ${PATH_TO_OS_GLUE}/glueLinuxOS/glueLinuxOS.h

    ${PATH_TO_SERIAL_API}/adjunct.h
    ${PATH_TO_SERIAL_API}/sbgc32.h
    examples/Linux/QuickStart/serialAPI_Config.h

)

# Create library target
add_library(${PROJECT_NAME} SHARED
    ${SOURCES} 
    ${HEADERS}
)

#add Demo executable
# check if BUILD_EXAMPLES is set
option(BUILD_EXAMPLES "Build examples" OFF)
if(BUILD_EXAMPLES)
  # include folders as they have their own CMakeLists.txt
  add_subdirectory(examples/Linux/DemoLaunch)
  add_subdirectory(examples/Linux/QuickStart)

  # target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME})
  # install(TARGETS ${PROJECT_NAME}_demo DESTINATION "${CMAKE_INSTALL_BINDIR}")

  # # add_subdirectory(examples/Linux/DemoLaunch)
endif()

# Set include directories
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

foreach(header ${HEADERS})
  get_filename_component(header_path ${header} DIRECTORY)
  target_include_directories(${PROJECT_NAME} PRIVATE ${header_path})
endforeach()

# set the version properties of the target
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})

# Install library (and header files ?)
# install(TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}Targets
#   LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
#   ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
#   RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
#   INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
# )

# Install include directories
install(FILES ${HEADERS}
  DESTINATION include/${PROJECT_NAME}
)
# install the shared library
install(TARGETS ${PROJECT_NAME} DESTINATION "${CMAKE_INSTALL_LIBDIR}" EXPORT ${PROJECT_NAME}Config)

# install the cmake project, for importing
install(EXPORT ${PROJECT_NAME}Config DESTINATION share/${PROJECT_NAME}/cmake)

# # run ldconfig after installing
install(CODE "execute_process( COMMAND ldconfig )")


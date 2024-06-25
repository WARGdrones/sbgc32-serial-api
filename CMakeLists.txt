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
set(PATH_TO_SOURCES sources)
set(PATH_TO_DRIVERS drivers/LinuxDriver)

# Add source files
set(SOURCES
  ${PATH_TO_DRIVERS}/driver_Linux.c
  ${PATH_TO_SOURCES}/adjvar/adjvar.c
  ${PATH_TO_SOURCES}/calib/calib.c
  ${PATH_TO_SOURCES}/core/core.c
  ${PATH_TO_SOURCES}/eeprom/eeprom.c
  ${PATH_TO_SOURCES}/gimbalControl/gimbalControl.c
  ${PATH_TO_SOURCES}/imu/imu.c
  ${PATH_TO_SOURCES}/profiles/profiles.c
  ${PATH_TO_SOURCES}/realtime/realtime.c
  ${PATH_TO_SOURCES}/service/service.c
)

# Add header files
set(HEADERS
    ${PATH_TO_DRIVERS}/driver_Linux.h
    ${PATH_TO_SOURCES}/adjvar/adjvar.h
    ${PATH_TO_SOURCES}/calib/calib.h
    ${PATH_TO_SOURCES}/eeprom/eeprom.h
    ${PATH_TO_SOURCES}/core/core.h
    ${PATH_TO_SOURCES}/core/adjunct.h
    ${PATH_TO_SOURCES}/gimbalControl/gimbalControl.h
    ${PATH_TO_SOURCES}/imu/imu.h
    ${PATH_TO_SOURCES}/profiles/profiles.h
    ${PATH_TO_SOURCES}/realtime/realtime.h
    ${PATH_TO_SOURCES}/service/service.h
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
  include_directories(
    ${PATH_TO_SOURCES}/adjvar
    ${PATH_TO_SOURCES}/calib
    ${PATH_TO_SOURCES}/core
    ${PATH_TO_SOURCES}/eeprom
    ${PATH_TO_SOURCES}/gimbalControl
    ${PATH_TO_SOURCES}/imu
    ${PATH_TO_SOURCES}/profiles
    ${PATH_TO_SOURCES}/realtime
    ${PATH_TO_SOURCES}/service
    ${PATH_TO_DRIVERS}
    )
    
  message(STATUS "Building examples")
  add_executable(${PROJECT_NAME}_demo
    examples/Linux/DemoLaunch/main.c
  )
  target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME})
  install(TARGETS ${PROJECT_NAME}_demo DESTINATION "${CMAKE_INSTALL_BINDIR}")

  # add_subdirectory(examples/Linux/DemoLaunch)
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


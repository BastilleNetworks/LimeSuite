########################################################################
## Support for Bastille Sensor platform
########################################################################


set(THIS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ConnectionBastilleSensor)

set(CONNECTION_BASTILLE_SENSOR_SOURCES
        # Implements required interfaces
        ${THIS_SOURCE_DIR}/ConnectionBastilleSensor.cpp
        ${THIS_SOURCE_DIR}/ConnectionBastilleSensorEntry.cpp

        # Helper routines
        ${THIS_SOURCE_DIR}/register_access.cpp
        ${THIS_SOURCE_DIR}/init.cpp
)

# Ensure warnings are enabled and are treated as errors
set_source_files_properties(
    ${CONNECTION_BASTILLE_SENSOR_SOURCES}
    PROPERTIES COMPILE_FLAGS
        "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -Werror"
)

########################################################################
## Feature registration
########################################################################
include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_BASTILLE_SENSOR "Enable Bastille Sensor platform support" ON "ENABLE_LIBRARY" OFF)
add_feature_info(ConnectionBastilleSensor ENABLE_BASTILLE_SENSOR "Bastille Sensor platform support")
if (NOT ENABLE_BASTILLE_SENSOR)
    return()
endif()

########################################################################
## Add to library
########################################################################
target_sources(LimeSuite PUBLIC ${CONNECTION_BASTILLE_SENSOR_SOURCES})

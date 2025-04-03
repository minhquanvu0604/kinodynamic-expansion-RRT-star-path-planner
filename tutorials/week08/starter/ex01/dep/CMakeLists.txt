execute_process(COMMAND lsb_release -cs
    OUTPUT_VARIABLE RELEASE_CODENAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if("${RELEASE_CODENAME}" STREQUAL "bionic")
    message ( STATUS "Ubuntu is in use, LTS version ${RELEASE_CODENAME}" )
elseif("${RELEASE_CODENAME}" STREQUAL "focal")
    message ( STATUS "Ubuntu is in use, LTS version ${RELEASE_CODENAME}" )
else("${RELEASE_CODENAME}" STREQUAL "bionic")
    message ( FATAL_ERROR "Your vesrion on Ubuntu ${RELEASE_CODENAME} is not supported" )
endif("${RELEASE_CODENAME}" STREQUAL "bionic")

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
    set (RELEASE_CODENAME "arm64")
endif()

# Find the library and files belonging to Radar (they have been provided in the dep subdirectory)
set(EXAMPLE_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${EXAMPLE_LIB_HOME}/include)
link_directories(${EXAMPLE_LIB_HOME}/lib/${RELEASE_CODENAME})

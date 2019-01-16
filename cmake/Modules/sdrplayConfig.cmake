INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_SDRPLAY sdrplay)

FIND_PATH(
    SDRPLAY_INCLUDE_DIRS
    NAMES sdrplay/api.h
    HINTS $ENV{SDRPLAY_DIR}/include
        ${PC_SDRPLAY_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    SDRPLAY_LIBRARIES
    NAMES gnuradio-sdrplay
    HINTS $ENV{SDRPLAY_DIR}/lib
        ${PC_SDRPLAY_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(SDRPLAY DEFAULT_MSG SDRPLAY_LIBRARIES SDRPLAY_INCLUDE_DIRS)
MARK_AS_ADVANCED(SDRPLAY_LIBRARIES SDRPLAY_INCLUDE_DIRS)


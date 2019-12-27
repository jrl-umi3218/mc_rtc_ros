# Adapted from https://stackoverflow.com/questions/58844585/use-qwt-installed-via-brew-in-cmake

set(QWT_PATHS
    /usr
    /usr/local/
    /usr/local/share/
    /usr/share/
    "${CMAKE_INSTALL_PREFIX}"
   )

find_path(Qwt_INCLUDE_DIR
          NAMES qwt.h
          PATHS ${QWT_PATHS}
          HINTS ${QWT_HINTS}
          PATH_SUFFIXES include qwt-qt4 qwt-qt5 qwt Headers
          DOC "Variable storing the location of Qwt header"
         )

set(ARCH_SUFFIX "lib")
if("${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin")
  set(ARCH_SUFFIX "")
endif()

find_library(Qwt_LIBRARY
             NAMES qwt qwt-qt4 qwt-qt5
             PATHS ${QWT_PATHS}
             HINTS ${QWT_HINTS}
             PATH_SUFFIXES ${ARCH_SUFFIX}
             DOC "Variable storing the location of Qwt library"
            )

set(Qwt_VERSION ${Qwt_FIND_VERSION})
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qwt
                                  FOUND_VAR Qwt_FOUND
                                  REQUIRED_VARS
                                  Qwt_LIBRARY
                                  Qwt_INCLUDE_DIR
                                  VERSION_VAR Qwt_VERSION
                                 )

if(Qwt_FOUND)
  set(Qwt_LIBRARIES ${Qwt_LIBRARY})
  set(Qwt_INCLUDE_DIRS ${Qwt_INCLUDE_DIR})
else()
  message(FATAL_ERROR "Could not find Qwt on your system")
endif()
if(Qwt_FOUND AND NOT TARGET Qwt::Qwt)
    if("${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin")
        get_filename_component(FRAMEWORK_LOC ${Qwt_LIBRARY} DIRECTORY)
        add_library(Qwt::Qwt INTERFACE IMPORTED)
        set_target_properties(Qwt::Qwt PROPERTIES
                              INTERFACE_COMPILE_OPTIONS ""
                              INTERFACE_INCLUDE_DIRECTORIES "${Qwt_INCLUDE_DIR}"
                              )

        target_link_libraries(Qwt::Qwt INTERFACE "-F${FRAMEWORK_LOC} -framework qwt")
    else()
        add_library(Qwt::Qwt UNKNOWN IMPORTED)
        set_target_properties(Qwt::Qwt PROPERTIES
                              IMPORTED_LOCATION "${Qwt_LIBRARIES}"
                              INTERFACE_COMPILE_OPTIONS ""
                              INTERFACE_INCLUDE_DIRECTORIES "${Qwt_INCLUDE_DIR}"
                              )
    endif()
endif()

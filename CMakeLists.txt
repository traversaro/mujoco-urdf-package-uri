cmake_minimum_required(VERSION 3.16)

project(mujoco-urdf-package-uri
        LANGUAGES CXX C
        VERSION 0.0.1)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
option(BUILD_TESTING "Create tests using CMake" OFF)

if(BUILD_TESTING)
    enable_testing()
endif()

# Get AddInstallRPATHSupport, InstallBasicPackageFiles, AddUninstallTarget
include(FetchContent)
FetchContent_Declare(YCM
                     GIT_REPOSITORY https://github.com/robotology/ycm
                     GIT_TAG v0.16.2)
FetchContent_GetProperties(YCM)
if(NOT YCM_POPULATED)
    message(STATUS "Fetching YCM.")
    FetchContent_Populate(YCM)
    # Add YCM modules in CMAKE_MODULE_PATH
    include(${ycm_SOURCE_DIR}/tools/UseYCMFromSource.cmake)
endif()

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

# Encourage user to specify a build type (e.g. Release, Debug, etc.), otherwise set it to Release.
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

# Find dependencies
find_package(mujoco REQUIRED)

# Add mujoco_urdf_package_uri C library
add_library(mujoco_urdf_package_uri mujoco_urdf_package_uri/mujoco_urdf_package_uri.h
                                    mujoco_urdf_package_uri/mujoco_urdf_package_uri.cc)
add_library(mujoco_urdf_package_uri::mujoco_urdf_package_uri ALIAS mujoco_urdf_package_uri)
target_include_directories(mujoco_urdf_package_uri PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")
target_link_libraries(mujoco_urdf_package_uri PRIVATE mujoco::mujoco)

install(TARGETS mujoco_urdf_package_uri
        EXPORT  ${PROJECT_NAME})


include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY AnyNewerVersion
                            VARS_PREFIX ${PROJECT_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)

# Python part
option(MUPU_ENABLE_PYTHON "Enable Python bindings" ON)

if(MUPU_ENABLE_PYTHON)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
    find_package(pybind11 REQUIRED)
    pybind11_add_module(mujoco_urdf_package_uri_python mujoco_urdf_package_uri/mujoco_urdf_package_uri_python.cc)
    target_compile_definitions(mujoco_urdf_package_uri_python PRIVATE MUPU_VERSION_INFO=${PROJECT_VERSION})
    target_link_libraries(mujoco_urdf_package_uri_python PRIVATE mujoco_urdf_package_uri::mujoco_urdf_package_uri)

    if(NOT DEFINED MUPU_PKGDIR)
      # 'distutils.sysconfig.get_python_lib' returns the absolute path of Python
      # by default a global location managed by the distro e.g. /usr/lib/python.
      #
      # pybind11 and FindPython3 set respectively PYTHON_SITE_PACKAGES/Python3_SITELIB
      # from 'distutils.sysconfig.get_python_lib'
      #
      # Those are especially annoying on Ubuntu since it has
      # some hardcoded paths in python3.x/site.py
      #
      # `sysconfig.get_path` may return paths that does not even exists.
      #
      # So below we retrieve the first site-package path from 'site.getsitepackages()'.
    
      execute_process(
        COMMAND
        ${Python3_EXECUTABLE} -c "import site; print(site.getsitepackages()[0])"
        OUTPUT_VARIABLE _PYTHON_SITE_PACKAGE OUTPUT_STRIP_TRAILING_WHITESPACE
      )
      set(MUPU_PKGDIR "${_PYTHON_SITE_PACKAGE}")
    endif()
    set(MUPU_INSTDIR "${MUPU_PKGDIR}/mujoco_urdf_package_uri")
      
    # Setup installation path
    install(TARGETS mujoco_urdf_package_uri_python DESTINATION "${MUPU_PKGDIR}")
    
    # Create the Python package in the build tree for testing purposes
    set(MUPU_BUILDDIR "${CMAKE_BINARY_DIR}/mujoco_urdf_package_uri")
    set_target_properties(
      mujoco_urdf_package_uri_python PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY "${MUPU_BUILDDIR}")
    
    # Create the __init__.py file
    file(
      GENERATE
      OUTPUT "${MUPU_BUILDDIR}/__init__.py"
      CONTENT "from mujoco_urdf_package_uri._bindings import *\n")
    
    # Install the __init__.py file
    install(
      FILES "${MUPU_BUILDDIR}/__init__.py"
      DESTINATION ${MUPU_INSTDIR})
      
    # Install pip metadata files to ensure that mujoco_urdf_package_uri installed via CMake is listed by pip list
    # See https://packaging.python.org/specifications/recording-installed-packages/
    # and https://packaging.python.org/en/latest/specifications/core-metadata/#core-metadata
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/METADATA "")
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Metadata-Version: 2.1\n")
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Name: mujoco-urdf-package-uri\n")
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Version: ${PROJECT_VERSION}\n")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/INSTALLER "cmake\n")
    install(
      FILES "${CMAKE_CURRENT_BINARY_DIR}/METADATA" "${CMAKE_CURRENT_BINARY_DIR}/INSTALLER"
      DESTINATION ${MUPU_PKGDIR}/mujoco-urdf-package-uri-${PROJECT_VERSION}.dist-info)
endif()

# Uninstall
include(AddUninstallTarget)


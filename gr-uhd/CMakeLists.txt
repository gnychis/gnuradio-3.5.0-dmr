# Copyright 2011 Free Software Foundation, Inc.
# 
# This file is part of GNU Radio
# 
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup dependencies
########################################################################
include(GrBoost)

find_package(UHD)

########################################################################
# Register component
########################################################################
include(GrComponent)
GR_REGISTER_COMPONENT("gr-uhd" ENABLE_GR_UHD
    Boost_FOUND
    UHD_FOUND
    ENABLE_GR_CORE
)

GR_SET_GLOBAL(GR_UHD_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${UHD_INCLUDE_DIRS}
)

########################################################################
# Begin conditional configuration
########################################################################
if(ENABLE_GR_UHD)

########################################################################
# Setup CPack components
########################################################################
include(GrPackage)
CPACK_SET(CPACK_COMPONENT_GROUP_UHD_DESCRIPTION "GNU Radio UHD Blocks")

CPACK_COMPONENT("uhd_runtime"
    GROUP        "UHD"
    DISPLAY_NAME "Runtime"
    DESCRIPTION  "Runtime"
    DEPENDS      "core_runtime"
)

CPACK_COMPONENT("uhd_devel"
    GROUP        "UHD"
    DISPLAY_NAME "Development"
    DESCRIPTION  "C++ headers, package config, import libraries"
    DEPENDS      "core_devel"
)

CPACK_COMPONENT("uhd_python"
    GROUP        "UHD"
    DISPLAY_NAME "Python"
    DESCRIPTION  "Python modules for runtime; GRC xml files"
    DEPENDS      "core_python;uhd_runtime"
)

CPACK_COMPONENT("uhd_swig"
    GROUP        "UHD"
    DISPLAY_NAME "SWIG"
    DESCRIPTION  "SWIG development .i files"
    DEPENDS      "core_swig;uhd_python;uhd_devel"
)

########################################################################
# Add subdirectories
########################################################################
add_subdirectory(include)
add_subdirectory(lib)
add_subdirectory(examples)
add_subdirectory(doc)
if(ENABLE_PYTHON)
    add_subdirectory(swig)
    add_subdirectory(grc)
    add_subdirectory(apps)
endif(ENABLE_PYTHON)

########################################################################
# Create Pkg Config File
########################################################################
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/gnuradio-uhd.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/gnuradio-uhd.pc
@ONLY)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/gnuradio-uhd.pc
    DESTINATION ${GR_LIBRARY_DIR}/pkgconfig
    COMPONENT "uhd_devel"
)

endif(ENABLE_GR_UHD)

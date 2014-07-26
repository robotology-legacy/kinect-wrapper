# Copyright: (C) 2012 Miguel Sarabia del Castillo
# Imperial College London
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

# - Try to find OpenNI
# Once done this will define
#  OpenNI_FOUND - System has OpenNI
#  OpenNI_INCLUDE_DIRS - The OpenNI include directories
#  OpenNI_LIBRARIES - The libraries needed to use OpenNI
#  OpenNI_DIR - Directory where OpenNI was found (can be set by user to force 
#               CMake to look in a particular directory)

find_path(NITE2_INCLUDE_DIRS NAMES NiTE.h HINTS $ENV{NITE2_INCLUDE})
find_library(NITE2_LIBRARIES NAMES NiTE2 HINTS $ENV{NITE2_REDIST64})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NITE2 DEFAULT_MSG
    NITE2_LIBRARIES NITE2_INCLUDE_DIRS)

set(NITE_FOUND ${NITE2_FOUND})

mark_as_advanced(NITE2_LIBRARIES NITE2_INCLUDE_DIRS NITE_FOUND)


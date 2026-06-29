# FindStreamline.cmake — locate the NVIDIA Streamline SDK (DLSS 4.5).
#
# Streamline is proprietary and license-gated; download it from
# https://github.com/NVIDIA-RTX/Streamline (or the NVIDIA developer site) and
# point -DSTREAMLINE_DIR=<path> at the unpacked SDK root.
#
# Sets: STREAMLINE_FOUND, STREAMLINE_INCLUDE_DIR, STREAMLINE_LIBRARIES,
#       STREAMLINE_DLLS (interposer + DLSS / DLSS-D plugins to ship next to exe)

set(STREAMLINE_FOUND FALSE)

if(NOT STREAMLINE_DIR OR NOT EXISTS "${STREAMLINE_DIR}")
  message(STATUS "Streamline: STREAMLINE_DIR not set or missing")
  return()
endif()

find_path(STREAMLINE_INCLUDE_DIR
  NAMES sl.h
  HINTS "${STREAMLINE_DIR}/include")

find_library(STREAMLINE_INTERPOSER_LIB
  NAMES sl.interposer
  HINTS "${STREAMLINE_DIR}/lib/x64" "${STREAMLINE_DIR}/lib")

if(STREAMLINE_INCLUDE_DIR AND STREAMLINE_INTERPOSER_LIB)
  set(STREAMLINE_FOUND TRUE)
  set(STREAMLINE_LIBRARIES ${STREAMLINE_INTERPOSER_LIB})

  # runtime plugins to copy next to the executable. Names follow the SDK bin/x64
  # layout; adjust if your SDK version differs.
  file(GLOB STREAMLINE_DLLS
    "${STREAMLINE_DIR}/bin/x64/sl.interposer.dll"
    "${STREAMLINE_DIR}/bin/x64/sl.common.dll"
    "${STREAMLINE_DIR}/bin/x64/sl.dlss.dll"
    "${STREAMLINE_DIR}/bin/x64/sl.dlss_d.dll"     # Ray Reconstruction
    "${STREAMLINE_DIR}/bin/x64/nvngx_dlss.dll"
    "${STREAMLINE_DIR}/bin/x64/nvngx_dlssd.dll")

  message(STATUS "Streamline: found at ${STREAMLINE_DIR}")
else()
  message(STATUS "Streamline: headers/libs not found under ${STREAMLINE_DIR}")
endif()

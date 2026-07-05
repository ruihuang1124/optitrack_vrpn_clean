find_package(Threads REQUIRED)

find_path(VRPN_INCLUDE_DIR
  NAMES vrpn_Connection.h vrpn_Tracker.h)

find_library(VRPN_LIBRARY
  NAMES vrpn)

find_library(VRPN_QUAT_LIBRARY
  NAMES quat)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VRPN
  REQUIRED_VARS VRPN_INCLUDE_DIR VRPN_LIBRARY)

if(VRPN_FOUND AND NOT TARGET VRPN::VRPN)
  set(VRPN_EXTRA_LIBS Threads::Threads)
  if(VRPN_QUAT_LIBRARY)
    list(APPEND VRPN_EXTRA_LIBS "${VRPN_QUAT_LIBRARY}")
  endif()

  add_library(VRPN::VRPN UNKNOWN IMPORTED)
  set_target_properties(VRPN::VRPN PROPERTIES
    IMPORTED_LOCATION "${VRPN_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${VRPN_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${VRPN_EXTRA_LIBS}")
endif()

mark_as_advanced(VRPN_INCLUDE_DIR VRPN_LIBRARY VRPN_QUAT_LIBRARY)

set(libCarla_INSTALL_DIR $ENV{CARLA_ROOT}/PythonAPI/carla/dependencies)
set(libCarla_INCLUDE_DIR ${libCarla_INSTALL_DIR}/include)
set(libCarla_LIB_DIR ${libCarla_INSTALL_DIR}/lib)
set(libCarla_LIBS 
    ${libCarla_LIB_DIR}/libboost_filesystem.a 
    ${libCarla_LIB_DIR}/libboost_program_options.a 
    ${libCarla_LIB_DIR}/libboost_system.a 
    ${libCarla_LIB_DIR}/libcarla_client.a 
    ${libCarla_LIB_DIR}/librpc.a 
    ${libCarla_LIB_DIR}/libDebugUtils.a 
    ${libCarla_LIB_DIR}/libDetour.a 
    ${libCarla_LIB_DIR}/libDetourCrowd.a 
    ${libCarla_LIB_DIR}/libDetourTileCache.a 
    ${libCarla_LIB_DIR}/libRecast.a
    ${libCarla_LIB_DIR}/libboost_filesystem.so
    ${libCarla_LIB_DIR}/libboost_program_options.so
    ${libCarla_LIB_DIR}/libboost_system.so
)

set(DEP_INCLUDE_DIR ${libCarla_INCLUDE_DIR}/system)
add_library(IVisionTargets INTERFACE)

find_package(iDynTree REQUIRED)

target_sources(IVisionTargets INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hde/interfaces/IVisionTargets.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces/IVisionTargets.h>)

target_include_directories(IVisionTargets INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces>)

target_link_libraries(IVisionTargets INTERFACE
                      iDynTree::idyntree-core)

install(
    FILES hde/interfaces/IVisionTargets.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces)


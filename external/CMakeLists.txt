# EXCLUDE_FROM_ALL is needed for the header-only libraries
# so their headers and cmake files are not copied to the
# output folder when "make install" is used

if (ZEN_USE_STATIC_LIBS)
    # install spdlog in case of static build
    # because CMake will otherwise complain about
    # missing export target
    set(SPDLOG_INSTALL ON CACHE BOOL "" FORCE)
endif()

add_subdirectory(expected-lite EXCLUDE_FROM_ALL)
add_subdirectory(gsl EXCLUDE_FROM_ALL)

# download binary driver libraries for windows if
# we compile for Win32 and we are allowed by the build
# options todo so
if (WIN32 AND ZEN_USE_BINARY_LIBRARIES)
    message("Binary libraries will be downloaded")

    # FetchContent is supported since CMake Version 3.11
    include(FetchContent)
    set(FETCHCONTENT_QUIET OFF)
    FetchContent_Declare(
        binary_libraries
        GIT_REPOSITORY https://bitbucket.org/lpresearch/openzenbinarylibraries.git
        GIT_TAG        v1.0.0
    )

    FetchContent_GetProperties(binary_libraries)

    if (NOT binary_libraries_POPULATED)
      FetchContent_Populate(binary_libraries)
      add_subdirectory(${binary_libraries_SOURCE_DIR} ${binary_libraries_BINARY_DIR})
    endif()
else()
    message("Binary libraries will not be downloaded")
endif()

# tells cereal to not build any tests
set(JUST_INSTALL_CEREAL ON CACHE BOOL "" FORCE)
# we don't want cereals install targets, so use
# EXCLUDE_FROM_ALL on this folder

# currently, the cereal external gives an cmake warning
# because project() call does not set version. There is a PR in the pipeline
# which will fix this in the future
# https://github.com/USCiLab/cereal/pull/659
add_subdirectory(cereal EXCLUDE_FROM_ALL)

add_subdirectory(spdlog EXCLUDE_FROM_ALL)
set_property(TARGET spdlog PROPERTY POSITION_INDEPENDENT_CODE ON)

# expose the Asio headers as library
add_library(openzen_asio INTERFACE)
target_include_directories(openzen_asio INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/asio/asio/include
    )

add_subdirectory(glfw EXCLUDE_FROM_ALL)

add_library(imgui STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_demo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_tables.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_widgets.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/imgui_impl_glfw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
)

target_link_libraries(imgui PUBLIC glfw)

add_library(implot STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/implot/implot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/implot/implot_items.cpp
)

target_include_directories(implot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/implot)

target_link_libraries(implot PUBLIC imgui)

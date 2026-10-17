cmake_minimum_required(VERSION 3.14)

project(OpenZenVisualizer VERSION 1.0.0 LANGUAGES CXX)

add_executable(OpenZenVisualizer
    main.cpp
    DataBuffer.h
    SensorThread.h
    GuiRenderer.h
    DeltaTEstimator.h
    MagEllipsoidCalib.h
    MadgwickFilter.h
    ComplementaryFilter.h
    EKFFilter.h
)

target_compile_features(OpenZenVisualizer
    PRIVATE
        cxx_std_17
)

if(MSVC)
    target_compile_options(OpenZenVisualizer PRIVATE /utf-8)
endif()

target_link_libraries(OpenZenVisualizer
    PRIVATE
        OpenZen
        imgui
        implot
        glfw
        Threads::Threads
        opengl32
)

target_include_directories(OpenZenVisualizer
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

if(ZEN_USE_STATIC_LIBS)
    target_compile_definitions(OpenZenVisualizer
        PRIVATE
            ZEN_API_STATIC
    )
else()
    add_custom_command(TARGET OpenZenVisualizer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:OpenZen>
            $<TARGET_FILE_DIR:OpenZenVisualizer>
    )
endif()

if(WIN32)
    add_custom_command(TARGET OpenZenVisualizer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:glfw>
            $<TARGET_FILE_DIR:OpenZenVisualizer>
    )
endif()
#==============================================================================
# dances CMake file
#
# Hanno Hildenbrandt 2022
#============================================================================== 
cmake_minimum_required(VERSION 3.18)

if (WIN32)
    set(Manifest ${CMAKE_SOURCE_DIR}/dances_imgui/dances_imgui.manifest)
endif()

if (DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE $ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake)
else()
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()
if (WIN32)
    set(VCPKG_TARGET_TRIPLET x64-windows)
endif()

set(GLSL_VERSION /DIMGUI_IMPL_OPENGL_LOADER_CUSTOM /DGLSL_OPENGL_MAJOR_VERSION=4 /DGLSL_OPENGL_MINOR_VERSION=5)
add_definitions(/DGLM_ENABLE_EXPERIMENTAL /DGLM_FORCE_XYZW_ONLY /DTBB_SUPPRESS_DEPRECATED_MESSAGES /DGLFW_INCLUDE_NONE ${GLSL_VERSION})
if (WIN32)
    add_definitions(/DNOMINMAX /DWIN32_LEAN_AND_MEAN /D_CRT_SECURE_NO_WARNINGS)
else()
    add_definitions(-Wno-switch)
endif()
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(/DGLSL_DEBUG_OUTPUT)
else()
    add_definitions(/DNDEBUG)
endif()
add_definitions(/DIMGUI_DISABLE_OBSOLETE_KEYIO)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})


project(dances-framework)

find_package(Git REQUIRED)
execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --remote
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                RESULT_VARIABLE GIT_SUBMOD_RESULT)
if (NOT GIT_SUBMOD_RESULT EQUAL "0")
    message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
endif()

find_package(OpenGL REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)


file(GLOB_RECURSE model_src 
    ${PROJECT_SOURCE_DIR}/model/*.cpp 
    ${PROJECT_SOURCE_DIR}/model/*.hpp
)

file(GLOB dances_src
    ${PROJECT_SOURCE_DIR}/libs/*.hpp
    ${PROJECT_SOURCE_DIR}/dances_imgui/*.cpp
    ${PROJECT_SOURCE_DIR}/dances_imgui/*.hpp
    ${PROJECT_SOURCE_DIR}/dances_imgui/*.h
    ${PROJECT_SOURCE_DIR}/libs/glsl/*.cpp
)

add_executable(dances ${dances_src} ${model_src} ${IMGUI_modules} ${Manifest})

target_include_directories(dances PRIVATE
     ${PROJECT_SOURCE_DIR}
     ${PROJECT_SOURCE_DIR}/libs
     ${PROJECT_SOURCE_DIR}/model
     ${CURRENT_SOURCE_DIR}
     ${TBB}
)
target_link_libraries(dances PUBLIC 
    ${CMAKE_DL_LIBS}
    TBB::tbb 
    OpenGL::GL 
    glm::glm 
    glad::glad 
    glfw 
    nlohmann_json::nlohmann_json 
    imgui::imgui implot::implot)

set_target_properties(dances PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/$<0:>)

cmake_minimum_required(VERSION 3.5)

# Project Name
PROJECT(class1)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Adicionando o tinyxml2.cpp à lista de fontes
set(SOURCES
    main.cpp
    tinyxml2/tinyxml2.cpp   # Aqui você inclui o tinyxml2.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

# OpenGL e GLUT (continua igual)
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})

if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

if (WIN32)

    message(STATUS "Toolkits_DIR set to: " ${TOOLKITS_FOLDER})
    set(TOOLKITS_FOLDER "" CACHE PATH "Path to Toolkits folder")

    # Verificações básicas
    if (NOT EXISTS "${TOOLKITS_FOLDER}/glut/GL/glut.h" OR NOT EXISTS "${TOOLKITS_FOLDER}/glut/glut32.lib")
        message(FATAL_ERROR ": GLUT not found")
    endif()

    if (NOT EXISTS "${TOOLKITS_FOLDER}/glew/GL/glew.h" OR NOT EXISTS "${TOOLKITS_FOLDER}/glew/glew32.lib")
        message(FATAL_ERROR ": GLEW not found")
    endif()

    if (NOT EXISTS "${TOOLKITS_FOLDER}/devil/IL/il.h" OR NOT EXISTS "${TOOLKITS_FOLDER}/devil/DevIL.lib")
        message(FATAL_ERROR ": DevIL not found")
    endif()

    # Includes
    include_directories(
        ${TOOLKITS_FOLDER}/glut
        ${TOOLKITS_FOLDER}/glew
        ${TOOLKITS_FOLDER}/devil
    )

    # Link com as libs
    target_link_libraries(${PROJECT_NAME}
        ${OPENGL_LIBRARIES}
        ${TOOLKITS_FOLDER}/glut/glut32.lib
        ${TOOLKITS_FOLDER}/glew/glew32.lib
        ${TOOLKITS_FOLDER}/devil/DevIL.lib
    )

    # Copiando DLLs
    if (EXISTS "${TOOLKITS_FOLDER}/glut/glut32.dll")
        file(COPY ${TOOLKITS_FOLDER}/glut/glut32.dll DESTINATION ${CMAKE_BINARY_DIR})
    endif()

    if (EXISTS "${TOOLKITS_FOLDER}/glew/glew32.dll")
        file(COPY ${TOOLKITS_FOLDER}/glew/glew32.dll DESTINATION ${CMAKE_BINARY_DIR})
    endif()

    if (EXISTS "${TOOLKITS_FOLDER}/devil/DevIL.dll")
        file(COPY ${TOOLKITS_FOLDER}/devil/DevIL.dll DESTINATION ${CMAKE_BINARY_DIR})
    endif()

    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})


else (WIN32) # Linux and Mac

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")

    find_package(GLUT REQUIRED)
    find_package(OpenGL REQUIRED)

    include_directories(${GLUT_INCLUDE_DIR})
    include_directories(${OpenGL_INCLUDE_DIRS})
    link_directories(${GLUT_LIBRARY_DIRS} ${OpenGL_LIBRARY_DIRS})
    add_definitions(${GLUT_DEFINITIONS})

    if (NOT APPLE)
        find_package(GLEW REQUIRED)
        find_package(DevIL REQUIRED)

        include_directories(${GLEW_INCLUDE_DIRS})
        include_directories(${IL_INCLUDE_DIR})       # For DevIL

        target_link_libraries(${PROJECT_NAME}
            ${OPENGL_LIBRARIES}
            ${GLUT_LIBRARIES}
            ${GLEW_LIBRARIES}
            ${IL_LIBRARIES}                          # Linka com DevIL
        )
    else()
        target_link_libraries(${PROJECT_NAME}
            ${OPENGL_LIBRARIES}
            ${GLUT_LIBRARIES}
        )
    endif()

    if(NOT GLUT_FOUND)
        message(FATAL_ERROR ": GLUT not found!")
    endif()
endif(WIN32)


cmake_minimum_required(VERSION 3.16)

project(client-ccv)

# Устанавливаем стандарт C++
set(CMAKE_CXX_STANDARD 17)

# Условие для Windows и Linux
if(WIN32)
    # Для Windows используем vcpkg
    set(VCPKG_ROOT "E:/Program/vcpkg")
    set(VCPKG_TRIPLET "x64-windows")

    # Подключаем заголовочные файлы и библиотеки из vcpkg
    include_directories("${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}/include")
    link_directories("${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}/lib")

    # Линковка необходимых библиотек
    target_link_libraries(client-ccv PRIVATE opencv boost_system ws2_32 lz4 zlib aom)
else()
    # Если Linux, используем find_package
    find_package(OpenCV REQUIRED)
    find_package(Boost REQUIRED COMPONENTS system)
    find_package(lz4 REQUIRED)
    find_package(zlib REQUIRED)
    find_package(aom REQUIRED)

    # Линковка библиотек для Linux
    target_link_libraries(client-ccv PRIVATE
        opencv_core
        opencv_imgcodecs
        Boost::Boost
        Boost::system
        lz4
        zlib
        aom
    )
endif()

# Указываем все исходные файлы
file(GLOB SOURCES "src/*.cpp")

# Создаем исполняемый файл
add_executable(client-ccv ${SOURCES})

# Указываем директорию для заголовочных файлов
target_include_directories(client-ccv PRIVATE ${CMAKE_SOURCE_DIR}/include)

cmake_minimum_required(VERSION 3.16)

project(server-ccv)

# Устанавливаем стандарт C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Условие для Windows
if(WIN32)
    # Устанавливаем путь к vcpkg toolchain, если не задан явно
    if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
        set(CMAKE_TOOLCHAIN_FILE "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
    endif()
endif()

# Указываем все исходные файлы
file(GLOB SOURCES "src/*.cpp")

# Создаём исполняемый файл
add_executable(server-ccv ${SOURCES})

# Установка версии Windows API для Boost.Asio (например, Windows 10 — 0x0A00)
if(WIN32)
    target_compile_definitions(server-ccv PRIVATE _WIN32_WINNT=0x0A00)
endif()

# Поиск библиотек через vcpkg
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(lz4 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(aom REQUIRED)

# Подключение зависимостей
target_link_libraries(server-ccv PRIVATE
    ${OpenCV_LIBS}
    Boost::system
    lz4::lz4
    ZLIB::ZLIB
    unofficial::aom
)

# Только для Windows — линковка с winsock
if(WIN32)
    target_link_libraries(server-ccv PRIVATE ws2_32)
endif()

# Подключаем директории с заголовочными файлами
target_include_directories(server-ccv PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

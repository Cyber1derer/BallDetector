# CMakeList.txt: проект CMake для BallDetectorV2; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)

project ("BallDetectorV2")

# Добавьте источник в исполняемый файл этого проекта.

# TODO: Добавьте тесты и целевые объекты, если это необходимо.
find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

add_executable (${PROJECT_NAME}  "BallDetectorV2.cpp" "BallDetectorV2.h")

target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json  ${OpenCV_LIBS})




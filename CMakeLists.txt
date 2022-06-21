# CMakeList.txt: проект CMake для BallDetectorV2; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)

project ("BallDetectorV2")

# Добавьте источник в исполняемый файл этого проекта.
add_executable (BallDetectorV2 "BallDetectorV2.cpp" "BallDetectorV2.h")

# TODO: Добавьте тесты и целевые объекты, если это необходимо.
find_package(OpenCV REQUIRED)
target_link_libraries(BallDetectorV2  ${OpenCV_LIBS})

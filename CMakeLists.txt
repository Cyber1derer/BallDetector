cmake_minimum_required (VERSION 3.8)

project ("SphereDetector")

# Добавьте источник в исполняемый файл этого проекта.

# TODO: Добавьте тесты и целевые объекты, если это необходимо.
find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

add_executable (${PROJECT_NAME}   "BallDetector.h" "BallDetector.cpp" "ColorFilter.cpp" "ColorFilter.h")

target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json  ${OpenCV_LIBS})



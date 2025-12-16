cmake_minimum_required(VERSION 3.8)

project(hybrid_robot_description) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Installation des fichiers nécessaires à l'exécution
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
  
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)


# Installation du fichier de configuration du contrôleur
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Ajout du dossier scripts
install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)
# Ajout du dossier map
install(
  DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf worlds map rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


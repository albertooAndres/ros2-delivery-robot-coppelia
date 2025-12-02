###############################################################################
# 1. Versión mínima de CMake y opciones de compilación
###############################################################################
cmake_minimum_required(VERSION 3.8)
project(proyecto_final)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################################################################
# 2. Dependencias de ROS 2 y mensajes
###############################################################################
find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(sensor_msgs      REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(std_msgs         REQUIRED)        # ← nuevo, lo usa el nodo IR

###############################################################################
# 3. Compilación de los nodos
###############################################################################
# ── Nodo LIDAR: frena al detectar obstáculo ────────────────────────────────
add_executable(lidar_bridge_node
  src/lidar_bridge_node.cpp
)
ament_target_dependencies(lidar_bridge_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# ── Nodo IR line follower: sigue la línea negra ────────────────────────────
add_executable(ir_line_follower_node
  src/ir_line_follower_node.cpp
)
ament_target_dependencies(ir_line_follower_node
  rclcpp
  std_msgs
  geometry_msgs
)

# ── Nodo traffic Light: sigue la línea negra ───────────────────────────────
add_executable(traffic_light_node src/traffic_light_node.cpp)
ament_target_dependencies(traffic_light_node
  rclcpp
  std_msgs
)

###############################################################################
# 4. Instalación de ejecutables y recursos
###############################################################################
install(TARGETS
  lidar_bridge_node
  ir_line_follower_node
  traffic_light_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param scenes rviz urdf models
        DESTINATION share/${PROJECT_NAME})

###############################################################################
# 5. Exportaciones
###############################################################################
ament_export_include_directories(include)
ament_package()


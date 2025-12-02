# 🤖 Robot de Reparto Autónomo en CoppeliaSim con ROS 2

> **Implementación de un sistema de navegación autónoma para un TurtleBot3 Burger capaz de realizar seguimiento de línea, evasión de obstáculos y entrega de paquetes en un entorno urbano simulado.**

![ROS 2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)
![CoppeliaSim](https://img.shields.io/badge/Simulator-CoppeliaSim-orange)
![Language](https://img.shields.io/badge/Language-C%2B%2B%20%7C%20Lua-blue)

## 📋 Descripción del Proyecto
[cite_start]Este proyecto desarrolla una simulación en **CoppeliaSim** utilizando **ROS 2**, donde un robot móvil TurtleBot3 Burger realiza tareas de entrega autónoma de paquetes[cite: 27]. [cite_start]El sistema combina navegación reactiva mediante seguimiento de líneas con seguridad activa mediante detección de obstáculos[cite: 27, 28].

[cite_start]El entorno recrea una pequeña ciudad con tráfico de peatones, pasos de cebra y puntos de entrega, permitiendo evaluar la interacción del robot con elementos dinámicos[cite: 54, 55].

---

## 🚀 Funcionalidades Principales

* [cite_start]**Seguimiento de Línea (Path Following):** El robot navega autónomamente siguiendo una línea negra trazada en el suelo mediante un sensor infrarrojo simulado[cite: 36, 101].
* [cite_start]**Evasión de Obstáculos:** Utiliza un sensor LIDAR para detectar objetos estáticos o dinámicos (peatones) y detiene el robot si se incumple la distancia de seguridad[cite: 41].
* **Entrega de Paquetes:** Transporta un paquete desde el inicio hasta un destinatario ("Bill Standing"). [cite_start]Al llegar, simula la entrega y confirma la finalización del recorrido[cite: 38, 39].
* [cite_start]**Interacción con Peatones:** Detecta peatones en los pasos de cebra y cede el paso[cite: 44].
* [cite_start]**Semáforo Interactivo:** Incluye un nodo de control de tráfico que regula manualmente si los peatones pueden cruzar o no, afectando la visibilidad de los mismos para el sensor LIDAR[cite: 47, 563].

---

## 🛠️ Hardware Simulado y Sensores

El robot utilizado es un **TurtleBot3 Burger** equipado con:

1.  **Sensor LIDAR (LDS-01):**
    * [cite_start]Realiza un escaneo 2D de 360 grados con un alcance máximo de 8 metros[cite: 193, 197].
    * [cite_start]Publica en el tópico `/scan` para la detección de colisiones[cite: 241].

2.  **Sensor de Línea (TCRT5000 Simulado):**
    * [cite_start]Implementado mediante un sensor de visión ortogonal de 1x1 píxeles[cite: 283, 285].
    * [cite_start]Detecta la luminancia del suelo para diferenciar la línea negra del asfalto gris[cite: 280, 281].

---

## 🧩 Arquitectura de Nodos ROS 2

El sistema se compone de tres nodos principales desarrollados en C++:

### 1. `ir_line_follower_node`
Controla el movimiento del robot basándose en la visión.
* [cite_start]**Suscripciones:** Recibe datos de `/ir_line` (luminancia) y `/safety_stop` (señal de emergencia)[cite: 356, 358].
* **Algoritmo:** Aplica un control proporcional donde la velocidad angular ($\omega$) se calcula como:  
    $\omega = -K_{LINE} \cdot (lineColour - grey)$[cite: 401, 402].

### 2. `lidar_bridge_node`
Actúa como puente entre CoppeliaSim y ROS 2 y gestiona la seguridad.
* **Función:** Procesa la nube de puntos del LIDAR. [cite_start]Si detecta un objeto a menos de **0.17m** (distancia de seguridad), publica una orden de parada en `/cmd_vel` y activa el flag `/safety_stop`[cite: 551, 554].

### 3. `traffic_light_node`
Simula un semáforo para controlar el flujo de peatones.
* **Control:** Permite cambiar el estado (Verde/Rojo) manualmente. [cite_start]Si está en rojo, los peatones desaparecen de la simulación para evitar lecturas en el LIDAR[cite: 563, 603].

---

## 📦 Instalación y Ejecución

### Requisitos
* ROS 2 (Humble/Foxy)
* CoppeliaSim Edu

### Ejecución
[cite_start]El proyecto cuenta con un archivo `launch.py` que inicia la simulación, carga la escena `.ttt` y levanta todos los nodos necesarios automáticamente[cite: 656, 660].

```bash
ros2 launch <nombre_paquete> launch.py
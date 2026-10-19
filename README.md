# Robot-Pogyo_Lorona-WS2_description

Paquete de descripcion (URDF/xacro) del robot diferencial **Pogyo Lorona WS2** para
ROS 2 Jazzy + Gazebo Sim (gz-sim 8).

## Contenido

- `urdf/` modelo del robot (xacro, materiales, extension Gazebo y ros2control).
- `meshes/` mallas STL del chasis, ruedas, caster, lidar y camara.
- `launch/` `display.launch.py` (RViz) y `gazebo.launch.py` (Gazebo Sim).
- `config/` configuracion de RViz y del puente `ros_gz_bridge`.
- `scripts/` `trayectoria_grupo5.py`: dibuja el numero **5** con el robot.

## Requisitos

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
  python3-colcon-common-extensions
```

## Compilacion

```bash
mkdir -p ~/ros2_ws/src
# copiar/clonar este paquete dentro de ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Ejecucion

Visualizar el robot y los colores en RViz:

```bash
ros2 launch Robot-Pogyo_Lorona-WS2_description display.launch.py
```

Simulacion en Gazebo (agrega el robot y puentea `/cmd_vel` y `/odom`):

```bash
ros2 launch Robot-Pogyo_Lorona-WS2_description gazebo.launch.py
```

En otra terminal, ejecutar la trayectoria del grupo 5 (numero 5):

```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/Robot-Pogyo_Lorona-WS2_description/scripts/trayectoria_grupo5.py
```

## Trayectoria del numero 5

El script controla el robot en lazo cerrado leyendo `/odom` y publicando `/cmd_vel`.
Secuencia de segmentos (largo de barra `0.60 m`, alto `0.30 m`):

1. Avanzar 0.60 m (barra inferior).
2. Girar +90 deg.
3. Avanzar 0.30 m (vertical inferior derecha).
4. Girar +90 deg.
5. Avanzar 0.60 m (barra central).
6. Girar -90 deg.
7. Avanzar 0.30 m (vertical superior izquierda).
8. Girar -90 deg.
9. Avanzar 0.60 m (barra superior).

## Parametros de traccion

En `urdf/Robot-Pogyo_Lorona-WS2.gazebo` el plugin `gz-sim-diff-drive-system` usa:

- Ruedas: `Left_Joint` / `Right_Joint`
- `wheel_separation`: 0.16 m
- `wheel_radius`: 0.0375 m
- Topicos: `/cmd_vel` y `/odom`

## Solucion de problemas

### El robot aparece pero no se mueve

Suele ser que Gazebo arranco **en pausa** (estado guardado por la GUI). Comprueba con:

```bash
ros2 topic hz /clock
```

Si no publica nada, la simulacion esta en pausa. Soluciones:

- Pulsa el boton **Play (▶)** en la ventana de Gazebo.
- O revisa `~/.gz/sim/8/gui.config` y deja:

```xml
<play_pause>false</play_pause>
<start_paused>false</start_paused>
```

- Confirma que `/cmd_vel` tiene un suscriptor y `/odom` trafico:

```bash
ros2 topic info /cmd_vel
ros2 topic hz /odom
```

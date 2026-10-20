# Robot-Pogyo_Lorona-WS2_description


Paquete de descripcion (URDF/xacro) del robot diferencial **Pogyo Lorona WS2** para
ROS 2 Jazzy + Gazebo Sim (gz-sim 8).

## Contenido

- `urdf/` modelo del robot (xacro, materiales, extension Gazebo y ros2control).
- `meshes/` mallas STL del chasis, ruedas, caster, lidar y camara.
- `launch/` `display.launch.py` (RViz) y `gazebo.launch.py` (Gazebo Sim).
- `config/` configuracion de RViz y del puente `ros_gz_bridge`.
- `worlds/` `empty_fast.sdf`: mundo de Gazebo con paso de 4 ms para una simulacion fluida.
- `scripts/` `trayectoria_grupo5.py` (control por pose real) y
  `plot_odom_grupo5.py` (grafica en vivo y PNG de la trayectoria).

## Requisitos

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  python3-colcon-common-extensions python3-matplotlib
```

La grafica usa matplotlib con backend Tk; necesita una sesion grafica (`DISPLAY`).

El launch de Gazebo usa el plugin `gz-sim-diff-drive-system`; no requiere
`ros2_control` ni `gz_ros2_control`.

## Compilacion

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/rlorona/WS2_RL_JP_G5.git \
  Robot-Pogyo_Lorona-WS2_description
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Ejecucion

Visualizar el robot y los colores en RViz:

```bash
ros2 launch Robot-Pogyo_Lorona-WS2_description display.launch.py
```

Simulacion en Gazebo (agrega el robot y puentea `/clock`, `/cmd_vel` y `/odom`):

```bash
ros2 launch Robot-Pogyo_Lorona-WS2_description gazebo.launch.py
```

En otra terminal, ejecutar la trayectoria del grupo 5 (numero 5):

```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/Robot-Pogyo_Lorona-WS2_description/scripts/trayectoria_grupo5.py
```

## Trayectoria del numero 5

El script controla el robot en lazo cerrado leyendo la **pose real** de Gazebo
(`/world/empty/pose/info` via `gz.transport`), no `/odom`, y publicando `/cmd_vel`.
Va a **puntos exactos (waypoints)** y en cada tramo corrige el rumbo y el desvio
lateral. Tambien publica la pose real en `/pose_ground_truth` para la grafica.

La pose de dibujo es el **punto medio del eje de las ruedas motrices** (centro
instantaneo de giro), no el origen de `base_link`, que esta 8 cm adelante. Asi
los giros en el lugar no desplazan el trazo y el 5 sale recto. La grafica muestra
esa pose real; `/odom` es solo la estimacion del plugin.
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

## Grafica de la trayectoria (matplotlib)

En otra terminal, **antes** de ejecutar la trayectoria:

```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/Robot-Pogyo_Lorona-WS2_description/scripts/plot_odom_grupo5.py
```

Se abre una ventana con el recorrido **real** (pose fisica de Gazebo, punto de
dibujo en el eje de las ruedas) leido de `/pose_ground_truth` y se guarda la
imagen `~/Desktop/WS2_grupo5_trayectoria.png`.
Conviene esperar a que aparezca la ventana ("Ventana lista") antes de lanzar
`trayectoria_grupo5.py`; al cerrar la ventana se guarda el PNG. La imagen de la
entrega queda tambien en `docs/WS2_grupo5_trayectoria.png`.

## Video de la simulacion

El video del robot dibujando el 5 en Gazebo esta en Google Drive:

https://drive.google.com/file/d/1hXXQgFZEFct7ZmKo_lDTNc5uEydO2b_d/view?usp=sharing

El enlace esta compartido como "cualquiera con el enlace puede ver".

## Parametros de traccion

En `urdf/Robot-Pogyo_Lorona-WS2.gazebo` el plugin `gz-sim-diff-drive-system` usa:

- Ruedas: `Left_Joint` / `Right_Joint`
- `wheel_separation`: 0.16 m
- `wheel_radius`: 0.0375 m
- Topicos: `/cmd_vel` y `/odom`
- Caster: esfera sobre dos articulaciones pasivas (swivel Z + rueda Y); friccion 0.2.
- Ruedas motrices: friccion 2.0.

## Solucion de problemas

### Las ruedas giran pero el robot no avanza

Si `/odom` avanza pero la pose real del robot en Gazebo no cambia, las ruedas estan
patinando o no tocan el piso. En este paquete se corrigio asi:

- Las ruedas usan colision **cilindrica** (no malla STL) y su centro esta 1 mm por
  debajo del piso nominal, para garantizar contacto con el suelo.
- El chasis usa una caja y el caster una esfera sobre una articulacion swivel;
  lidar y camara no tienen colision.
- `gazebo.launch.py` carga `worlds/empty_fast.sdf` (paso de 4 ms).

La pose fisica real se puede comprobar con:

```bash
gz topic -e -t /world/empty/pose/info -n 1 | grep -A6 'Robot-Pogyo'
```

Si el robot se ve muy lento, revisa el factor de tiempo real con:

```bash
gz topic -e -t /world/empty/stats -n 1
```

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

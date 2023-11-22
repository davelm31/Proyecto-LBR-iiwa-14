# Proyecto-LBR-iiwa-14
En el siguiente proyecto se modificó el robot manipuladore LBR iiwa 14 agregándole una articulación prismática en la parte superior convirtiéndose en un robot de 7 grados de libertad en 8 grados de libertad. Se implementa diferentes análisis para el robot LBR iiwa 14 como la cinemática directa, inversa, control cinemático y control dinámico.

### Ejecucion en terminal
Una ves descargado el repositorio generar los siguientes comandos en la terminal 
```
catkin_make
source devel/setup.bash
roslaunch iiwa_description display_iiwa.launch
```
### Cinematica directa 

![alt text]([https://github.com/davelm31/Proyecto-LBR-iiwa-14/blob/main/Cinematica%20directa.png)])

Lista de parámetros DH:

|   |      $d_1$     |    $\theta$    |     a    | $\alpha$ |
|:-:|:--------------:|:--------------:|:--------:|:--------:|
| 1 |    0.34        |  $q_0      $   |     0    |  $-\pi\2$|
| 2 |        0       |  $-q_1      $  |  0       | $\pi\2$  |
| 3 |        0.42    | $q_2        $  | 0        | $\pi\2$  |
| 4 |        0       |     $q_3$      |     0    | $-\pi\2$ |
| 5 | 0.4            |      $q_4$     |     0    | $-\pi\2$ |
| 6 |        0       |  $q_5      $   |     0    |  $\pi\2$ |
| 7 |    0.179       |  $q_6      $   | -0.3     |$-\pi\2$  |
| 8 |    0           |              0 | -$q_8$   |     0    |


### Archivos
```
.
└── root/
    └── iiwa_ros
         └── iiwa_description
            ├── launch/
            │   ├── controller.launch
            │   ├── controller.yaml
            │   ├── display.launch
            │   ├── gazebo.launch
            │   ├── urdf.rviz
            ├── meshes/
            └── src/
            │   ├── control_dinInv.py (Control dinamico operacional)
            │   ├── control_dinInv2.py (Control dinamico articular)
            │   ├── markers.py
            │   ├── proyfunctions.py (Funciones utiles)
            │   ├── test_din.py (Comprobación de modelo dinámico)
            │   ├── test_fkine.py (Cinemática directa)
            │   ├── test_ikine.py (Cinemática inversa)
            └── urdf/
```

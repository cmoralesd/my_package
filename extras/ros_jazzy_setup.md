# Instalando ROS2 Jazzy en Ubuntu 24.04

Revisa un video con las instrucciones completas de este tutorial aquí: https://youtu.be/HJx_Jre8QlM   
Si prefieres utilizar ROS2 Jazzy en un contenedor Docker, revisa aquí: (en desarrollo...) 

## Preparativos
La instalación de ROS2 Jazzy requiere de Ubuntu 24.04 LTS, por lo cual comenzaremos con esta instalación. Existen 4 opciones comunmente utilizadas para esto:

### 1. Instalar Ubuntu 24.04 LTS Desktop en forma nativa en el computador (recomendado para máximo rendimiento)
La instalación de Ubuntu puede compartir el disco duro con Windows, mediante *dual boot*.   Con este método, al encender el computador es posible elegir si se trabajará con Ubuntu o con Windows.      
Esta es la configuración que aprovecha de mejor manera los recursos del computador, especialmente la tarjeta gráfica, y es la recomendada para desarrollo.
Este enlace describe en detalle cómo realizar el procedimiento de instalación: https://www.softzone.es/windows/como-se-hace/ubuntu-windows-dual-boot/

### 2. Instalar Ubuntu 24.04 en una máquina virtual (recomendado para el periodo de aprendizaje)
Una máquina virtual es un entorno seguro para familiarizarse con Linux y ROS2 sin tener que hacer cambios en el computador.   
Dependiendo de los recursos (memoria RAM y número de procesadores que pueden destinarse a la máquina virtual), el rendimiento puede ser suficientemente bueno, incluso para simulaciones con GazeboSim.   
Para descargar VMWare: https://www.techspot.com/downloads/189-vmware-workstation-for-windows.html    
Para descargar Ubuntu: https://ubuntu.com/download/desktop   

### 3. En Windows, utilizando WSL (Windows Subsystem for Linux)
Este método facilita la utilización de software de Linux en un entorno Windows. Su desempeño es bastante bueno, excepto para la utilización de simulaciones en GazeboSim. Es una configuración adecuada para las tareas de control y monitoreo utilizando ROS2, pero para simulaciones fluidas puede ser necesario utilizar un software externo, como Webots, en lugar de utilizar GazeboSim.   

### 4. En un contenedor Docker
Este método permite ejecutar ROS2 Jazzy en equipos con un sistema Linux distinto de Ubuntu 24.04. Es el método que deberá utilizrse para trabajar con ROS2 Jazzy en computadoras de placa reducida, tales como Raspberry Pi o Jetson Nano. Las indicaciones para trabajar en Docker no están contenidas en este tutorial.


## 1. Instalar ROS2 Jazzy
El detalle de la instalación puede encontrarse en el tutorial oficial: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html   
*El procedimiento siguiente es idéntico para instalar ROS en una instalación nativa de Ubuntu 24.04, en una máquina virtual o por medio de WSL*.   
1. Añadir el componente 'universe' a todos los repositorios 
`sudo add-apt-repository universe`  

2. Agregar la llave GPG de ROS2  
`sudo apt update && sudo apt install curl -y`  
`sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`

3. Agregar el repositorio al listado de fuentes de software  
`echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`

4. Instalar herramientas de desarrollo  
`sudo apt update && sudo apt install ros-dev-tools`

5. Instalar ROS  
`sudo apt upgrade`  
`sudo apt install ros-jazzy-desktop`  

6. Habilitar ros-jazzy como fuente de software en todos los terminales, agregando la siguiente instrucción al archivo *~/.bashrc*:   
`source /opt/ros/jazzy/setup.bash`  

## 2. Otras instalaciones requeridas
Utilizar los siguientes comandos para instalar paquetes adicionales, requeridos para que todo funcione.  
    `sudo apt install mesa-utils`  
    `sudo apt install python3-pip`  
    `sudo apt install jstest-gtk`   
    `sudo apt install ros-jazzy-joy*`  
    `sudo apt install ros-jazzy-joint-state-publisher`  
    `sudo apt install ros-jazzy-joint-state-publisher-gui`  
    `sudo apt install ros-jazzy-ros2-control`  
    `sudo apt install ros-jazzy-ros2-controllers`  
    `sudo apt install ros-jazzy-ros-gz`  
    `sudo apt install ros-jazzy-gz-ros2-control `  
    `sudo apt install ros-jazzy-navigation2`  
    `sudo apt install ros-jazzy-nav2-bringup`  

## 3. Configurar video para RVIZ y GazeboSim
La instalación de ROS2 Jazzy en Ubuntu 24.04 puede arrojar problemas con los componentes que requieren interfaz gráfica y renderización 3D, tales como RVIS2 y GazeboSim.    Si las opciones de video por defecto en Ubuntu 24.04 no permiten la ejecución de RVIZ2 y/o GazeboSim, realizar las siguientes configuraciones:  
### Cambiar la interfaz gráfica *wayland* por *X11*  
(Siguiendo la solución propuesta en https://robotics.stackexchange.com/questions/111436/rviz2-is-not-working-in-ros2-jazzy)  
1. Verificar el sistema de gestión de ventanas gráficas, ejecutando:   
    `echo $XDG_SESSION_TYPE`   
   Si la salida es *wayland*, entonces instalar X11 con:   
     `sudo apt-get install xorg openbox`
3. Editar el archivo *custom.conf*, utilizando *nano*:   
     `sudo nano /etc/gdm3/custom.conf`   
   Habilitar la línea "WaylandEnable=false", quitando el caracter de comentario '#'.   
   Guardar con `<control + o>` y salir con `<control + x>`.
4. Reiniciar

### Instalar la última versión de MESA driver
(Siguiendo la solución propuesta en https://itsfoss.com/install-mesa-ubuntu/)
1. Agregar el repositorio de MESA driver a las fuentes de software:   
   `sudo add-apt-repository ppa:kisak/kisak-mesa`
2. Actualizar la lista de repositorios y realizar una actualización de software   
   `sudo apt update`  
   `sudo apt upgrade`  
3. Ejecutar `glxinfo | grep OpenGL` y verificar que no hay errores.

### Deshabilitar LibGL_DRI3
(Siguiendo la solución propuesta en https://gazebosim.org/docs/garden/troubleshooting#ubuntu)  
Agregar la siguiente instrucción al final del archivo *~/.bashrc*    
        `export LIBGL_DRI3_DISABLE=1`

## 4. Instalar Visual Studio Code
1. Descargar archivo .deb desde https://code.visualstudio.com/download  
2. Abrir terminal en carpeta descargas y ejecutar   
       `sudo apt install ./code_<version>.deb`
4. Una vez abierto VSCode, instalar extensiones: *Python* (by Microsoft), *ROS* (by Microsoft)

## 5. Otras configuraciones útiles
Al compilar los repositorios de ROS2 utilizando `colcon build --symlink-install`, aparece una molesta advertencia de librerías obsoletas (deprecation warning). Este es sólo un mensaje de alerta (warning) y no afecta la funcionalidad. Sin embargo, si se desea dejar de visualizar este mensaje, incluir las siguientes líneas al final del archivo *~/.bashrc*:   
    `PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::setuptools.command.develop`   
    `export PYTHONWARNINGS`
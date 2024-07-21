# Primeros pasos con ROS2 - Tu primer paquete con Ament Python
Con las instrucciones de este tutorial crearemos un primer paquete de ROS2, utilizando Ament Python.    
Antes de seguir, asegúrate que tienes listo tu setup con Ubuntu 24.04 y Ros2 jazzy. Instrucciones detalladas aquí:  
El procedimiento de este tutorial está basado en el tutorial oficial (https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) y en las buenas prácticas propuestas por Murilo (https://ros2-tutorial.readthedocs.io/en/latest/index.html)   
Para una explicación detallada de todo el proceso seguido en este tutorial, revisa este video:  
## 1. Crea el espacio de trabajo
Abre un terminal nuevo, ubícate en la carpeta de usuario `~$` y crea una nueva carpeta para contener el espacio de trabajo (workspace):   
      `mkdir my_ws`   
Entra a la carpeta recién creada:   
      `cd my_ws`   
Crea allí una subcarpeta *src* que alojará el código fuente (source code):   
      `mkdir src`   
Compila el espacio de trabajo utilizando *colcon*:   
      `colcon build`  

## 2. Crea un paquete utilizando Ament Python
Un paquete de software es una estructura organizada de archivos de código. Cuando se desea instalar o compartir software en ROS2, éste debe estar organizado en un paquete.
ROS2 contiene instrucciones que permiten crear paquetes rápidamente utilizando plantillas.   
Todos los paquetes se deben crear en el espacio de trabajo, dentro de la carpeta *src*. Abre un terminal en esa ubicación:      
      `cd ~/my_ws/src`   
Crea tu primer paquete para alojar código escrito en lenguaje Python. El nombre del paquete será *my_package* y contendrá un nodo llamado *my_node*. Esta es la instrucción para crear el paquete:   
      `ros2 pkg create my_package --build-type ament_python --node-name my_node`   
Una vez creado el paquete, debemos compilar. Para ello, vuelve al espacio de trabajo:   
      `cd ..`  
Y compila:
      `colcon build`  
Para ejecutar el código contenido en el paquete recién creado, primero es necesario agregar el espacio de trabajo como fuente de software.   
Desde un terminal, accede a la carpeta del espacio de trabajo, `~/my_ws$`, ejecuta:   
      `source install/setup.bash`   
Ahora es posible ejecutar el código contenido en el nodo *my_node*:   
      `ros2 run my_package my_node`   
El terminal mostrará un mensaje de saludo: *"Hi from my_package"*.   

## 3. Utiliza VSCode para editar el paquete
VSCode es una excelente herramienta para trabajar en la edición y deputación de código. Abriremos una ventana de VSCode directamente desde la carpeta *src* en el espacio de trabajo:   
      `cd ~/my_ws/src`   
      `code .`   
Por medio de VSCode, revisa la estructura de archivos que componen el paquete. Editaremos algunos de ellos.  
### Edita el archivo package.xml
Todo paquete de ROS contiene un archivo *package.xml*. Aquí se detalla información sobre el paquete y su desarrollador, para ser compartida con otros usuarios.   
Como una buena práctica, en este archivo editaremos la información de:
- versión
- descripción
- mantenedor
- licencia
Además, este archivo contiene la declaración de dependencias de nuestro paquete. Es importante declarar aquí *todas las dependencias requeridas para el funcionamiento de los nodos*.   
Si queremos recibir o transmitir datos por medio de ROS, al menos deberemos declarar la dependencia a la librería *rclpy*.  
Agregaremos las dependencias entre las etiquetas `<license>` y `<test_depend>`.   
      `<depend>rclpy</depend>`   
### Edita el archivo setup.py
Todo paquete de ROS creado con Ament Python contiene un archivo *setup.py*. Aquí se declaran los archivos que van a ser compartidos con otros nodos (data files) y las instrucciones de entrada (entry points) que permitirán la ejecución de los nodos desde el terminal.   
Por ahora, en este archivo sólo actualizaremos la información del paquete, para hacerla consistente con package.xml:   
### Edita el archivo my_node.py
El archivo my_node.py es el archivo ejecutable de este paquete. Realmente, aun no es un nodo de ROS2 y su tarea es simplemente enviar un mensaje de saludo por el terminal. Editaremos este archivo para convertirlo en un nodo de ROS2.    
Ten presente que, al editar cualquier archivo, los cambios no se verán reflejados en el ejecutable, sino hasta que el paquete sea nuevamente compilado. Sin embargo, utilizando la opción `--symlink_install`, es posible editar los archivos y hacer que los cambios sean inmediatemente ejecutables al momento de guardar, sin necesidad de volver a compilar.   
      `cd ~/my_ws/`   
      `colcon build --symlink_install`   
Tras realizar los cambios, para iniciar nuevamente el nodo se debe ejecutar:   
      `ros2 run my_package my_node`   
En caso que el nodo no se inicie correctamente:
1. Asegúrate que has agregado el espacio de trabajo como fuente de software: `source install/setup.bash`
2. Elimina las carpetas *build*, *install* y *log* desde el espacio de trabajo. Vuelve a compilar y, nuevamente, ejecuta `source install/setup.bash`     
------------------------------------------------------------   
#### NOTA: setuptools deprecation warning
El uso de la opción *--simlink-install* activa una advertencia que el módulo *setuptools* quedará obsoleto en las próximas versiones. Si desea dejar de visualizar esta notificación, agregar las siguientes instrucciones al archivo *~/.bashrc*:   
    `PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::setuptools.command.develop`   
    `export PYTHONWARNINGS`   
------------------------------------------------------------   

Seguiremos editando *my_node* para hacer que muestre el mensaje en forma repetitiva. Para ello, le daremos la estructura típica de un nodo de Python. Esta estructura nos servirá como plantilla para los futuros nodos que vamos a crear.   
```
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        timer_period = 0.5 #sec
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        print('hello from ROS2!')

def main(args=None):
    try: 
        rclpy.init(args=args)
        my_node = MyNode()
        rclpy.spin(my_node)

    except KeyboardInterrupt:
        print(' ... exit node')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
```

## 4. Crea un nodo publicador
El nodo de ejemplo *my_node* simplemente muestra un saludo en el terminal donde el nodo se pone en ejecución. Crearemos ahora otro nodo para transmitir un mensaje que podrá ser recibido por otros nodos de ROS2, sea que éstos se alojen en el mismo equipo o en otro equipo conectado a la misma red.   
Tomando como ejemplo la estructura de *my_node*, agrega las líneas de código necesarias para incluir un publicador y un registo de eventos en el log.   
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5 #sec
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "message # %d" %self.i
        self.publisher.publish(msg)
        self.get_logger().info("Publishing: '%s" % msg.data)
        self.i += 1

def main(args=None):
    try: 
        rclpy.init(args=args)
        pub = MyPublisher()
        rclpy.spin(pub)

    except KeyboardInterrupt:
        print(' ... exit node')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
```

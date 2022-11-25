# roboclaw_node
Nodo en ROS encargado de enviar ordenes a los motores a partir de subscripciones a los topicos de los demas nodos como el control remoto, control por gestos, y un follow me

## Requerimientos de Software
Se implementó ROS Melodic para la distribución de Linux Ubuntu 18.04 para la tarjeta JETSON NANO, además el repositorio cuenta con scripts programados en python, se puede usar la versión 2 o 3 de python para ejecutarlos. 

Librerias necesarias en los scripts
* rospy
* roboclaw_driver
* std_msgs.msg
* geometry_msgs.msg

## Parametros
En caso de requerir parametros de configuración inicial esta tabla de resumen agrupa lo que se usaron en los setups.

|Parameter|Default|Definition|
|-----|----------|-------|
|dev|/dev/ttyACM0|Dev that is the Roboclaw|
|baud|115200|Baud rate the Roboclaw is configured for|
|address|128|The address the Roboclaw is set to, 128 is 0x80|

## Topicos
### Subscritos

/Udir_track [(geometry_msgs/Twist)](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
/Mode [(geometry_msgs/Twist)](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
/controlByGesture [(geometry_msgs/Twist)](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
/followMe [(geometry_msgs/Twist)](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
## Configuración del RoboClaw
Se configuró el controlador en un entrono linux, por lo tanto se debe obtener el puerto serial que usa el controlador para comunicarse con la JETSON y darle todos los permisos, Se implementa el siguiente comando
```bash
ls /dev/tty*
```
Se verifica que el ttyACM0 se encuentre habilitado
```bash
sudo chmod a+rw /dev/ttyACM0
```
seguido de esto se debe configurar los modos del RoboClaw,  para esto se ubican los tres pulsadores que tiene el RoboClaw en su parte lateral

assets/configPinesRoboClaw.png

El RoboClaw cuenta con un led testigo que va a indicar en que configuracion se encuentra cada uno de los tres modos, hara blink las veces necesarias segun el numero correspondiente a cada configuración. Para acceder al modo de configuración se debera mantener pulsado el boton correspondiente de lo que se quiera configurar, en nuetro caso se va a configurar el Packet Mode en 7, el Serial Mode en 6 , y el Baterry Option en 2. En el siguiente video se muestra como se configuró el RoboClaw que se usó para el rover.
assets/configRoboClaw.mp4

## Instalación del repositorio
```bash
git clone https://github.com/SJulian25/roboclaw_node.git
```
## Como correr el codigo
Es necesario iniciar un roscore para correr el nodo que implementamos, y ademas poder obtener los topicos de los nodos a los que nos subscribimos. Entonces en la ruta donde clonamos el repo:
```bash
roscore
```
Luego de iniciar el ROS Master abrimos una nueva terminal con `Crtl+shift+t` para ir al directorio de los scripts
```bash
cd <worksapce>/src
```
Cuando nos encontramos en el directorio /src vamos a ver dos archivos, corremos `roboclaw_node.py` para escuchar los topicos a los que fuimos subscritos.
## Como funciona
El nodo se subscribe a un topico maestro que le va a indicar cual de los demas topicos debe escuchar y así tomar decisciones con respecto a los motores
```python
        rospy.Subscriber("Mode", Int8, self.Mode)
        if self.mode == 0:
            rospy.Subscriber("Udir_track", Twist, self.manualControl)
        elif self.mode == 1:
            rospy.Subscriber("Gestos", Twist, self.controlByGestures)
        elif self.mode ==2:
            rospy.Subscriber("followMe", Twist, self.followMe)
        else:
            pass
            
        rospy.spin()
```
Cuando el nodo puede leer e interpretar el tipo de dato que se envían por los topicos, envía acciónes a los motores llamando funciones del archivo `roboclaw_driver.py`
```python
    def Mode(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.mode = data

    def manualControl(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
	x = data.linear.x
	z = data.angular.z
        self.moveMotors(x,z)

    def controlByGestures(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
	x = data.linear.x
	z = data.angular.z
        self.moveMotors(x,z)

    def followMe(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
	x = data.linear.x
	z = data.angular.z
        self.moveMotors(x,z)
```
###  Author
**[Universidad de Ibagué - Ingeniería Electrónica.](https://electronica.unibague.edu.co)**
 [Julian C. Salgado R.](https://github.com/SJulian25)

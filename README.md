<div align="center">
<h1> Instituto Tecnologico de Tijuana 
<br>Exposicion: Sensor LIDAR Y RS485 comunicación de sensores
<br>Mondaca Medina Sofia Carolina
<br>18212226 </h1>
</div>

![Sensor LiDAR](images/cooltext419209209399426.png "Sensor LiDAR giratorio")

* Proviene del acronimo en ingles de **Li**ght **d**etection **a**nd **r**anging 
    * Significa "deteccion de luz y distancias"
    * Tambien es comunmente referido como "laser scanning" o "3D scanning"
* Es una tecnologia utilizada para medir la distancia extacta de un objeto en la superficie de la Tierra
    * Estas mediciones se realizan por medio de un laser pulsado que es seguro para la vista humana
* El sensor crea una representacion 3D del espacio escaneado en cuestion
* Es utilizado primariamente en la industria automotriz, infraestructura, robotica, cartografica, entre otras.
    * Actualmente son los sensores mas vitales para los autos que se conducen sin mano humana, es decir, los autos automaticos
* Su precio varia desde los 900$ MXN a 3000$ MXN
* Existen diversos modelos para un sensor LiDAR, he aqui algunos ejemplos:

![Sensor LiDAR](images/182407-10533478.jpg)
![Sensor LiDAR](images/IM0068358.png)

<div></div>

# ¿Como funciona un sensor LiDAR?

<p>Un LiDAR consiste de un foco emisor de rayos laser infrarrojos y de un lente receptor infrarrojo capaz de ver los laser. Cuando los rayos laser impactan sobre un objeto, se reflejan** o **rebotan. Estas rayos que vuelven reflejados son detectados por el lente, y asi el procesador del sensor obtiene una nube de puntos del entorno en donde conoce la posicion precisa en el espacio y la distancia entre los puntos. Finalmente, procesa una imagen tridimensional en tiempo real.</p>

<div></div>

# ¿Como se compara un sensor LiDAR a una camara?

| **Sensor LiDAR**                                                                                 | **Sistema de camaras**                                              |
|--------------------------------------------------------------------------------------------------|---------------------------------------------------------------------|
| "Observa" en 3D a traves de mapas en 3D de alta resolucion dandole una gran ventaja en presicion | Producen imagenes 2D del ambiente a su alrededor                    |
| Produce medidas exactas                                                                          | Tienen que asumir la distancia de un objeto                         |
| Tiene su propia fuente de luz, por lo que puede "ver" en cualquier condicion luminosa            | Son dependientes de la iluminacion y otras condiciones del ambiente |


<div></div>

![RS485](images/cooltext419209274547449.png)

* Es un estandar de comunicaciones tambien conocido como **EIA/TIA-485**
* Su principal funcion es transportar una señal a traves de dos cables
    * Uno de los cables transmite la señal original
    * El otro su copia inversa
* Este metodo se conoce como "intercambio de datos bidireccional" y ofrece una gran resistencia a las interferencias en modo comun
* Sin embargo, los dispositivos RS-485 no pueden transmitir y recibir datos al mismo tiempo
    * Es necesario adoptar cierto comportamiento para evitar colision de paquetes de datos

![RS485](images/aaa.png)

<div></div>

# Caracteristicas y limitaciones del RS-485

* La longitud maxima del cable utilizado en comunicaciones RS-485 es de 1200 metros
* Es necesario que los dispositivos del sistema establezcan un formato comun para la transmision de los paquetes de datos
* Es generalmente utilizado en cables de partrenzados

<div></div>

# RS-485 vs RS-232

Los estandares RS-485 y RS-232 proporcionan soluciones para transmitir datos en largas distancias, sin embargo, las ampliaciones del RS-232 eventualmente dieron el nacimiento de RS-485, por lo que la siguiente tabla ofrece una comparacion entre los dos estandares.

| **Sensor LiDAR**                                                                                 | **Sistema de camaras**                                              |
|--------------------------------------------------------------------------------------------------|---------------------------------------------------------------------|
| "Observa" en 3D a traves de mapas en 3D de alta resolucion dandole una gran ventaja en presicion | Producen imagenes 2D del ambiente a su alrededor                    |
| Produce medidas exactas                                                                          | Tienen que asumir la distancia de un objeto                         |
| Tiene su propia fuente de luz, por lo que puede "ver" en cualquier condicion luminosa            | Son dependientes de la iluminacion y otras condiciones del ambiente |

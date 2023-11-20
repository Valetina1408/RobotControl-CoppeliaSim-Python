# Control de Robot en CoppeliaSim con Comunicación Cliente-Servidor
## Descripción de los Códigos:
### Introducción Inicial del Proyecto:
Este proyecto se centra en el desarrollo de un sistema de control para un robot en CoppeliaSim mediante una arquitectura de comunicación cliente-servidor. El servidor, implementado en el entorno de CoppeliaSim, gestiona la simulación y controla los movimientos del robot. Por otro lado, el cliente permite la interacción del usuario y el control remoto del robot.

### Funcionalidades Principales del Código del Servidor (CoppeliaSim):

__Configuración de la Conexión:__ Establece la conexión entre el servidor y el cliente mediante la biblioteca de CoppeliaSim.

__Control de Movimientos:__ Dirige los motores del robot en CoppeliaSim para realizar movimientos específicos, como traslaciones y rotaciones.

__Interfaz de Usuario:__ Proporciona una interfaz básica para interactuar con la simulación y visualizar los resultados.

### Funcionalidades Principales del Código del Cliente:

__Configuración de la Conexión:__ Establece la conexión con el servidor a través de sockets.

__Control de Cinemática Inversa:__ Implementa algoritmos de cinemática inversa para calcular los ángulos de las articulaciones del robot en función de la posición deseada.

__Interacción con el Usuario:__ Utiliza la biblioteca Pygame para permitir que el usuario envíe comandos de movimiento al robot.

__Envío de Datos:__ Transmite datos relevantes al servidor, como posiciones deseadas y ángulos de rotación.

## Requisitos y Ejecución:
Todos los archivos y bibliotecas necesarias para la ejecución se encuentran proporcionados en el código. Asegúrate de tener instaladas las bibliotecas requeridas, como Pygame y Numpy, para ejecutar el proyecto. El código ha sido desarrollado y probado en el entorno de Thonny, por lo que se recomienda su uso para una ejecución sin problemas.






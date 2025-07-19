# Sistema de Evaluación de Scheduling en ESP32 con FreeRTOS

Este proyecto proporciona un entorno para analizar y comparar el rendimiento en tiempo real de diferentes estrategias de scheduling de FreeRTOS en un ESP32.

El sistema ejecuta tres tareas simuladas (control de LED, cálculo intensivo y lectura de sensor) y muestra un dashboard en la terminal con métricas de rendimiento como tiempos de respuesta, jitter y cambios de contexto.

El archivo que contiene el articulo realizado con Overleaf es: IEEE_Conference_Template.zip
El video donde se muestra la funcionalidad es Finaal.mp3

## Requisitos del Sistema

### Hardware
*   Una placa de desarrollo ESP32 (el proyecto está configurado para una **ESP32 DevKitC-VE**).

*   Un cable USB para conectar la placa al ordenador.

### Software
*   **Docker Desktop** instalado y en ejecución.

*   **Visual Studio Code** con la extensión [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

*   **Python 3** en la máquina anfitriona (host).

## Instalación y Configuración

Este proyecto está diseñado para ejecutarse dentro de un Dev Container, que ya incluye el toolchain de ESP-IDF y todas las dependencias necesarias. Sin embargo, es necesario instalar herramientas en la máquina anfitriona para comunicar el puerto serie del ESP32 con el contenedor.

1.  **Instalar herramientas en la máquina anfitriona:**
    Abre una terminal en tu máquina (no dentro del contenedor) y ejecuta los siguientes comandos para instalar `esptool` y el servidor RFC2217:

    ```bash
    pip install esptool
    pip install esp_rfc2217_server
    ```

2.  **Abrir el proyecto en el Dev Container:**
    Abre la carpeta del proyecto en VS Code. La extensión Dev Containers te notificará para "Reabrir en Contenedor" ("Reopen in Container"). Haz clic en esa opción. VS Code construirá y abrirá el entorno de desarrollo.

## Ejecución de la Aplicación

### 1. Conectar el ESP32 al Contenedor

Para que el contenedor pueda acceder al ESP32 conectado a tu máquina, necesitas reenviar el puerto serie usando `esp_rfc2217_server`.

1.  **Identifica el puerto serie de tu ESP32:**
    *   **Windows:** Abre el "Administrador de dispositivos" y busca en "Puertos (COM y LPT)". El nombre será algo como `Silicon Labs CP210x USB to UART Bridge (COM3)`.

    *   **Linux:** Ejecuta `ls /dev/tty*` en una terminal. El puerto será probablemente `/dev/ttyUSB0`.

    *   **macOS:** Ejecuta `ls /dev/tty.*`. El puerto será similar a `/dev/tty.SLAB_USBtoUART`.


2.  **Inicia el servidor de reenvío de puertos:**
    En una terminal de tu máquina anfitriona, ejecuta el siguiente comando, reemplazando `<TU_PUERTO_SERIAL>` con el puerto que identificaste en el paso anterior.

    ```bash
    # Ejemplo para Windows
    python -m esp_rfc2217_server -p 4000 COM3

    # Ejemplo para Linux/macOS
    python3 -m esp_rfc2217_server -p 4000 /dev/ttyUSB0
    ```

3.  **Verifica la conexión:**

    La salida debería ser similar a esta. Toma nota de la dirección IP que se muestra (`192.168.56.1` en este ejemplo), ya que la necesitarás más adelante.

    ```
    INFO: RFC 2217 TCP/IP to Serial redirector - type Ctrl-C / BREAK to quit
    INFO: Serving serial port: COM3
    INFO: TCP/IP port: 4000
    INFO: Waiting for connection ... use the 'rfc2217://192.168.56.1:4000?ign_set_control' as a PORT
    ```
    Cuando el contenedor se conecte, verás un mensaje como: `INFO: Connected by 192.168.56.1:54839`.

### 2. Compilar, Flashear y Monitorear

Abre una terminal dentro del Dev Container en VS Code (`Terminal > New Terminal`) y ejecuta los siguientes comandos.

1.  **Compilar el proyecto:**
    ```bash
    idf.py build
    ```

2.  **Poner el ESP32 en modo "Download":**
    Para flashear el firmware, la mayoría de las placas ESP32 necesitan entrar en modo de descarga.
    *   Mantén presionado el botón `BOOT`.
    *   Presiona y suelta el botón `EN` (a veces etiquetado como `RST`).
    *   Suelta el botón `BOOT`.

3.  **Flashear el firmware:**
    Reemplaza la dirección IP si la que te mostró el servidor RFC2217 es diferente.
    ```bash
    idf.py -p rfc2217://192.168.56.1:4000?ign_set_control flash
    ```

4.  **Monitorear la salida:**
    Una vez flasheado, el programa comenzará a ejecutarse. Para ver el dashboard de métricas, ejecuta:
    ```bash
    idf.py -p rfc2217://192.168.56.1:4000?ign_set_control monitor
    ```
    Para salir del monitor, presiona `Ctrl+]`.

# Visualizador de un UR3e por TCP/IP

Práctica de Redes de Comunicación Industriales. Programas en Python que leen
el estado de un robot UR3e (real o simulado con URSim) por TCP/IP,
decodifican sus mensajes binarios y reproducen su postura en PyBullet.

| Script | Nivel | Qué hace |
|---|---|---|
| `primaria.py` | Aprobado | Lee la interfaz primaria (puerto 30001, 10 Hz) y mueve el robot simulado |
| `tiempo_real.py` | Aprobado | Lee la interfaz de tiempo real (puerto 30003, 500 Hz) y dibuja a 30 Hz |
| `BloqueC.py` | Notable | Visualiza varios robots a la vez, con un hilo receptor por robot |
| `BloqueD.py` | Sobresaliente | Envía órdenes URScript: mini-aplicación *Enseñar y reproducir* |

## Instalación

Requiere Python 3.9 o posterior y el visualizador `ur3e_rg2-0.1.0-py3-none-any.whl`
(disponible en el aula virtual; no se incluye en este repositorio).

```powershell
python -m venv venv
venv\Scripts\activate          # Linux: source venv/bin/activate
pip install ur3e_rg2-0.1.0-py3-none-any.whl
```

El entorno virtual hay que activarlo en cada terminal nueva: aparece `(venv)`
al principio de la línea.

## Configurar las IP de los robots

- **`primaria.py` y `tiempo_real.py`**: constante `ROBOT_IP` al principio de
  cada script.
- **`BloqueC.py` y `BloqueD.py`**: fichero `robots.json`, sin tocar el código.
  Cada robot tiene un nombre, su IP, el puerto y su pose en la escena
  `[x, y, z, roll, pitch, yaw]` (m y rad):

```json
[
    {"nombre": "URSim propio",    "ip": "10.172.18.90", "puerto": 30001, "pose": [0, 0, 0, 0, 0, 0]},
    {"nombre": "URSim companero", "ip": "10.172.18.91", "puerto": 30001, "pose": [0, 1, 0, 0, 0, 0]}
]
```

En el aula, la IP es la de la máquina virtual con URSim en modo puente
(red 10.172.18.x). En el laboratorio de robótica, la del UR3e asignado
(10.172.21.203 a .206).

## Ejecución

Desde la carpeta del proyecto, con el entorno virtual activado:

```powershell
python primaria.py
python tiempo_real.py
python BloqueC.py                    # usa robots.json
python BloqueC.py otro.json          # otra configuración
python BloqueD.py                    # primer robot de robots.json
python BloqueD.py robots.json 1      # el segundo robot de la lista
```

Para salir: Ctrl+C, cerrar la ventana de PyBullet o, en `BloqueD.py`, el comando `q`.

### Antes de ejecutar

- URSim encendido desde PolyScope (**ON** y **START**). Hasta que el robot no
  está en modo RUNNING, los programas esperan sin mover el robot simulado.
- `BloqueD.py` envía órdenes, así que además:
  - **Ajustes → Sistema → Control remoto** activado y **Remoto** seleccionado
    arriba a la derecha.
  - La interfaz habilitada en **Ajustes → Seguridad → Servicios**.

## Comportamiento ante desconexiones

- Si se reinicia URSim o se corta la red, los programas reconectan solos.
- Si pasan más de 45 s sin datos (`TIEMPO_MAX_SIN_DATOS`), dan por hecho que
  URSim se ha cerrado y terminan. En `BloqueC.py` solo terminan cuando todos
  los robots llevan ese tiempo sin datos; un robot caído no detiene a los demás.
- Cerrar solo la ventana de PolyScope no basta para que el programa lo
  detecte: el controlador sigue enviando datos. Hay que apagar el robot desde
  PolyScope o apagar la máquina virtual.

## `BloqueD.py`: Enseñar y reproducir

Los comandos se escriben en la consola y se confirman con Enter (`h` muestra la ayuda):

| Comando | Acción |
|---|---|
| `g` | Guardar la posición actual del robot como punto |
| `d` | Guardar la posición de los deslizadores (robot azul) como punto |
| `b` / `v` | Borrar el último punto / ver la lista |
| `p` | Previsualizar la secuencia en el robot azul |
| `e` | Ejecutar la secuencia en el robot, punto a punto |
| `r` / `c` | Reanudar tras una pausa / cancelar |
| `x` | Parar el robot inmediatamente (`stopj`) |
| `s [fichero]` / `l [fichero]` | Guardar / cargar la secuencia (por defecto `secuencia.json`) |
| `q` | Salir |

Cada punto se envía como un `movej` con v = 0,3 rad/s y a = 0,5 rad/s², que
son los límites obligatorios del robot real. El siguiente punto no se envía
hasta que el propio robot confirma el anterior: `isProgramRunning` pasa a
verdadero, vuelve a falso y `q_actual` queda a menos de 0,01 rad del objetivo.
Un error del robot (mensaje de tipo 20, subtipos 6 o 10), una parada de
protección o una pérdida de conexión detienen la secuencia, muestran el motivo
y esperan a que el operador reanude o cancele.

**Seguridad:** ninguna orden se envía a un robot real sin haberla probado antes
en URSim y sin autorización y presencia del profesor.

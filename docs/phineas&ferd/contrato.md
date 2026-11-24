# Contrato entre grupos (borrador v0)

Este documento es la lista exacta de lo que cada grupo publica y consume. Si un mensaje no está aquí, no existe. Cualquier cambio entra por PR y lo aprueba al menos una persona de cada grupo.

## Convenciones

- **Marco `arena`** (propuesta, falta acordarla): el origen está en la esquina interior inferior izquierda de la imagen cenital, sobre el piso; `x` apunta hacia Perry y `y` hacia la caja amarilla, ambos sobre el piso; `z` apunta hacia arriba. Ver `pista.md`.
- **Unidades:** metros, radianes y segundos (REP 103 de ROS). Los `multirobot_interfaces` actuales ya usan metros. Las pantallas pueden mostrar centímetros.
- **Ángulo `theta`:** se mide desde `+x` en sentido antihorario, en el rango `(-pi, pi]`.
- **Dominio ROS:** 0, como en la etapa 1. Versión: ROS 2 Lyrical.
- **Relojes:** las Raspberry Pi no tienen pila de reloj, así que sus horas pueden diferir entre sí y respecto al equipo de la cámara cenital. Ninguna regla de seguridad debe comparar marcas de tiempo de máquinas distintas. Cada nodo mide la edad de un mensaje con su propio reloj, desde que lo recibe.

## Cenital publica

### `/world/state` · tema · objetivo de 20 Hz (mínimo 15)

```text
WorldState
  header              stamp, frame_id = "arena"
  cubes[]             CubeObservation
    id                uint32   estable mientras el cubo siga visible
    color             string   "blue" | "yellow" | "red"
    x, y              float64  m, centro del cubo sobre el piso
    confidence        float32  0..1
  robots[]            RobotObservation
    name              string   "phineas" | "ferb"
    x, y              float64  m, centro del carro proyectado al piso (paralaje corregida)
    theta             float64  rad, dirección de la pala
    confidence        float32  0..1
```

### `/world/sensors` · tema · misma frecuencia

Son los sensores virtuales, que calcula la cenital por recomendación del profesor. Primera lista de lo que la flota necesita:

```text
VirtualSensors
  header
  robot_distance       float64  m, entre los centros de Phineas y Ferb
  robots_in_zone[]     string   carros dentro de la región de la zona
  per_robot[]
    name               string
    nearest_red_id     uint32
    nearest_red_dist   float64  m
    events[]           string   p. ej. "entered:zone", "left:zone", "near:red:7", "near:robot"
```

> La cenital genera la alerta de colisión (`robot_distance < 34 cm` / `"near:robot"`) desde los triángulos físicos del techo (D3), sin añadir stickers ni LEDs rojos o verdes al chasis. Cada carro resuelve el desvío en su Pi con la regla virtual de luces de navegación (babor = rojo, estribor = verde), planificando y validando antes de moverse que no chocará cubos rojos (`≥ 12 cm`) ni la cinta negra.

Falta acordar: el formato de los eventos de región y si se agrega un "pasillo libre" (consulta: "¿hay un rojo entre A y G?"). Este último podría ser un servicio.

### `/world/get` · servicio

La petición va vacía. La respuesta es el último `WorldState` y su `VirtualSensors`. Es la consulta a pedido ("retornan las coordenadas al consultarlas").

### `/world/image` · tema · opcional

Un `sensor_msgs/CompressedImage` para el visor, a baja frecuencia (5 Hz es suficiente).

## Cada carro publica (Pi 5 de Phineas y Ferb)

Cada carro corre su propio planificador, tráfico entre pares, control y HAL localmente en su Raspberry Pi 5. Las órdenes de velocidad `(vx, vy, wz)` son internas dentro de cada Pi (controlador a bordo -> HAL, con vigilancia local de 300 ms) y no viajan por red.

### `/phineas/plan`, `/ferb/plan` · tema · ~5–10 Hz

Contiene el estado de ese carro (`LIBRE`, `EMPUJAR`, …), el id del cubo objetivo, la pose objetivo y la ruta como lista de puntos. Lo usan el compañero para tráfico entre pares, el visor y el registro.

### `/phineas/status`, `/ferb/status` · tema · 5 Hz

```text
line_detected   bool
battery_v       float32
cmd_age_ms      uint32   edad de la última orden interna recibida por el HAL
mode            string   "run" | "stopped_watchdog" | "stopped_line" | "manual"
```

## Manipulador (sin cambios respecto de la etapa 1)

Es el contrato de `multirobot_interfaces` que ya funcionó entre los tres robots:

- `/zone/request`, `/zone/release`: servicios. Cada carro los llama **directamente** desde su Pi, con su propio nombre como solicitante.
- `/zone/occupant`: latido a 2 Hz. Si pasan 2 s sin recibirlo, nadie entra a la zona.
- `/perry/state`: estado del brazo (`gate_closed` = zona cerrada).
- El lease (turno con caducidad) dura 60 s.

## Calidad de servicio sugerida

| Tema | QoS |
| --- | --- |
| `/world/state`, `/world/sensors` | best effort, keep last 1 (datos de sensor: vale el más reciente) |
| `/<robot>/status`, `/<robot>/plan` | reliable, keep last 1 |
| servicios de zona | por defecto |

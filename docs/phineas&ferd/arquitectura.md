# Arquitectura (propuesta)

## Roles

| Grupo | Integrantes (referencia) | Responsabilidad | Corre en |
| --- | --- | --- | --- |
| Cenital | Samuel y compañía | Ve: calibra la cámara, detecta cubos y carros, publica el mundo y **calcula los sensores virtuales**. Solo entrega datos, no da órdenes | PC o Pi de la cámara |
| Flota | Ignacio y compañía | Decide y mueve: misión, tráfico entre pares, control y HAL a bordo de Phineas y Ferb; visor de monitoreo | Pi 5 de cada carro (y pantalla para el visor) |
| Manipulador | Vielma y compañía | Mejora Perry (pinza, velocidad, precisión), mantiene el árbitro de zona y el conteo | Pi de Perry |

En resumen: la cenital "ve pero es mocha" y la flota "no es mocha pero no ve".

## Paradigma

Se usa un paradigma **híbrido y descentralizado**. Cada carro planifica de forma autónoma en su propia Raspberry Pi 5 con la vista de toda la pista, y conserva dos reflejos locales:

- Si ve la línea negra, frena.
- Si pasan 300 ms sin una orden interna nueva hacia el HAL, frena.

## Capas (pizarra del profesor)

| Capa | Contenido | Dueño |
| --- | --- | --- |
| A.C. (aplicación de control) | Misión (qué cubo toma cada carro), tráfico entre pares (quién pasa y quién espera), clasificación (Perry) | cada carro (flota), manipulador |
| A · S · A,S V · C | Actuadores (velocidad de cada carro), sensores (cámara cenital, sensor de línea), sensores y actuadores virtuales, comunicación (el contrato) | flota, cenital, todos |
| HAL (abstracción del hardware) | `drive(vx, vy, wz)`, `stop()`, `line()`. Phineas y Ferb difieren solo en su archivo de configuración | flota |
| RT (tiempo de ejecución) | Lazos a ~20 Hz en cenital y en cada Pi; vigilancia interna de 300 ms en el HAL de cada carro | todos |
| HW | Cámara (1 m de altura) + equipo cenital (PC o Pi), 2 × (Pi 5, Fusion HAT+, 4 motores TT, TCRT5000), Perry | — |

VCI aparece en el cuadro del Sand Box. Suponemos que significa *Virtual Communication Interface*, pero no está confirmado.

## Estructura de carpetas

Las carpetas se nombran por la parte del sistema (arquitectura que grita), cada una tiene un grupo dueño, y la lógica pura va separada de ROS para poder probarla sin hardware.

```text
Tri-State-Area/
├─ README.md                qué es y cómo correr la demo completa
├─ docs/
│  ├─ contrato.md           mensajes entre grupos
│  ├─ pista.md              medidas, origen, ejes, fotos
│  ├─ arquitectura.md       este archivo
│  └─ decisiones/           un registro corto por decisión (ADR)
├─ interfaces/              [los tres grupos · cambios solo por PR revisado]
│  └─ tsa_interfaces/       .msg y .srv de ROS 2
├─ cenital/                 [cenital]
│  ├─ cenital_core/         calibración, detección, sensores virtuales (sin ROS)
│  └─ cenital_ros/          publica /world/state y /world/sensors; atiende /world/get
├─ carro/                   [flota · corre a bordo en la Pi 5 de cada carro]
│  ├─ carro_core/           modelo del mundo, misión, tráfico entre pares, control (sin ROS)
│  ├─ carro_hal/            Fusion HAT+, mecanum, sensor de línea, vigilancia interna
│  ├─ carro_ros/            lee /world/state y /<compañero>/plan → HAL; publica /<robot>/plan y /<robot>/status
│  └─ config/               phineas.yaml, ferb.yaml
├─ perry/                   [manipulador · brazo y árbitro de zona]
├─ visor/                   proyección virtual: el mundo y los planes sobre la imagen
├─ sim/                     mundo falso con el mismo contrato
├─ tools/  docker/
└─ .github/workflows/       pruebas de cada carpeta en cada PR
```

Un archivo `CODEOWNERS` puede asignar cada carpeta a su grupo. Así cada PR lo revisa quien corresponde.

### Qué se reutiliza de los repositorios anteriores

- `kinematics.py` y `motors.py` (Fusion HAT+, mecanum): pasan a `carro/carro_hal/`.
- `multirobot_interfaces` y el árbitro de zona de Perry: son la base de `interfaces/` y `perry/`.
- `peer_avoidance.py`, que ya está escrito en coordenadas de mesa: es una referencia para el tráfico.
- La regla "lógica pura con pruebas, nodo delgado" y el registro de cada decisión.

### Qué no se trae

- La percepción a bordo para navegar: balizas, franjas de color, modelo de cámara inclinada.
- Los pilotos reactivos `autopilot.py` y `behavior_fsm`, que son dos máquinas de estado duplicadas.
- Los nodos de sensores que nunca se montaron.
- La simulación de Webots, que modela otro robot.

## Flujo de la aplicación

El sistema es un lazo que se repite ~20 veces por segundo:

1. **Ver** (cenital): captura, píxel → metros, corrige la paralaje y publica `/world/state`.
2. **Interpretar** (cenital): calcula los sensores virtuales y los publica en `/world/sensors`.
3. **Decidir** (Pi 5 de cada carro):
   - La misión elige o mantiene el cubo objetivo de ese carro.
   - El anti-colisión usa la alerta de cercanía de la cenital (< 34 cm) y la regla de luces de navegación (babor = rojo, estribor = verde), planificando y validando antes de moverse que el desvío no choque cubos rojos (≥ 12 cm) ni la cinta negra.
   - El control convierte el objetivo en velocidades localmente en la Pi.
4. **Actuar** (HAL en la misma Pi 5): recibe `(vx, vy, wz)` localmente y mueve los motores. Frena si ve la línea negra o si vence la vigilancia interna de 300 ms.

### Una entrega completa

```text
Cenital -> Phineas : /world/state y /world/sensors (20 Hz)
Phineas            : elige cubo azul #3, publica /phineas/plan, va hasta A y pre-zona (control local)
Phineas -> Perry   : /zone/request (phineas)
Perry   -> Phineas : concedido (lease 60 s)
Phineas            : empuja hasta pasar la línea verde (control local)
Cenital -> Phineas : cubo #3 en la zona
Phineas            : retrocede recto y sale (control local)
Cenital -> Phineas : sensor virtual: Phineas fuera de la zona
Phineas -> Perry   : /zone/release (1 azul)
Perry              : asentando -> clasifica
```

Cada carro pide y libera el turno directamente desde su Pi. Así el contrato del árbitro de Perry no cambia.

### Estados de cada carro (en su propia Pi)

```text
LIBRE -> IR_A_APROXIMACION -> EMPUJAR -> PEDIR_ZONA -> ENTREGAR -> SALIR -> LIBRE
Interrupciones: ESPERAR (tráfico) · DETENIDO (sin datos de la cámara > 0,5 s) · RECUPERAR (el cubo se salió de la pala)
```

### Control "ir a una pose" (mecanum)

```text
e        = p_objetivo - p_robot                 (mundo)
[vx, vy] = sat( K · R(-theta) · e )             (marco del carro)
wz       = sat( K_theta · (theta_objetivo - theta) )
```

### Empuje guiado

El carro se coloca en un punto A detrás del cubo C, sobre la recta hacia el destino G, y empuja en línea recta:

```text
A = C + d · (C - G) / |C - G|
```

### Anti-colisión: luces de navegación (rojo/verde)

- **Marcas físicas y alerta desde cenital:** la cenital rastrea los triángulos físicos de cartulina del techo (D3) para obtener `(x, y, theta)` y emitir alertas de colisión en `/world/sensors` (`robot_distance < 34 cm` / `"near:robot"`). No se montan stickers ni LEDs rojos o verdes físicos en el chasis para no contaminar la detección HSV de cubos rojos ni la línea verde.
- **Reglas de prioridad (alineadas con la dirección de movimiento):**
  1. *Maniobra restringida:* quien está en `SALIR`, `ENTREGAR` o `EMPUJAR` (con cubo) tiene prioridad sobre un carro vacío.
  2. *Misma condición:* ves **rojo** (compañero entra por tu derecha) → cedes el paso; ves **verde** (entra por tu izquierda) → mantienes tu ruta; **de frente** → ambos se apartan a estribor (`-vy` mecanum; desempate: `ferb` cede); **alcance** → el de atrás frena o rodea.
- **Planificar primero y luego moverse (re-evaluado a 20 Hz):** antes de ordenar un desvío a estribor (`~15–20 cm`), cada Pi valida en `< 0,1 ms` sobre `/world/state` que el paso quede dentro del margen virtual interior y a `≥ 12 cm` de todo cubo rojo (evitando empujar cubos rojos ligeros de 2,5 cm o activar `stopped_line`). Si estribor está bloqueado, el carro frena en `ESPERAR` y evalúa un retroceso recto seguro (`-vx`) si lleva `> 3 s` bloqueado. El chequeo se repite a ~20 Hz mientras avanza.

## Lenguaje

Se usa Python con ROS 2 Lyrical. No se justifica Go ni Rust, por estas razones:

- **El límite es el retardo, no el cálculo.** El Wi-Fi da 560–870 ms de ping (medido por el grupo de Phineas), cada cuadro de la cámara tarda 33–66 ms y Phineas oscilaba con solo 0,15 s de retardo. Ninguna de esas cifras cambia con el lenguaje.
- **OpenCV ya es C++ por dentro.**
- **ROS 2 solo soporta oficialmente Python (`rclpy`) y C++ (`rclcpp`).**
- **El código heredado está en Python**, con ~280 pruebas.

Si una medición muestra que un nodo concreto no da abasto, se optimiza ese nodo, primero con C++.

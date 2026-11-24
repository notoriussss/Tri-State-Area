# Contexto para armar Tri-State-Area

Paquete de contexto para levantar el monorepo `notoriussss/Tri-State-Area` (etapa 2 de Robótica, UCAB Guayana, prof. Jesús Lárez). Lo preparó el grupo de la flota (Phineas y Ferb) el 17-11-2026.

Nada de esto es definitivo. Las decisiones tomadas están en `decisiones.md`, con su fuente. Todo lo demás es una propuesta que se puede discutir.

## Orden de lectura

| Archivo | Qué contiene | Dónde va en el repositorio |
| --- | --- | --- |
| `plan-flota-tri-state.html` | La visión completa con dibujos: roles, pista, capas, flujo, control, hoja de ruta | referencia; no hace falta copiarlo |
| `arquitectura.md` | Roles, capas, estructura de carpetas y flujo de la aplicación | `docs/arquitectura.md` |
| `contrato.md` | Mensajes entre los tres grupos (borrador v0) | `docs/contrato.md` |
| `pista.md` | Medidas, origen, ejes, cámara y paralaje | `docs/pista.md` |
| `decisiones.md` | Lo ya decidido, con su fuente y su fecha | `docs/decisiones/` (una por archivo, si se prefiere) |
| `preguntas.md` | Lo que falta aclarar y a quién preguntarle | issues del repositorio |
| `cenital.jpg` | Captura de la cámara cenital nueva | `docs/img/` |

El HTML necesita `cenital.jpg` en la misma carpeta para mostrar la foto. También está publicado en claude.ai (Ignacio lo comparte).

## Lo urgente

1. **Firmar `contrato.md`** entre los tres grupos. Con el contrato escrito, cada grupo trabaja en paralelo sin esperar a los otros.
2. **Crear `sim/` temprano.** Es un mundo falso que publica los mismos mensajes que la cámara. Permite que la flota y Perry prueben sin la cenital real.
3. **Acordar el origen y los ejes** (`pista.md`). Una vez acordados, los tres grupos deben usar los mismos.

## Fuentes

- Pizarra y notas de la clase del 17-11-2026.
- Informes de Phineas (Ostos, Vielma, Carvajal, Falconi) y de Ferb (Freitas, Guzmán, Lara, Márquez, Sandoval).
- Repositorios `alejandrovielma/ucab-multirobot` y `notoriussss/Ferb-Recolector`.
- Repositorio de Perry, `Kobalt09/Robot_Claw`, y su informe.
- Castillejo, O. (2020). *Desarrollo de una inteligencia artificial para dar solución a problemas de transporte en robótica multiagente*. Trabajo de grado, UCAB. Tutor: Jesús Lárez.

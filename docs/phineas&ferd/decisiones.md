# Decisiones

Cada decisión indica de dónde sale. "Propuesta" significa que todavía no la acordaron los tres grupos.

| # | Decisión | Fuente | Fecha | Estado |
| --- | --- | --- | --- | --- |
| D1 | La cenital ve, publica el mundo y calcula los sensores virtuales. La flota decide y mueve. El manipulador clasifica | profesor, reparto de grupos | 17-11-2026 | decidida |
| D2 | El paradigma es híbrido y descentralizado: cada carro planifica de forma autónoma en su propia Pi 5 con la vista cenital y conserva dos reflejos locales (línea negra y vigilancia interna del HAL) | recomendación del informe de Perry | 17-11-2026 | propuesta |
| D3 | El marcador de los carros es un triángulo de cartulina; el centroide da la posición y la punta la orientación | profesor | 17-11-2026 | decidida |
| D4 | No hay proyector físico. La "proyección" se dibuja en pantalla sobre la imagen (visor) | profesor | 17-11-2026 | decidida |
| D5 | Python con ROS 2 Lyrical; no se usa Go ni Rust salvo que una medición lo justifique | análisis de retardos (ver `arquitectura.md`) | 17-11-2026 | propuesta |
| D6 | Los cubos rojos no se entregan; se evitan. Si alguno llega a la zona, Perry lo descarta | profesor | 17-11-2026 | decidida |
| D7 | Para evitar rojos se prueban dos opciones: el radio de seguridad completo (17 cm) contra la variante de 12 cm más un pasillo libre para empujar | profesor (idea) y flota (variante) | 17-11-2026 | a evaluar con el simulador |
| D8 | El árbitro de zona de Perry se mantiene sin cambios; cada carro pide y libera el turno directamente desde su Pi | contrato de la etapa 1 | 17-11-2026 | propuesta |
| D9 | Hay un monorepo `Tri-State-Area` con carpetas por parte del sistema, cada una con su grupo dueño | equipo; Samuel arma la estructura | 17-11-2026 | decidida (la estructura es una propuesta) |
| D10 | Hay un simulador (`sim/`) que publica el mismo contrato que la cámara real | flota | 17-11-2026 | propuesta |
| D11 | Unidades en metros y radianes (REP 103); marco `arena` con origen en la esquina interior inferior izquierda de la imagen | flota | 17-11-2026 | propuesta |
| D12 | Las cámaras frontales de los carros se pueden conservar, pero la navegación sale de la cenital | Ignacio | 17-11-2026 | decidida |
| D13 | El anti-colisión usa la regla estándar de luces de navegación (babor = rojo, estribor = verde) calculada virtualmente desde los triángulos físicos del techo (D3) y la alerta de colisión de cenital (< 34 cm), planificando y validando antes de moverse (margen interior y ≥ 12 cm de cubos rojos, re-evaluado a 20 Hz); sin stickers ni LEDs rojos/verdes en el chasis | flota | 24-11-2026 | propuesta |

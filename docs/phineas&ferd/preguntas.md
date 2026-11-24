# Preguntas abiertas

Cada pregunta indica a quién hacérsela. Conviene convertirlas en *issues* del repositorio. El resto de las dudas se resolverá en la práctica.

## Cenital

1. ¿Con qué frecuencia y con qué retardo se publicará `/world/state`? De esto dependen las ganancias del control.
2. ¿Qué color y tamaño tendrá el triángulo de cada carro? Proponemos un isósceles alargado de unos 10 cm, con dos colores distintos de los cubos y de la zona.
3. ¿La cámara está justo encima del punto rojo? De eso depende el punto `c` de la corrección de paralaje.
4. ¿Qué sensores virtuales se publicarán y con qué formato? Ver la propuesta en `contrato.md`.
5. ¿Aceptan el origen y los ejes propuestos en `pista.md`?
6. ¿La resolución de trabajo será 1280 × 720?

## Manipulador

7. ¿Se confirma que el árbitro de zona de la etapa 1 se mantiene sin cambios con cada carro llamando directamente a `/zone/request` y `/zone/release` desde su Pi?
8. ¿Qué tan adentro de la línea verde hay que dejar el cubo? ¿Siguen valiendo la franja de 12 a 19,5 cm del eje de Perry y la separación de 1,5 cm entre cubos?

## Profesor

9. ¿Cómo se evaluará esta etapa? Por ejemplo, cubos por minuto, choques, rojos tocados o tiempo total.
10. ¿Acepta la variante de dos pasos para los rojos (12 cm más pasillo libre) en lugar del radio completo?
11. ¿VCI significa *Virtual Communication Interface*? Es nuestra suposición.

## Equipo

12. ¿Usaremos un enrutador propio para los tres grupos? ¿En qué equipo corre la cenital y el visor el día de la demo?
13. ¿Quién se encarga de `sim/` y de `visor/`?

## Medir

14. ¿Cuánto espacio libre hay en el techo de cada carro para el triángulo?
15. ¿Cuál es la latencia real de la red, desde el equipo de la cenital a cada Pi (y entre Phineas y Ferb)?

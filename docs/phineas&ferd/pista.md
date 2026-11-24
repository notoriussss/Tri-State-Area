# La pista

Cada dato indica su fuente y qué tan confiable es.

## Medidas

| Dato | Valor | Fuente | Confianza |
| --- | --- | --- | --- |
| Pista (interior de la cinta negra) | 116,5 × 70 cm | informes de Phineas y Ferb | vigente, en teoría |
| Centro de la pista | punto rojo pintado | confirmado por Ignacio, 17-11 | alta |
| Altura de la cámara sobre la pista | 100 cm exactos | confirmado por Ignacio, 17-11 | alta |
| Resolución de la captura | 1280 × 720 px | `cenital.jpg` | falta confirmar que sea la de trabajo |
| Escala en la imagen | ~9,73 px/cm (~1,03 mm/px) | estimada a ojo sobre la captura | ±1 cm |
| Cubo | 2,5 cm (~24 px en la imagen) | informe de Ferb y captura | alta |
| Carro | 18,6 × 16,1 × 9,1 cm (chasis) | informes de Phineas y Ferb | vigente, en teoría |
| Ruedas | mecanum de 65 mm, 4 motores TT 1:120, sin encoders | informes | alta |

## Disposición (según la captura)

- La **zona de entrega** es la muesca azul del borde derecho, donde la cinta negra se interrumpe. El carro entrega el cubo **pasando la línea verde**.
- La **caja amarilla** está arriba a la derecha y la **caja verde** abajo a la derecha, fuera de la pista.
- **Perry** queda fuera de la foto, a la derecha. La cámara sí cubre el área de entrega.
- En el tramo de la zona no hay cinta negra, así que el sensor de línea no protege ahí. Ese tramo lo vigila la región virtual de la zona.

## Origen y ejes (propuesta)

- El origen está en la esquina **interior inferior izquierda** de la imagen.
- `x` apunta hacia Perry (a la derecha en la imagen) e `y` hacia arriba en la imagen.
- `theta` se mide desde `+x`, en sentido antihorario.
- En la captura, el origen está en ~(30, 695) px y la esquina opuesta en ~(1165, 15) px.

## De píxel a mundo

Como la pista es plana, una homografía `H` de 3 × 3 calculada con las cuatro esquinas interiores de la cinta convierte cada píxel en metros:

```text
[x, y, 1]^T  ∝  H · [u, v, 1]^T
```

Si se mueve la cámara, solo hay que volver a medir las cuatro esquinas.

## Paralaje (importante con la cámara a 1 m)

La homografía es exacta solo sobre el piso. El marcador está en el techo del carro, a ~9 cm de altura, y la cámara lo ve desplazado hacia afuera del punto `c` que tiene justo debajo:

```text
p_real = c + (p_visto - c) · (H - h) / H        H = 1,00 m, h ≈ 0,09 m
error  ≈ r · h / (H - h) ≈ 0,1 · r              (r = distancia a c)
```

| Distancia a `c` | Error sin corregir |
| --- | --- |
| 20 cm | ~2 cm |
| 68 cm (esquina) | ~6,7 cm |

`c` es el punto rojo del centro si la cámara está justo encima de él, cosa que falta confirmar. Los cubos (2,5 cm de alto) también se desplazan, unos 0,9 cm en la esquina. Conviene corregirlos con `h = 0,0125 m` (la mitad de su altura).

## Marcador de los carros

El profesor pidió un **triángulo de cartulina** en el techo de cada carro, el mismo método de Castillejo (2020).

- **Posición:** el centroide del triángulo da `(x, y)`.
- **Orientación:** la punta, que es el vértice opuesto al lado más corto, da `theta`. Por eso conviene un isósceles alargado y no uno equilátero.
- **Colores:** cada carro necesita uno propio, que no sea azul, amarillo ni rojo (los cubos) ni verde (la zona).
- **Montaje:** el centroide del triángulo debe quedar sobre el centro del carro, y la punta debe apuntar hacia la pala.

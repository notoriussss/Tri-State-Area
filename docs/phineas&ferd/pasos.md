Conectar por cable LAN directo es muy buena idea (más rápido y estable). Solo toma en cuenta que, al no haber un router en medio, no habrá DHCP automático a menos que uses una de estas opciones:

### 1. Conectar y probar primero con `.local`
Enchufa el cable Ethernet directo entre la PC y la Raspberry Pi (no hace falta cable cruzado, ambas soportan Auto-MDIX). Espera ~15 segundos y prueba en PowerShell:
```powershell
ping <hostname>.local
ssh <usuario>@<hostname>.local
```
*(Por ejemplo `ucab@ferb.local` o el hostname/usuario que tenga la Pi).*

---

### 2. Si `.local` no responde (lo más común en cable directo)

La forma más práctica en Windows —que además **le da internet a la Pi** para poder correr `setup_pi.sh`— es activar **Compartir Internet (ICS)**:

1. En Windows abre **Conexiones de red** (`Win + R` → escribe `ncpa.cpl` → Enter).
2. Clic derecho en tu adaptador **Wi-Fi** → **Propiedades** → pestaña **Uso compartido**.
3. Marca **"Permitir que los usuarios de otras redes se conecten a través de la conexión a Internet de este equipo"** y en el desplegable elige **Ethernet**.
4. Acepta y espera ~10 segundos. Windows pondrá su Ethernet en `192.168.137.1` y actuará como DHCP para darle IP a la Pi.
5. Busca la IP que recibió la Pi ejecutando en PowerShell:
   ```powershell
   arp -a
   ```
   Busca en la interfaz `192.168.137.1` la IP asignada (por ejemplo `192.168.137.xxx`).

*(Alternativa sin Wi-Fi/ICS: ponle una IP estática manual al Ethernet de Windows, ej. `192.168.10.1` / máscara `255.255.255.0`, y una en la misma subred a la Pi, ej. `192.168.10.2` via `netplan` o monitor/teclado).*

---

### 3. Subir el código por el cable LAN
Una vez tengas la IP (o el `.local`) funcionando, desde PowerShell en `c:\Users\crist\Documents\GitHub\Tri-State-Area`:

```powershell
scp -r .\Ferb-Recolector <usuario>@<ip_de_la_pi>:~/recolector
```

Luego entra por SSH:
```powershell
ssh <usuario>@<ip_de_la_pi>
```
Y dentro de la Pi:
* **Primera vez:** `cd ~/recolector && bash scripts/setup_pi.sh` y luego `sudo reboot`.
* **Si ya estaba instalada:** `cd ~/recolector && source /opt/ros/lyrical/setup.bash && colcon build --symlink-install`.
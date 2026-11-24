Aquí tienes los comandos listos para copiar y pegar (conectado a la red Wi-Fi **`paquidermos`**):

### 1. Iniciar **Phineas** (`10.217.177.13`, clave `phineas`)

**Desde Windows PowerShell (por SSH en segundo plano):**
```powershell
ssh phineas@10.217.177.13 "pkill -f '[c]ollector_node' || true; sleep 1; setsid -f bash -c 'export ROS_DOMAIN_ID=0; source /opt/ros/lyrical/setup.bash; source ~/Tri-State-Area/install/setup.bash; exec ros2 launch fleet_ros collector.launch.py robot:=phineas' > /tmp/collector_phineas.log 2>&1 < /dev/null"
```

**O directamente dentro de la Pi de Phineas:**
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/lyrical/setup.bash
source ~/Tri-State-Area/install/setup.bash
ros2 launch fleet_ros collector.launch.py robot:=phineas
```
* **Interfaz web:** `http://10.217.177.13:8081`

---

### 2. Iniciar **Ferb** (`10.217.177.198`, clave `ferb`)

**Desde Windows PowerShell (por SSH en segundo plano):**
```powershell
ssh ferb@10.217.177.198 "pkill -f '[c]ollector_node' || true; sleep 1; setsid -f bash -c 'export ROS_DOMAIN_ID=0; source /opt/ros/lyrical/setup.bash; source ~/Tri-State-Area/install/setup.bash; exec ros2 launch fleet_ros collector.launch.py robot:=ferb' > /tmp/collector_ferb.log 2>&1 < /dev/null"
```

**O directamente dentro de la Pi de Ferb:**
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/lyrical/setup.bash
source ~/Tri-State-Area/install/setup.bash
ros2 launch fleet_ros collector.launch.py robot:=ferb
```
* **Interfaz web:** `http://10.217.177.198:8081`

---

### Detenerlos / Ver logs (si los corriste en segundo plano)
```powershell
# Detener:
ssh phineas@10.217.177.13 "pkill -f '[c]ollector_node'"
ssh ferb@10.217.177.198 "pkill -f '[c]ollector_node'"

# Ver logs:
ssh phineas@10.217.177.13 "cat /tmp/collector_phineas.log"
ssh ferb@10.217.177.198 "cat /tmp/collector_ferb.log"
```
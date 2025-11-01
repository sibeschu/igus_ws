# Igus Rebel Roboter Template - Anleitung für Studenten

Dieses Template ermöglicht es dir, den Igus Rebel Roboter mit MoveIt zu programmieren.

## Setup - Schritt für Schritt

Du benötigst **3 separate Terminals** für die Roboterprogrammierung:

### Terminal 1: Roboter starten
```bash
cd ~/ROS2-Igus-Einfuehrung-Robotik
source install/setup.bash
ros2 launch igus_rebel rebel.launch.py
```
⏳ Warte bis "igus_rebel_controller_manager started successfully" erscheint.

### Terminal 2: MoveIt Motion Planner starten
```bash
cd ~/ROS2-Igus-Einfuehrung-Robotik
source install/setup.bash
ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py use_gui:=true
```
⏳ Warte bis "You can start planning now!" erscheint und RViz geöffnet ist.

### Terminal 3: Dein Student-Programm starten

**Einmalige Ausführung:**
```bash
cd ~/ROS2-Igus-Einfuehrung-Robotik
source install/setup.bash
ros2 run sample_package student_template
```

**Wiederholte Ausführung (Loop-Modus):**
```bash
ros2 run sample_package student_template --loop
```

## Programm bearbeiten

Die Datei zum Bearbeiten ist:
```
src/sample_package/sample_package/student_template.py
```

Suche nach dem Abschnitt **"STUDENT CODE BEREICH"** in der Funktion `student_program()`. Dort kannst du dein Programm schreiben.

## Nach Änderungen: Neu kompilieren

Wenn du Änderungen am Python-Code vorgenommen hast, musst du das Paket neu bauen:

```bash
cd ~/ROS2-Igus-Einfuehrung-Robotik
colcon build --packages-select sample_package
source install/setup.bash
```

Dann kannst du dein Programm erneut starten (Terminal 3).

---

## Verfügbare Funktionen

### Roboterbewegung

#### `robot.move_to_home()`
Fährt den Roboter in eine sichere Home-Position.

```python
robot.move_to_home()
```

#### `robot.move_to_pose(x, y, z, roll, pitch, yaw)`
Bewegt den Endeffektor zu einer bestimmten Position und Orientierung.

**Parameter:**
- `x, y, z`: Position in Metern (Typ: float)
- `roll, pitch, yaw`: Orientierung in Radiant (Typ: float)

**Rückgabe:** `True` bei Erfolg, `False` bei Fehler

**Beispiel:**
```python
# Bewege zu Position (0.3m vorwärts, 0m seitlich, 0.4m hoch)
# mit Endeffektor nach unten zeigend
robot.move_to_pose(0.3, 0.0, 0.4, 0.0, pi/2, 0.0)
```

#### `robot.gripper_open()`
Öffnet den Greifer.

**Hinweis:** Dies ist ein Platzhalter. Die Implementierung hängt vom verwendeten Greifer ab.

```python
robot.gripper_open()
```

#### `robot.gripper_close()`
Schließt den Greifer.

**Hinweis:** Dies ist ein Platzhalter. Die Implementierung hängt vom verwendeten Greifer ab.

```python
robot.gripper_close()
```

#### `robot.wait(seconds)`
Pausiert das Programm für die angegebene Zeit.

**Parameter:**
- `seconds`: Wartezeit in Sekunden (Typ: float)

```python
robot.wait(2.0)  # Warte 2 Sekunden
```

### Beispielcode

---

## Beispielcode

#### Beispiel 1: Einfache Bewegung

```python
# Fahre zu Position (x=0.3m, y=0.0m, z=0.4m)
# Mit Orientierung: roll=0, pitch=90°, yaw=0
robot.move_to_pose(0.3, 0.0, 0.4, 0.0, pi/2, 0.0)
robot.wait(1.0)
```

#### Beispiel 2: Pick and Place

```python
# Über Objekt positionieren
robot.move_to_pose(0.3, 0.1, 0.3, 0.0, pi/2, 0.0)
robot.gripper_open()

# Absenken zum Greifen
robot.move_to_pose(0.3, 0.1, 0.2, 0.0, pi/2, 0.0)
robot.gripper_close()

# Objekt anheben
robot.move_to_pose(0.3, 0.1, 0.4, 0.0, pi/2, 0.0)

# Zu Zielposition fahren
robot.move_to_pose(0.3, -0.2, 0.3, 0.0, pi/2, 0.0)
robot.gripper_open()
```

#### Beispiel 3: Mehrere Positionen anfahren

```python
positionen = [
    (0.3, 0.1, 0.4, 0.0, pi/2, 0.0),
    (0.3, -0.1, 0.4, 0.0, pi/2, 0.0),
    (0.2, 0.0, 0.5, 0.0, pi/2, 0.0)
]

for i, pos in enumerate(positionen):
    print(f"Position {i+1}/{len(positionen)}")
    robot.move_to_pose(*pos)
    roboter.wait(0.5)
```

#### Beispiel 4: Kreisbewegung

```python
import math
from math import pi

print("Kreisbewegung:")

radius = 0.1
center_x, center_y, center_z = 0.3, 0.0, 0.4

for angle in range(0, 360, 30):  # Alle 30 Grad
    rad = math.radians(angle)
    x = center_x + radius * math.cos(rad)
    y = center_y + radius * math.sin(rad)
    roboter.move_to_pose(x, y, center_z, 0.0, pi/2, 0.0)
    roboter.wait(0.3)
```

## Code-Bereich für Studenten

Im Template gibt es einen markierten Bereich für studentischen Code:

```python
# ===============================================
# HIER STUDENT CODE EINFÜGEN
# ===============================================

# Dein Code hier...

# ===============================================
# ENDE STUDENT CODE
# ===============================================
```

## Koordinatensystem

- **X-Achse**: Nach vorne (vom Roboter weg)
- **Y-Achse**: Nach links
- **Z-Achse**: Nach oben

## Orientierung (Euler-Winkel)

- **Roll**: Rotation um X-Achse (in Radiant)
- **Pitch**: Rotation um Y-Achse (in Radiant)
- **Yaw**: Rotation um Z-Achse (in Radiant)

Beispiel: `pi/2` = 90 Grad

## Fehlerbehandlung

Wenn eine Position nicht erreichbar ist, wird eine Fehlermeldung ausgegeben:
```
[ERROR] [roboter_template]: Bewegungsplanung fehlgeschlagen - Position möglicherweise nicht erreichbar
```

---

## Fehlerbehandlung und Retry-Logik

Das Template versucht automatisch **bis zu 3 Mal**, eine Bewegung auszuführen, wenn sie fehlschlägt. 

In der Konsole siehst du:
- 🎯 **Versuch 1/3**: Erste Ausführung
- ⚠️ **Warnung**: Bei Fehlschlag
- 🎯 **Versuch 2/3**: Automatischer Wiederholungsversuch
- ✓ **Erfolg**: Bewegung erfolgreich
- ❌ **Fehler**: Nach 3 Versuchen fehlgeschlagen

Das Programm stoppt nicht bei Fehlern, sondern protokolliert diese und macht weiter.

---

## Tipps

1. **Immer Home-Position anfahren**: Beginne jedes Programm mit `robot.move_to_home()`
2. **Kleine Schritte**: Teste neue Positionen zuerst mit kleinen Bewegungen
3. **Sicherheitsabstände**: Halte ausreichend Abstand zu Hindernissen
4. **Emergency Stop**: Drücke `Ctrl+C` um das Programm jederzeit zu beenden
5. **Loop-Modus**: Nutze `--loop` für wiederholte Tests ohne Neustart

---

## Troubleshooting

### Problem: "MoveGroup Server nicht erreichbar"
**Lösung**: Stelle sicher, dass der Motion Planner läuft:
```bash
ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py use_gui:=true
```

### Problem: "Position nicht erreichbar"
**Lösung**: 
- Überprüfe die Koordinaten (sind sie im Arbeitsbereich?)
- Verkleinere die Distanz zur aktuellen Position
- Versuche eine andere Orientierung
- Achte auf Singularitäten (z.B. Arm vollständig gestreckt)

### Problem: "Goal abgelehnt" oder "fehlgeschlagen"
**Lösung**:
- Das Template versucht automatisch 3x
- Schaue in die Logs für detaillierte Fehlermeldungen
- Verlangsame Bewegungen oder verkleinere Distanzen

### Problem: Robot bewegt sich nicht
**Lösung**:
1. Überprüfe, ob der Roboter eingeschaltet ist
2. Überprüfe Terminal 1: Roboter-Controller läuft?
3. Überprüfe Terminal 2: MoveIt läuft und zeigt "You can start planning now!"?
4. Schaue in RViz: Wird der Roboter korrekt angezeigt?

---

## Quick Reference Card

| Befehl | Beschreibung | Beispiel |
|--------|--------------|----------|
| `robot.move_to_home()` | Zurück zur Startposition | `robot.move_to_home()` |
| `robot.move_to_pose(x,y,z,r,p,y)` | Bewege zu Position | `robot.move_to_pose(0.3, 0.0, 0.4, 0.0, pi/2, 0.0)` |
| `robot.gripper_open()` | Greifer öffnen | `robot.gripper_open()` |
| `robot.gripper_close()` | Greifer schließen | `robot.gripper_close()` |
| `robot.wait(s)` | Warten | `robot.wait(2.0)` |
| `--loop` | Loop-Modus | `ros2 run sample_package student_template --loop` |

**Koordinaten (in Metern):**
- X: Nach vorne (+) / Hinten (-)
- Y: Nach links (+) / Rechts (-)
- Z: Nach oben (+) / Unten (-)

**Winkel (in Radiant):**
- `pi/2` = 90°
- `pi` = 180°
- `-pi/2` = -90°

---

## Support

Bei Fragen wende dich an deinen Dozenten oder schaue in die MoveIt Dokumentation:
https://moveit.ros.org/

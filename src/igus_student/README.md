# Igus Student - Roboter Programmierung Template

Dieses Paket enthält ein einfaches Template für Studenten, um den Igus Rebel Roboter zu programmieren und zu steuern.

## 📋 Voraussetzungen

- ROS2 Jazzy installiert
- Igus Rebel Roboter verbunden und eingeschaltet
- MoveIt2 konfiguriert
- Workspace gebaut (`colcon build`)

## 🚀 Setup - Schritt für Schritt

Du benötigst **3 separate Terminals** für die Roboterprogrammierung:

### Terminal 1: Roboter-Interface starten

```bash
cd ~/ROS2-Igus-Einfuehrung-Robotik
source install/setup.bash
ros2 launch igus_rebel rebel.launch.py
```

**Warte bis diese Meldung erscheint:**
```
[INFO] [igus_rebel_controller_manager]: igus_rebel_controller_manager started successfully
```

### Terminal 2: MoveIt Motion Planner starten

```bash
cd ~/ROS2-Igus-Einfuehrung-Robotik
source install/setup.bash
ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py use_gui:=true
```

**Warte bis diese Meldung erscheint:**
```
[INFO] [move_group]: You can start planning now!
```

RViz2 öffnet sich automatisch mit der Roboter-Visualisierung.

### Terminal 3: Student-Programm starten

```bash
cd ~/ROS2-Igus-Einfuehrung-Robotik
source install/setup.bash
ros2 run igus_student student_control
```

## 📝 Programm bearbeiten

Die Datei zum Bearbeiten findest du hier:
```
src/igus_student/igus_student/student_robot_control.py
```

Öffne die Datei mit einem Editor (z.B. VS Code, nano, gedit):
```bash
code ~/ROS2-Igus-Einfuehrung-Robotik/src/igus_student/igus_student/student_robot_control.py
```

### 🎯 Student Code Bereich

Suche nach dem Abschnitt `def student_program():` und schreibe deinen Code zwischen:
```python
# ▼▼▼ DEIN CODE HIER ▼▼▼

# Dein Roboter-Programm hier schreiben

# ▲▲▲ DEIN CODE ENDE ▲▲▲
```

## 🛠️ Verfügbare Funktionen

### `move_to_pose(x, y, z, roll, pitch, yaw)`
Bewegt den Roboter-Endeffektor zu einer bestimmten Position mit Orientierung.

**Parameter:**
- `x, y, z`: Position in Metern (float)
  - **X-Achse**: Nach vorne (positiv) / Zurück (negativ)
  - **Y-Achse**: Nach links (positiv) / Rechts (negativ)
  - **Z-Achse**: Nach oben (positiv) / Unten (negativ)
- `roll, pitch, yaw`: Orientierung in Radiant (float)
  - **Roll**: Rotation um X-Achse
  - **Pitch**: Rotation um Y-Achse
  - **Yaw**: Rotation um Z-Achse

**Rückgabe:** `True` bei Erfolg, `False` bei Fehler

**Beispiele:**
```python
# Greifer nach unten zeigend
move_to_pose(0.4, 0.0, 0.3, pi, 0.0, 0.0)

# Greifer horizontal
move_to_pose(0.4, 0.0, 0.3, pi/2, 0.0, 0.0)
```

### `move_to_home()`
Fährt den Roboter zur sicheren Home-Position.

**Rückgabe:** `True` bei Erfolg, `False` bei Fehler

**Beispiel:**
```python
move_to_home()
```

## 📐 Winkel-Hilfe

Die Variable `pi` ist bereits importiert aus dem `math` Modul:

| Grad | Radiant | Konstante |
|------|---------|-----------|
| 0°   | 0       | `0.0` |
| 45°  | 0.785   | `pi/4` |
| 90°  | 1.571   | `pi/2` |
| 180° | 3.142   | `pi` |
| 270° | 4.712   | `3*pi/2` |
| -90° | -1.571  | `-pi/2` |

## 💡 Beispiel-Programme

### Beispiel 1: Einfache Bewegung
```python
def student_program():
    # Zur Home-Position fahren
    move_to_home()
    
    # Zu einer Position fahren
    move_to_pose(0.3, 0.0, 0.35, 0.0, pi/2, 0.0)
```

### Beispiel 2: Mehrere Positionen
```python
def student_program():
    # Home Position
    move_to_home()
    
    # Position 1
    move_to_pose(0.4, 0.1, 0.3, 0.0, pi/2, 0.0)
    
    # Position 2
    move_to_pose(0.4, -0.1, 0.3, 0.0, pi/2, 0.0)
    
    # Position 3
    move_to_pose(0.3, 0.0, 0.4, 0.0, pi/2, 0.0)
    
    # Zurück zu Home
    move_to_home()
```

### Beispiel 3: Pick and Place Simulation
```python
def student_program():
    # Start bei Home
    move_to_home()
    
    # Über Objekt positionieren
    move_to_pose(0.35, 0.1, 0.35, 0.0, pi/2, 0.0)
    
    # Absenken zum "Greifen"
    move_to_pose(0.35, 0.1, 0.25, 0.0, pi/2, 0.0)
    
    # Anheben
    move_to_pose(0.35, 0.1, 0.35, 0.0, pi/2, 0.0)
    
    # Zu Zielposition bewegen
    move_to_pose(0.35, -0.15, 0.30, 0.0, pi/2, 0.0)
    
    # Zurück zu Home
    move_to_home()
```

## 🔄 Nach Änderungen neu kompilieren

Wenn du Änderungen am Python-Code vorgenommen hast:

```bash
cd ~/ROS2-Igus-Einfuehrung-Robotik
colcon build --packages-select igus_student
source install/setup.bash
```

Dann kannst du dein Programm erneut starten (Terminal 3).

## 🐛 Troubleshooting

### Problem: "MoveGroup Server nicht erreichbar"
**Lösung:** 
- Prüfe ob Terminal 2 läuft und die Meldung "You can start planning now!" zeigt
- Starte den Motion Planner neu

### Problem: "Goal wurde abgelehnt"
**Lösung:**
- Die Zielposition ist möglicherweise außerhalb des Arbeitsbereichs
- Versuche kleinere Bewegungen
- Prüfe ob die Koordinaten im erlaubten Bereich sind

### Problem: "Position lag" Fehler im Roboter-Log
**Lösung:**
- Roboter versucht zu schnell zu bewegen
- Bewegungen sind zu groß
- Lass den Roboter zur Home-Position fahren und versuche es erneut

### Problem: Robot bewegt sich nicht
**Lösung:**
1. Prüfe alle 3 Terminals:
   - Terminal 1: Roboter-Interface läuft?
   - Terminal 2: MoveIt läuft und zeigt "You can start planning now!"?
   - Terminal 3: Programm läuft ohne Fehler?
2. Prüfe in RViz ob der Roboter korrekt angezeigt wird
3. Prüfe ob der physische Roboter eingeschaltet ist

### Problem: Programm stoppt mit Fehler
**Lösung:**
- Lies die Fehlermeldung sorgfältig
- Prüfe die Syntax deines Codes
- Prüfe ob alle Koordinaten als Zahlen (float) angegeben sind
- Stelle sicher, dass `pi` für Winkel verwendet wird

## 📚 Weitere Informationen

- **MoveIt Documentation**: https://moveit.ros.org/
- **ROS2 Documentation**: https://docs.ros.org/en/jazzy/

## 🔒 Sicherheitshinweise

1. **Arbeitsbereich freihalten**: Achte darauf, dass sich keine Personen oder Gegenstände im Bewegungsbereich des Roboters befinden
2. **Notaus**: Drücke `Ctrl+C` im Programm-Terminal um das Programm sofort zu stoppen
3. **Langsam starten**: Teste neue Positionen zuerst mit kleinen Bewegungen
4. **Sicherheitsabstand**: Halte immer ausreichend Abstand zum Roboter während der Bewegung

## 📞 Support

Bei Fragen oder Problemen wende dich an deinen Dozenten oder Betreuer.

---

**Viel Erfolg beim Programmieren! 🤖**

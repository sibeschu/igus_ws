#!/usr/bin/env python3
"""
KeyboardMoveSample ROS2 Node mit Curses UI

Dieser Node erlaubt die Steuerung eines Roboters über die Tastatur direkt
im Terminal. Die Studierenden sollen nachvollziehen, wie ROS2, Publisher,
Service-Clients und eine curses-basierte UI zusammenwirken.

Ablauf:

1. Initialisierung:
   - Node wird erstellt (`KeyboardMoveSample`).
   - Publisher für `TwistStamped` (/delta_twist_cmds) und `JointJog` (/delta_joint_cmds) werden erzeugt.
   - Service-Client für `/servo_node/switch_command_type` wird erstellt, um den
     Servo-Modus (Joint oder Twist) zu setzen.
   - Timer für kontinuierliche Bewegung wird gestartet (10 Hz).

2. Curses UI:
   - Läuft im Hauptthread und liest Tastatureingaben non-blocking.
   - Pfeiltasten, Home/End und 1-6 werden erkannt.
   - PageUp/PageDown ändern die Geschwindigkeit.
   - 'm' toggelt zwischen Joint- und Twist-Modus.
   - Leertaste löst Notfall-Stop aus.
   - ESC beendet die Steuerung.
   - UI zeigt Node-Name, aktuellen Modus, ausgewähltes Gelenk, Velocity, aktive Tasten
     und die Topics, an die gepublished wird.

3. Bewegung:
   - Timer-Callback prüft aktuell gedrückte Tasten (`active_keys`) und sendet
     entsprechende Joint- oder Twist-Kommandos.
   - Bei keinem Tastendruck werden keine Bewegungen ausgeführt.
   - Stop-Methoden senden wiederholt 0-Befehle für Sicherheit.

4. Moduswechsel:
   - Beim Umschalten zwischen Joint- und Twist-Modus wird der ServoCommandType
     über den Service-Client gesetzt (0=JOINT_JOG, 1=TWIST).

5. Cleanup:
   - Bei Beenden (ESC oder Ctrl+C) werden alle Bewegungen gestoppt.
   - Node wird sauber heruntergefahren.

Das Skript verknüpft so ROS2-Programmierung, Echtzeit-UI im Terminal
und Tastatureingaben zu einer übersichtlichen Steuerung.

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType

import curses
import threading
import time
from typing import Dict, Set

KEY_HOLD_TIMEOUT = 0.2      # Sekunden, wie lange eine Taste als gehalten gilt
STATUS_REFRESH = 0.1        # Sekunden zwischen UI-Aktualisierungen
MOVEMENT_TIMER_PERIOD = 0.0002 # Sekunden für kontinuierliche Bewegungsupdates (50 Hz)


class KeyboardMoveSample(Node):
    def __init__(self):
        super().__init__('keyboard_control_curses')

        # Publisher
        self.twist_topic = '/delta_twist_cmds'
        self.joint_topic = '/delta_joint_cmds'
        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.joint_pub = self.create_publisher(JointJog, self.joint_topic, 10)

        # Steuerungszustand
        self.joint_mode = True  # True=Joint, False=Twist
        self.selected_joint = 0
        self.velocity = 0.01

        # aktive Tasten: keycode -> letzter Timestamp
        self.active_keys: Dict[int, float] = {}
        self.ak_lock = threading.Lock()

        # Joint-Namen
        self.joint_names = [f'joint{i+1}' for i in range(6)]

        # Timer für kontinuierliche Bewegung
        self.movement_timer = self.create_timer(MOVEMENT_TIMER_PERIOD, self.movement_callback)

        # Service-Client für ServoCommandType
        self.mode_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        
        # Initialen Modus setzen
        self.set_command_type(0 if self.joint_mode else 1)

    # ---------- Tastenerfassung ----------
    def key_seen(self, key_code: int):
        """Speichert eine erkannte Taste mit aktuellem Timestamp."""
        with self.ak_lock:
            self.active_keys[key_code] = time.monotonic()

    def keys_current(self) -> Set[int]:
        """Gibt aktuell als gehalten betrachtete Tasten zurück."""
        with self.ak_lock:
            now = time.monotonic()
            keys = {k for k, t in self.active_keys.items() if now - t <= KEY_HOLD_TIMEOUT}
            # Stale Tasten entfernen
            for k, t in list(self.active_keys.items()):
                if now - t > KEY_HOLD_TIMEOUT:
                    del self.active_keys[k]
            return keys

    def clear_keys(self):
        with self.ak_lock:
            self.active_keys.clear()

    # ---------- Bewegung ----------
    def set_command_type(self, cmd_type: int):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Servo switch_command_type service not available")
            return
        req = ServoCommandType.Request()
        req.command_type = cmd_type
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Command type set to {cmd_type}")
        else:
            self.get_logger().error(f"Failed to set command type: {future.exception()}")

    def movement_callback(self):
        """Sendet Joint- oder Twist-Kommandos basierend auf gedrückten Tasten."""
        keys = self.keys_current()
        if not keys:
            return

        if self.joint_mode:
            for k in keys:
                if k == curses.KEY_UP:
                    self.move_joint(self.selected_joint, self.velocity)
                elif k == curses.KEY_DOWN:
                    self.move_joint(self.selected_joint, -self.velocity)
        else:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            lx = ly = lz = 0.0
            for k in keys:
                if k == curses.KEY_UP:
                    lx += self.velocity
                elif k == curses.KEY_DOWN:
                    lx -= self.velocity
                elif k == curses.KEY_LEFT:
                    ly += self.velocity
                elif k == curses.KEY_RIGHT:
                    ly -= self.velocity
                elif k == curses.KEY_HOME:
                    lz += self.velocity
                elif k == curses.KEY_END:
                    lz -= self.velocity
            msg.twist.linear.x = lx
            msg.twist.linear.y = ly
            msg.twist.linear.z = lz
            if lx or ly or lz:
                self.twist_pub.publish(msg)

    def move_joint(self, joint_idx: int, velocity: float):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = [self.joint_names[joint_idx]]
        msg.velocities = [velocity]
        msg.duration = 0.0
        self.joint_pub.publish(msg)

    def stop_all(self):
        """Sendet Stop-Befehle für alle Gelenke und Twist."""
        for j in range(len(self.joint_names)):
            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.joint_names = [self.joint_names[j]]
            msg.velocities = [0.0]
            msg.duration = 0.0
            for _ in range(2):
                self.joint_pub.publish(msg)

        tmsg = TwistStamped()
        tmsg.header.stamp = self.get_clock().now().to_msg()
        tmsg.header.frame_id = 'base_link'
        for _ in range(2):
            self.twist_pub.publish(tmsg)

    def emergency_stop(self):
        self.get_logger().warn("EMERGENCY STOP triggered (keyboard).")
        self.stop_all()

    def toggle_mode(self):
        """Wechselt zwischen Joint- und Twist-Modus und setzt ServoCommandType."""
        self.joint_mode = not self.joint_mode
        cmd_type = 0 if self.joint_mode else 1
        self.set_command_type(cmd_type)

    # ---------- UI Status ----------
    def get_status(self) -> Dict:
        """Gibt aktuelle Statusinfos für die UI zurück."""
        mode = "JOINT" if self.joint_mode else "TWIST"
        joint = f"joint{self.selected_joint + 1}" if self.joint_mode else "N/A"
        with self.ak_lock:
            active = list(self.active_keys.keys())
        return {
            "node_name": self.get_name(),
            "mode": mode,
            "selected_joint": joint,
            "velocity": self.velocity,
            "active_keys": active,
            "publishes": [self.joint_topic, self.twist_topic],
        }


# ---------- Curses UI ----------
def curses_main(stdscr, node: KeyboardMoveSample):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    stdscr.clear()

    def header(text, line=0):
        stdscr.attron(curses.A_BOLD)
        stdscr.addstr(line, 2, text)
        stdscr.attroff(curses.A_BOLD)

    last_status_update = 0.0
    try:
        while rclpy.ok():
            now = time.time()
            k = stdscr.getch()

            if k != -1:
                if k == 27:
                    break
                elif k == curses.KEY_PPAGE:
                    node.velocity += 0.01
                elif k == curses.KEY_NPAGE:
                    node.velocity = max(0.001, node.velocity - 0.01)
                elif k in (ord('m'), ord('M')):
                    node.toggle_mode()
                elif k in (ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6')):
                    node.selected_joint = int(chr(k)) - 1
                elif k == ord(' '):
                    node.emergency_stop()
                    node.clear_keys()
                else:
                    node.key_seen(k)

            if now - last_status_update >= STATUS_REFRESH:
                status = node.get_status()
                stdscr.erase()
                header("=== Roboter Tastatur-Steuerung ===", 0)
                stdscr.addstr(2, 2, f"Node: {status['node_name']}")
                stdscr.addstr(3, 2, f"Publisht auf: {', '.join(status['publishes'])}")

                stdscr.addstr(5, 2, "=== Einstellungen ===")
                stdscr.addstr(6, 4, f"Modus: {status['mode']}")
                stdscr.addstr(7, 4, f"Ausgewähltes Gelenk: {status['selected_joint']}")
                stdscr.addstr(8, 4, f"Geschwindigkeit: {status['velocity']:.3f}")

                stdscr.addstr(10, 2, "=== Steuerung ===")
                stdscr.addstr(11, 4, "Pfeiltasten : Bewegung")
                stdscr.addstr(12, 4, "Leertaste: Notfall-Stop")
                stdscr.addstr(13, 4, "M: Modus wechseln (Joint/Twist)")
                stdscr.addstr(14, 4, "1-6: Joint wählen")
                stdscr.addstr(15, 4, "PageUp/PageDown: Geschwindigkeit anpassen")
                stdscr.addstr(16, 4, "ESC: Beenden")

                stdscr.addstr(18, 2, "=== Aktive Tasten (letzte 200 ms) ===")
                keys = status['active_keys']
                if keys:
                    # show readable names for arrow/home/end if present
                    readable = []
                    for kk in keys:
                        if kk == curses.KEY_UP:
                            readable.append("UP")
                        elif kk == curses.KEY_DOWN:
                            readable.append("DOWN")
                        elif kk == curses.KEY_LEFT:
                            readable.append("LEFT")
                        elif kk == curses.KEY_RIGHT:
                            readable.append("RIGHT")
                        elif kk == curses.KEY_HOME:
                            readable.append("HOME")
                        elif kk == curses.KEY_END:
                            readable.append("END")
                        else:
                            # printable?
                            try:
                                ch = chr(kk)
                                if ch.isprintable():
                                    readable.append(ch)
                                else:
                                    readable.append(str(kk))
                            except Exception:
                                readable.append(str(kk))
                    stdscr.addstr(19, 4, ", ".join(readable))
                else:
                    stdscr.addstr(19, 4, "—")

                stdscr.refresh()
                last_status_update = now

            # small sleep to avoid busy loop
            time.sleep(0.01)

    finally:
        # on exit make sure robot stops
        node.get_logger().info("Exiting UI, sending stops...")
        node.stop_all()


def main():
    rclpy.init()
    node = KeyboardMoveSample()

    # start rclpy.spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run curses UI in main thread
    try:
        curses.wrapper(curses_main, node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.get_logger().info("Shutting down node...")
        try:
            node.cleanup_before_exit  # keep existence; if you implemented cleanup_before_exit, call it
        except Exception:
            pass
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()
        print("Roboter-Steuerung beendet.")


if __name__ == '__main__':
    main()

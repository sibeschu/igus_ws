# Einführung in die Robotik mit dem Igus-Rebel und ROS2

## Inhaltsverzeichnis

- [Grundwissen](#grundwissen)
- [ROS2 Installation](#ros2-installation)
- [Igus Installation](#igus-rebel-roboterarm-installation)
- [Kamera Installation](#realsense-kamera-installation)

## Grundwissen

### Pakete
Für Ubuntu Noble (24.04) gibt es [Debian Pakete](https://de.wikipedia.org/wiki/Debian-Paket) (Wikipedia) für ROS2 Jazzy Jalisco.

Im Gegensatz zu Windows wird hier keine .exe-Datei runtergeladen und ausgeführt, sondern das Programm über eine einfache Terminal-Eingabe installiert.

### Versionen
Noble und Jazzy Jalisco sind Namen für die jeweiligen Versionen von Ubuntu oder ROS2. So gibt es zum Beispiel ROS2 Versionen die Iron Irwini, Humble Hawksbill oder Foxy Fitzroy heißen und Ubuntu Versionen, die Trusty Tahr (14.04) oder Bionic Beaver (18.04) heißen. Jazzy Jalisco wird von Mai 2024 bis Mai 2029 gepflegt. Kilted Kaiju bis November 26 und Rolling Ridley von Juni 2020 bis "Ongoing".

Wir nutzen hier Jazzy, da die verwendeten ROS2-Pakete unter Jazzy getestet sind.

## ROS2 Installation

## Schritt 1

### Vorbereitung

#### System Einstellungen

Zuerst gehen wir sicher, dass unser Ubuntu System UTF-8 nutzt. Das ist wichtig für die Zeichenkodierung in z.B. Pfaden, Zeichenketten oder Meldungen. Sonst kann es zu Zeichenfehlern, Encoding-Problemen oder Programmfehlern kommen.

Mit <kbd>STRG</kbd> + <kbd>ALT</kbd> + <kbd>T</kbd> rufen wir ein neues Terminal auf.

Wir geben `locale` ein und prüfen, ob unser System mit UTF-8 arbeitet.

Falls nicht können wir dies mit

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

sicherstellen.

(Falls man lieber die deutsche Sprache als Systemsprache verwendet, kann man überall in den Befehlen "en_US" durch "de_DE" ersetzen.)

Mit `locale` überprüfen wir erneut die aktuelle Einstellung.

#### Erforderliche Verzeichnisse aktivieren

Zuerst gehen wir sicher, dass das Ubuntu Universe Verzeichnis aktiviert ist. (Mehr über [Verzeichnisse/Repositories](https://help.ubuntu.com/community/Repositories/Ubuntu))

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

## Igus-Rebel Roboterarm Installation

## RealSense Kamera Installation

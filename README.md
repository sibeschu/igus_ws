# Einführung in die Robotik mit dem Igus-Rebel und ROS2

## Inhaltsverzeichnis

- [Grundwissen](#grundwissen)
- [ROS2 Installation](#ros2-installation)
- [Igus Installation](#igus-rebel-roboterarm-installation)
- [Kamera Installation](#realsense-kamera-installation)

## Hinweis

Bitte lesen Sie genau und vollständig. Dieser Text beinhaltet einige Linuxbefehle die `im Text` vorkommen und einige die
```bash
in einem Code-Fenster angezeigt werden.
```
.

## Grundwissen

### Pakete
Für Ubuntu Noble (24.04) gibt es Debian Pakete für ROS2 Jazzy Jalisco. (Mehr über [Debian Pakete](https://de.wikipedia.org/wiki/Debian-Paket))

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

#### Erforderliche Repositories

Zuerst gehen wir sicher, dass das Ubuntu Universe Verzeichnis aktiviert ist. (Mehr über [Verzeichnisse/Repositories](https://help.ubuntu.com/community/Repositories/Ubuntu))

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Wir installieren ros2-apt-source Pakete. Diese konfigurieren ROS2 Verzeichnisse für unser System. Updates werden dann automatisch angezeigt, wenn neue Versionen für unsere Pakete veröffentlicht werden. Die Installation der ros2-apt-source Pakete erfolgt so:

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Da wir später ROS2 Pakete bauen wollen, installieren wir auch die entsprechenden Entwicklungswerkzeuge.

```bash
sudo apt update && sudo apt install ros-dev-tools
```

## Schritt 2
### Installation

Mit `sudo apt update`aktualisieren wir unseren "Repository cache". (Diesen Befehl haben wir bereits mehrfach verwendet, aber falls bei diesem Schritt eingestiegen wird steht er hier erneut.)

Bevor wir ROS2 installieren updaten wir unsere Systemdateien mit `sudo apt upgrade`.

Wir installieren nun ROS, RViz, demos und tutorials mit

```bash
sudo apt install ros-jazzy-desktop
```

Um unsere ROS2 Entwicklungsumgebung zu aktivieren schreiben wir `source /opt/ros/jazzy/setup.bash`.

Um die Installation zu verifizieren geben wir `ros2 run demo_nodes_cpp talker` ein.
Wir sollten nun Nachrichten wie diese sehen:
```bash
[INFO] [1760010581.519717306] [talker]: Publishing: 'Hello World: 1'
[INFO] [1760010582.519676343] [talker]: Publishing: 'Hello World: 2'
```

Wenn wir diesen Output sehen, öffnen wir ein neues Terminal mit <kbd>STRG</kbd> + <kbd>ALT</kbd> + <kbd>T</kbd> und geben diesmal `ros2 run demo_nodes_py listener` ein.
Wir sollten in diesem zweiten Terminal nun Nachrichten wie diese sehen:
```bash
[INFO] [1760010764.592431582] [listener]: I heard: [Hello World: 5]
[INFO] [1760010765.584406703] [listener]: I heard: [Hello World: 6]
```



## Igus-Rebel Roboterarm Installation

## RealSense Kamera Installation

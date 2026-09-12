# Heizungs Instrument Reader 1.0.2

## Korrektur in 1.0.2

- behebt die fehlenden Laufzeitvariablen für Confidence-, Start- und Sprungfilter
- bestätigt größere Änderungen standardmäßig mit zwei Messungen innerhalb von
  etwa 15 Sekunden

## Neu in 1.0.1

- anhand der Nahaufnahme korrigierte Thermometerskala: 20 °C bei 305°,
  60 °C bei 270° und 100 °C bei 235°
- stärkere Gewichtung einer durchgehenden dunklen Linie direkt ab Drehpunkt;
  Skalenstriche und Beschriftung am Außenrand werden geringer gewichtet
- getrennte Confidence-Grenzen für Druck und Temperaturen
- drei konsistente Startmessungen vor der ersten Freigabe
- einzelne Sprünge über 3 °C beziehungsweise 0,15 bar werden verworfen
- ein größerer neuer Messwert wird erst nach vier konsistenten Wiederholungen
  akzeptiert, damit reale länger anhaltende Änderungen weiterhin möglich sind
- Log zeigt Rohwert, gefilterten Wert und den jeweiligen Filtergrund
- FFmpeg-Fehler geben die RTSP-Adresse und Zugangsdaten nicht mehr aus

Gemeinsame Home-Assistant-App für drei analoge Heizungsinstrumente:

- Heizungsdruck
- Rücklauftemperatur (linkes/blaues Thermometer)
- Zulauftemperatur (rechtes/rotes Thermometer)

Pro Messzyklus wird nur ein RTSP-Frame abgerufen und einmal dekodiert. Die App
lokalisiert das bekannte Dreier-Cluster gemeinsam und wertet danach drei kleine
ROIs aus.

## Ressourcenstrategie

- Messung aller drei Instrumente standardmäßig alle 15 Sekunden
- vollständige Clusterkontrolle beim Start und alle 600 Sekunden
- vorzeitige neue Clustersuche nach zwei unsicheren Zyklen
- ein FFmpeg-Aufruf pro Zyklus statt je eines pro Einzel-App
- ein gemeinsamer Satz aus Kalibrierung, Medianfiltern und MQTT-Diagnose

## Umstellung

Vor dem Start dieser App beide bisherigen Apps stoppen:

```bash
ha apps stop 37884730_manometer_reader
ha apps stop 37884730_thermometer_reader
```

Der tatsächliche zweite Slug kann abweichen und lässt sich mit `ha apps list`
prüfen. Erst danach den `instrument_reader` starten. So konkurrieren keine
mehreren MQTT-Clients und RTSP-Abfragen miteinander.

Die neue App verwendet für die Hauptwerte dieselben MQTT-Discovery-IDs wie die
bisherigen Apps, damit bestehende Home-Assistant-Entity-IDs erhalten bleiben:

- `sensor.heizungsdruck`
- `sensor.heizung_ruecklauf_temperatur`
- `sensor.heizung_zulauf_temperatur`

Abhängig von bereits vorgenommenen manuellen Umbenennungen können die sichtbaren
Entity-IDs abweichen; maßgeblich ist das Gerät `Heizungsinstrumente`.

## Debugkameras

- `camera.heizung_thermometer_debug_vollbild`
- `camera.heizung_ruecklauf_debug`
- `camera.heizung_zulauf_debug`
- `camera.manometer_heizung_debug_roi`

Alle Bilder werden nach jedem 15-Sekunden-Zyklus aktualisiert. Die Vollbildkamera
steht bereits während der dreifachen Erstbestätigung zur Verfügung.

## Dashboard

```yaml
type: vertical-stack
cards:
  - type: entities
    title: Heizungsinstrumente
    entities:
      - entity: sensor.heizungsdruck
      - entity: sensor.heizung_ruecklauf_temperatur
      - entity: sensor.heizung_zulauf_temperatur

  - type: horizontal-stack
    cards:
      - type: picture-entity
        entity: camera.heizung_ruecklauf_debug
        name: Rücklauf
        camera_view: auto
        show_state: false
      - type: picture-entity
        entity: camera.heizung_zulauf_debug
        name: Zulauf
        camera_view: auto
        show_state: false

  - type: horizontal-stack
    cards:
      - type: picture-entity
        entity: camera.manometer_heizung_debug_roi
        name: Heizungsdruck
        camera_view: auto
        show_state: false
      - type: picture-entity
        entity: camera.heizung_thermometer_debug_vollbild
        name: Cluster
        camera_view: auto
        show_state: false
```

## Optionen

Für den ersten Start dieselbe RTSP-URL und dieselben MQTT-Zugangsdaten wie in
den bisherigen Apps verwenden. Die gemeinsame App besitzt mit
`heizung/instrumente` ein eigenes MQTT-Basistopic und mit
`ha_instrument_reader` einen eigenen Clientnamen.

Die Thermometerskala ist auf 20 °C bei 305°, 60 °C bei 270° und 100 °C bei
235° eingestellt. Der Drehpunktversatz `-0.55 × Radius` folgt der gelieferten
Nahaufnahme. Die Druckskala übernimmt 0–4 bar aus dem bisherigen Reader.

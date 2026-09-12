# Heizung Thermometer Reader 1.0.0

Eigenständige Home-Assistant-App zum optischen Auslesen der beiden analogen
Heizungsthermometer über denselben Tapo-RTSP-Stream wie der Manometer Reader.

## Zuordnung

- linkes, blau markiertes Thermometer: **Rücklauftemperatur**
- rechtes, rot markiertes Thermometer: **Zulauftemperatur**

Die App findet das bekannte Dreier-Cluster aus Rücklauf, Zulauf und dem unteren
Manometer als Positions-, Größen- und Rotationsreferenz. Ausgewertet werden nur
die beiden oberen Thermometer.

## Skala

- rechts unten: 20 °C / 315°
- Mitte unten: 60 °C / 270°
- links unten: 100 °C / 225°
- Drehrichtung von 20 nach 100: im Uhrzeigersinn

Der Zeigerdrehpunkt liegt bei diesem Thermometertyp oberhalb des geometrischen
Zifferblattmittelpunkts. Der Standardwert
`needle_center_y_ratio: -0.55` berücksichtigt die Nahaufnahme.

## Home-Assistant-Entities

MQTT Discovery erzeugt unter dem Gerät `Heizungsthermometer`:

- `sensor.heizung_ruecklauf_temperatur`
- `sensor.heizung_zulauf_temperatur`
- Gültigkeits- und Qualitätssensoren
- Clusterstatus und Kameradrehung
- `camera.heizung_thermometer_debug_vollbild`
- `camera.heizung_ruecklauf_debug`
- `camera.heizung_zulauf_debug`

Die Bilder aktualisieren sich nach jedem Messzyklus automatisch.

```yaml
type: vertical-stack
cards:
  - type: entities
    title: Heizungstemperaturen
    entities:
      - entity: sensor.heizung_ruecklauf_temperatur
        name: Rücklauf
      - entity: sensor.heizung_zulauf_temperatur
        name: Zulauf

  - type: horizontal-stack
    cards:
      - type: picture-entity
        entity: camera.heizung_ruecklauf_debug
        name: Rücklauf-Erkennung
        camera_view: auto
        show_state: false
      - type: picture-entity
        entity: camera.heizung_zulauf_debug
        name: Zulauf-Erkennung
        camera_view: auto
        show_state: false

  - type: picture-entity
    entity: camera.heizung_thermometer_debug_vollbild
    name: Instrumenten-Cluster
    camera_view: auto
    show_state: false
```

## Installation

Den Ordner `thermometer_reader` zusätzlich zum vorhandenen
`manometer_reader` in dasselbe Add-on-Repository übernehmen. Anschließend das
Repository in Home Assistant aktualisieren und `Heizung Thermometer Reader`
installieren.

Die App benötigt dieselbe `rtsp_url` und dieselben MQTT-Zugangsdaten wie der
Manometer Reader. Der eigene MQTT-Clientname und das eigene Basistopic verhindern
Konflikte, wenn beide Apps gleichzeitig laufen.

Für einen Pi 4 ist ein Intervall von 60 Sekunden empfohlen. Beide Apps öffnen
derzeit unabhängig voneinander jeweils einen Frame aus dem RTSP-Stream.

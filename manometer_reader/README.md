# Manometer Reader 0.4.2

Anlagenspezifische Home-Assistant-App zur optischen Auswertung des analogen
Heizungsmanometers über RTSP/Tapo, OpenCV und MQTT.

## Korrekturen in 0.4.2

- verwendet die reale Geometrie der Anlage: zwei Thermometer nebeneinander,
  das Manometer um etwa 0,73 Referenzbreiten nach rechts und 1,61 nach unten
- sucht nur im Anlagenbereich des Bildes
- verwendet für das 3er-Cluster eigene Radien von 14 bis 38 Pixel; die alte
  Option `auto_max_radius: 80` kann daher keinen Großkreis mehr einschleusen
- verwirft automatisch jede Kalibrierung aus 0.4.0/0.4.1
- schreibt bereits während 1/3, 2/3 und bei Fehlsuche ein Debug-Vollbild
- protokolliert die Koordinaten und Radien von T1, T2 und Manometer
- veröffentlicht Vollbild und Manometer-ROI als MQTT-Kameras
- korrigiert die Versionsangabe im Startlog

## Erststart

Eine alte Kalibrierung wird automatisch ignoriert. Danach müssen drei
aufeinanderfolgende Bilder dasselbe kleine Instrumentencluster liefern. Der
Drucksensor bleibt bis dahin absichtlich `unavailable`.

Erwartete Größenordnung im aktuellen 1920×1080-Kamerabild:

| Instrument | X | Y | Radius |
| --- | ---: | ---: | ---: |
| Thermometer links | ca. 965 | ca. 358 | 20–25 px |
| Thermometer rechts | ca. 1075 | ca. 358 | 20–25 px |
| Manometer | ca. 1155 | ca. 535 | 20–25 px |

Das Log meldet bei jeder Clustersuche die wirklich erkannten Werte. Ein Radius
deutlich über 38 Pixel kann in dieser Version nicht als Cluster-Instrument
akzeptiert werden.

## MQTT-Kameras

MQTT Discovery erzeugt automatisch:

- `camera.manometer_heizung_debug_vollbild`
- `camera.manometer_heizung_debug_roi`

Das Vollbild wird auch während der Kalibrierung aktualisiert. Das ROI-Bild ist
verfügbar, sobald ein Manometer bestätigt und ausgewertet wurde. Beide Bilder
werden nach jedem Messzyklus aktualisiert; ein Dashboard-Reload-Timer ist nicht
erforderlich.

```yaml
type: vertical-stack
cards:
  - type: picture-entity
    entity: camera.manometer_heizung_debug_vollbild
    name: Manometer – Erkennung
    camera_view: auto
    show_state: false

  - type: picture-entity
    entity: camera.manometer_heizung_debug_roi
    name: Manometer – Zeigerauswertung
    camera_view: auto
    show_state: false
```

## Relevante Standardoptionen

```yaml
cluster_gauge_offset_x_ratio: 0.73
cluster_gauge_offset_y_ratio: 1.61
cluster_tolerance: 0.32
cluster_min_radius_px: 14
cluster_max_radius_px: 38
cluster_zone_x_min: 0.40
cluster_zone_x_max: 0.68
cluster_zone_y_min: 0.22
cluster_zone_y_max: 0.60
cluster_confirmations: 3
```

Wenn die Kamera absichtlich neu ausgerichtet wurde, `rebaseline_on_start: true`
für genau einen Start setzen und anschließend wieder auf `false` stellen.

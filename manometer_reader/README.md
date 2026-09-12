# Manometer Reader 0.4.1

Home-Assistant-App für die anlagenspezifische Erkennung eines analogen
Heizungsmanometers über RTSP/Tapo, OpenCV und MQTT.

## Neu in 0.4.1

- sucht nicht mehr einen beliebigen Einzelkreis
- erkennt das feste Dreieck aus Thermometer 1, Thermometer 2 und Manometer
- bewertet Abstände, relative Lage und Radien normiert; Verschieben, Zoomen und
  moderate Kameradrehung bleiben möglich
- akzeptiert eine neue Erstkalibrierung erst nach drei übereinstimmenden Bildern
- korrigiert die Skalenwinkel anhand der erkannten Kameradrehung
- zeichnet alle Kreiskandidaten sowie T1, T2 und das gewählte Manometer ins
  Debug-Vollbild
- ignoriert die fehlerhafte Einzelkreis-Kalibrierung aus 0.4.0 automatisch

## Update von 0.4.0

Die App erkennt eine alte `calibration.json` ohne 3er-Cluster und lernt die
Referenz neu. `gauge_template.jpg` wird dabei verworfen. Manuelles Löschen ist
nicht erforderlich.

Nach dem Start erscheinen nacheinander Meldungen wie:

```text
3er-Cluster Kandidat bestätigt 1/3
3er-Cluster Kandidat bestätigt 2/3
3er-Referenz angelegt: T1/T2/Manometer, M=(...)
```

Erst nach der dritten übereinstimmenden Erkennung wird ein Druckwert
veröffentlicht. Bis dahin bleibt der Drucksensor `unavailable`.

## Cluster-Geometrie

Der Standard erwartet diese feste Anordnung (unabhängig von absoluter Position,
Größe und moderater Drehung): T1 und T2 nebeneinander, das Manometer unter T2.

Wichtige Optionen:

- `cluster_confirmations: 3` – Anzahl übereinstimmender Bilder beim Einlernen
- `cluster_vertical_ratio: 0.80` – Abstand T2→M relativ zu T1→T2
- `cluster_tolerance: 0.42` – geometrische Toleranz relativ zum T1/T2-Abstand
- `cluster_prior_radius_px: 400` – großzügige Nähe zur bisherigen manuellen
  Manometerposition; `0` deaktiviert diesen Zusatz
- `cluster_confirmation_px: 18` – maximale Positionsabweichung zwischen
  Bestätigungsbildern

Die vorhandenen `roi_*`, `center_*` und `radius` Werte dienen beim erstmaligen
Suchen nur noch als grobe Positionshilfe. Die tatsächliche ROI stammt aus dem
gefundenen Cluster.

## Debug-Dateien

Im App-Konfigurationsordner unter `/addon_configs/..._manometer_reader/`:

- `calibration.json` – Referenz und letzte Position aller drei Instrumente
- `gauge_template.jpg` – Vorlage des korrekt gewählten Manometers
- `debug/latest_frame.jpg` – alle Kandidaten (grau), T1/T2 (orange) und
  Manometer (grün)
- `debug/latest_roi.jpg` – vergrößerter Manometerausschnitt mit erkanntem Zeiger

## MQTT-Diagnose

Zusätzlich zu den bisherigen Entitäten werden `3er-Cluster Qualität` und
`Kamera Drehung` veröffentlicht. Mögliche Statuswerte sind unter anderem
`cluster_wird_bestaetigt`, `cluster_nicht_gefunden`, `ok`,
`kamera_verschoben_nachgefuehrt` und `zeiger_unsicher`.

Wenn die Kamera absichtlich neu ausgerichtet wurde, `rebaseline_on_start: true`
für genau einen Start setzen und danach wieder auf `false` zurückstellen.

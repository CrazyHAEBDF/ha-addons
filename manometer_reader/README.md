# Manometer Reader 0.4.0

Home-Assistant-App für ein analoges Heizungsmanometer über RTSP/Tapo + OpenCV + MQTT.

## Neu in 0.4.0

- automatische Lokalisierung des Manometers
- automatische Nachführung, wenn die Kamera verschoben wurde
- persistente Referenz in `/config/calibration.json`
- Template des Manometers in `/config/gauge_template.jpg`
- Warnung `Kamera verschoben` per MQTT Discovery
- Diagnose-Entities für Position, Radius, Qualität und Status
- Druck wird bei fehlender/unsicherer Erkennung `unavailable`; echte 0 bar bleiben ein gültiger Messwert
- CPU-schonender Standard: Messung alle 60 s, ffmpeg mit niedriger Priorität und einem Decoder-Thread
- exponentieller Backoff bei RTSP-Fehlern
- Zeigersuche gewichtet den äußeren Radius stärker, damit der lange dünne Messzeiger gegenüber dem kurzen breiten Gegengewicht bevorzugt wird

## Referenz / Autokalibrierung

Beim ersten erfolgreichen Start wird die aktuelle Manometerposition als Referenz gespeichert. Danach sucht die App das gleiche Instrument regelmäßig erneut und folgt dessen neuer Position automatisch.

Wenn die Kamera bewusst dauerhaft neu ausgerichtet wurde, einmal `rebaseline_on_start: true` setzen und die App starten. Danach die Option wieder auf `false` stellen, damit die neue Referenz erhalten bleibt.

## Dateien im App-Konfigurationsordner

- `calibration.json` – gespeicherte Referenz und letzte Position
- `gauge_template.jpg` – visuelle Vorlage für die Nachführung
- `debug/latest_frame.jpg` – Vollbild mit erkannter Position
- `debug/latest_roi.jpg` – vergrößerter Manometerausschnitt mit erkanntem Zeiger

Auf HAOS liegen diese Dateien im jeweiligen App-Ordner unter `/addon_configs/..._manometer_reader/`.

## Wichtige Optionen

- `interval_sec: 60` – Messintervall; für Heizungsdruck reichen 30–60 s normalerweise aus
- `auto_locate_interval_sec: 600` – vollständige Positionsprüfung alle 10 Minuten
- `auto_locate_after_invalid: 2` – bei zwei unsicheren Zeigermessungen sofort neu lokalisieren
- `movement_warning_px: 18` – ab dieser Positionsabweichung wird `Kamera verschoben` aktiv
- `template_match_min: 0.55` – Mindestähnlichkeit zur gespeicherten Vorlage
- `rebaseline_on_start: false` – nur einmal auf true setzen, wenn eine neue Kameraposition als normal akzeptiert werden soll

## MQTT-Entities

- Heizungsdruck
- Manometer erkannt
- Kamera verschoben
- Messung gültig
- Erkennungsqualität
- Manometer X / Y / Radius
- Kamera Verschiebung
- Manometer Status


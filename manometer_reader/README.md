# Manometer Reader

Home-Assistant-Add-on/App zum Ablesen eines analogen Heizungsmanometers über einen RTSP-Stream.

## Standardkalibrierung für das gezeigte Caleffi-Manometer

- Skala: 0–4 bar
- Nullpunkt: ca. 225°
- Endpunkt: ca. 315°
- Drehrichtung mit steigendem Druck: im Uhrzeigersinn
- Erwarteter Heizungsbereich: 1,5–2,5 bar
- Kritischer Bereich: ab 3,0 bar
- `search_max_value` ist absichtlich auf 3,2 bar begrenzt, damit die dünne Gegenseite des Zeigers nicht als Zeigerspitze interpretiert wird.

## Tapo-IR-Screenshot

Für den bereitgestellten 2048x1152-Screenshot sind die Startwerte:

- ROI: x=982, y=502, w=60, h=60
- Mittelpunkt im ROI: x=30, y=30
- Radius: 24

Diese Werte sind Startwerte. Bitte mit `debug/latest_roi.jpg` kontrollieren und ggf. um wenige Pixel anpassen.

## Debugdateien

Bei `debug: true` werden geschrieben:

- `/config/debug/latest_roi.jpg`
- `/config/debug/latest_frame.jpg`

Auf dem HAOS-Host liegen diese im add-on-spezifischen Ordner unter `/addon_configs/..._manometer_reader/debug/`.

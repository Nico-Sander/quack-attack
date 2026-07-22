# Aktuell bekannte Einschränkungen

> Hinweis: Der Fahr-Controller (`control_lane_node.py`) wurde neu geschrieben
> („follow-the-gap"-Regler mit kleiner Zustandsmaschine, siehe
> [`CONTROL_LANE_NODE.md`](CONTROL_LANE_NODE.md)). Die früher hier beschriebenen
> Probleme #2 und #3 (Feststecken vor Enten, Recovery wird nie erreicht) sind durch die
> neue Architektur strukturell adressiert. Dieses Dokument beschreibt den aktuellen Stand.

## Behoben: Bot bleibt vor einem Duckie stehen und erreicht den Recovery-Zustand nie

Der alte Regler fuhr bis vor eine Ente und blieb dann dauerhaft stehen (`v=0, omega=0`),
ohne je in den Recovery-Zustand zu wechseln. Ursache war strukturell: Es gab mehrere
Codepfade, die bewusst `(0,0)` kommandierten, und eine zweifach implementierte
Scan/Return-Recovery, die pro Frame anhand einer verrauschten STOP/nicht-STOP-Klassifikation
ausgewählt wurde.

Lösung in der Neufassung: Es gibt **genau eine** Stelle, die den Fahrbefehl schreibt, und sie
erzwingt die Invariante „nie `v==0 UND omega==0`". Ohne Weg nach vorn dreht sich der Bot
(`ESCAPE_ROTATE`) statt anzuhalten; findet er länger keine Lücke, lockert er schrittweise
seine Anforderungen (`escape_relax_after`), bis sich eine öffnet. Die Never-Freeze-Invariante
ist zusätzlich durch einen Off-ROS-Testharness über alle Zustandszweige abgesichert.

## 1. Linienerkennung bei starker Schrägstellung

Wenn der Duckiebot nach einem Ausweichmanöver sehr schräg oder nahezu rechtwinklig vor einer
Linie steht, kann die Linie im Kamerabild fast horizontal verlaufen und wird dann nicht immer
zuverlässig erkannt.

Abschwächung in der Neufassung:
- Die Linien-Validität wird entprellt (`LANE_HOLD_FRAMES`), bevor eine Seite als offen gilt —
  ein einzelner Flacker-Frame hebt die harte Grenze nicht mehr auf.
- Der Zielpunkt wird immer in den Korridor `[linke Linie + lane_margin, rechte Linie - lane_margin]`
  geklemmt, sodass eine sichtbare Linie eine harte Grenze bleibt.

Offen: Wenn **beide** Linien gleichzeitig fast horizontal verlaufen und nicht erkannt werden,
wechselt der Bot in „unbekannte Geometrie" (Kriechen mit U-Kurven-Bias). Das ist sicher gegen
Feststecken, aber die exakte Ausrichtung zur Fahrbahn in dieser Situation ist noch zu testen.

## 2. Rein reaktiv / kein Weltmodell

Der Regler ist bewusst rein reaktiv (Kamera + Zeit, keine Odometrie). Er plant keinen Pfad
und hat kein Gedächtnis über das kurze `DUCKIE_HOLD_TIME`-Fenster hinaus. In einer Situation,
in der die einzige befahrbare Lücke nur durch eine längere, blind gefahrene Kurve erreichbar
wäre, verlässt er sich darauf, dass beim Drehen laufend neue Kamerabilder eine Lücke zeigen.
Für die gegebene Kursgeometrie (breiter Bulb, verstreute Enten) ist das ausreichend; für
deutlich engere Szenarien könnte ein encoder-gestützter, gemessener Wendewinkel nötig werden.
Dafür ist bereits ein sauberer Umbau-Punkt vorgesehen (`heading_bias()`).

## 3. Parameter-Tuning auf der Hardware ausstehend

Die 14 Default-Parameter (siehe [`PARAMETERS.md`](PARAMETERS.md)) sind aus den vom
Hardware-Owner genannten Wertebereichen abgeleitet, aber noch nicht auf dem Bot feinjustiert.
Besonders zu prüfen: `front_slow_ymax`/`front_block_ymax` (ab wann verlangsamen/drehen),
`gap_min_width` (wann eine Lücke als befahrbar gilt) und `omega_rotate` (Drehgeschwindigkeit
beim Suchen).

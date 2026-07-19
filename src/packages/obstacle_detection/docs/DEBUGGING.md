# Debugging und typische Fehlerbilder

Dieses Dokument beschreibt, wo man bei typischen Problemen zuerst suchen sollte.
Für die Fahrlogik siehe [`CONTROL_LANE_NODE.md`](CONTROL_LANE_NODE.md) und die Parameter
in [`PARAMETERS.md`](PARAMETERS.md).

---

## Der Bot fährt nicht los

Prüfen:

```bash
echo $VEHICLE_NAME
rostopic hz /$VEHICLE_NAME/detect/lane
rostopic echo /$VEHICLE_NAME/car_cmd_switch_node/cmd
```

Wenn `/detect/lane` keine Werte liefert, läuft der Lane-Node nicht korrekt oder im falschen
Namespace.

Der Controller sendet Fahrbefehle mit fester Rate (10 Hz), sobald er läuft — unabhängig
davon, ob neue Sensordaten ankommen. Kommt auf `car_cmd_switch_node/cmd` nichts an, läuft
der Node nicht oder das Topic/der Namespace stimmt nicht.

Hinweis: Der Controller kommandiert konstruktionsbedingt **nie** dauerhaft `v=0, omega=0`.
Steht der Bot trotzdem still, ist entweder der Node nicht aktiv, oder der nachgelagerte
`car_cmd_switch_node`/Wheels-Driver nimmt die Befehle nicht an.

---

## Duckie wird als gelbe oder weiße Linie erkannt

Prüfen:

```bash
rostopic echo /$VEHICLE_NAME/detect/lane_borders
```

Wichtig sind `yellow_valid` und `white_valid`. Im Dashboard die `Yellow mask` / `White mask`
prüfen: Die Duckie-Bereiche sollten dort aus der Maske entfernt sein.

---

## Bot reagiert zu spät / zu früh auf Duckies

Der Controller behandelt eine Ente als blockierend, sobald ihr `ymax` im Planungsband liegt
und ihr (verbreitertes) Intervall den Fahrweg schneidet.

Relevante Parameter:

```text
front_slow_ymax      # ab wann verlangsamt wird
front_block_ymax     # ab wann gedreht statt gefahren wird
duckie_margin_base   # seitlicher Grundabstand
duckie_margin_gain   # zusätzlicher Abstand bei naher Ente
```

Reagiert der Bot zu spät: `front_slow_ymax`/`front_block_ymax` senken. Zu früh auf entfernte
Enten: Rausch-Filter `MIN_DUCKIE_WIDTH/HEIGHT` (Konstanten im Node) bzw. `confidence_threshold`
in `detect_obstacle_node.json` erhöhen.

---

## Bot fährt zu knapp an Duckies vorbei

Relevante Parameter:

```text
duckie_margin_base
duckie_margin_gain
gap_min_width
lane_margin
```

Mehr Sicherheitsabstand erzeugt robustere Umfahrungen, kann aber freie Bereiche zu klein
machen (dann öfter `ESCAPE_ROTATE`).

---

## Bot dreht sich viel / findet keine Lücke

Zeigt das Dashboard `state = ESCAPE_ROTATE`, hält der Controller keinen befahrbaren Weg nach
vorn für gegeben und dreht sich, um eine Lücke zu suchen (das ist der beabsichtigte
Recovery-Zustand, kein Feststecken).

Prüfen im Dashboard:

```text
state
reason
blocked_intervals
free_intervals
selected_free_interval
nearest_front_ymax
```

Relevante Parameter:

```text
gap_min_width        # kleiner -> schmalere Lücken werden akzeptiert
omega_rotate         # Drehgeschwindigkeit beim Suchen
escape_min_dwell     # Mindest-Drehzeit vor erneuter Bewertung
escape_relax_after   # ab wann Anforderungen gelockert werden
```

Dreht der Bot dauerhaft, ohne je eine Lücke zu akzeptieren: `gap_min_width` senken oder
`escape_relax_after` verkürzen. Die Relaxation garantiert, dass er sich nie dauerhaft
festdreht, sondern die Anforderungen so lange lockert, bis eine Lücke befahrbar wird.

---

## Bot überfährt eine Linie

Der Zielpunkt wird immer in den Korridor `[linke Linie + lane_margin, rechte Linie - lane_margin]`
geklemmt. Überfährt der Bot dennoch eine Linie, zuerst prüfen, ob die Linie überhaupt erkannt
wird:

```bash
rostopic echo /$VEHICLE_NAME/detect/lane_borders
```

Ist `white_valid`/`yellow_valid` in der Situation dauerhaft `false` (z. B. stark schräge
Linie), behandelt der Controller diese Seite nach `LANE_HOLD_FRAMES` als offen. `lane_margin`
erhöhen für mehr Sicherheitsabstand zu sichtbaren Linien.

---

## Dashboard crasht mit X11/MIT-SHM-Fehler

Starten mit:

```bash
QT_X11_NO_MITSHM=1 rosrun obstacle_detection dashboard_node.py
```

Optional:

```bash
export QT_X11_NO_MITSHM=1
export NO_AT_BRIDGE=1
```

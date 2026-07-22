# control_lane_node.py

`control_lane_node.py` ist der zentrale Fahr-Node. Er wurde von Grund auf neu geschrieben
(„follow-the-gap"-Regler mit kleiner Zustandsmaschine), um das frühere Problem strukturell
zu beseitigen: Der alte Regler blieb vor Enten stehen (`v=0, omega=0`) und erreichte den
Recovery-Zustand oft nie.

## Leitprinzip: nie einfrieren

Es gibt **genau eine Stelle**, die den Fahrbefehl schreibt (das „output guard" am Ende von
`GapPlanner.step()`), und diese erzwingt kompromisslos die Invariante:

```text
der Bot wird NIE mit (v == 0 UND omega == 0) kommandiert
```

Wenn es keinen Weg nach vorn gibt, **dreht** sich der Bot auf der Stelle, um eine Lücke zu
suchen, statt anzuhalten. Findet er länger keine Lücke, lockert er schrittweise seine
Anforderungen (`ESCAPE_ROTATE`-Relaxation), bis sich eine öffnet. Dadurch kann er immer
Fortschritt machen.

## Architektur

Die reine Entscheidungslogik steckt in der Klasse `GapPlanner` und ist bewusst frei von
`rospy`, damit sie ohne ROS mit synthetischen Eingaben getestet werden kann
(`step(now) -> (v, omega, debug)`). Die ROS-Klasse `ControlLaneNode` cached nur die
Sensordaten in den Planner und ruft `step()` mit fester Rate (10 Hz) auf.

Lane-Following liefert die **Route** (inklusive der U-Kurve); die Ausweichschicht verändert
nur die Lenkung und klemmt sie hart in den Linien-Korridor.

### Readiness-Gate beim Start

Der Controller sendet erst dann Fahrbefehle, wenn **alle drei** Sensordaten mindestens
einmal eingetroffen sind (`/detect/lane`, `/detect/lane_borders`, `/detect/duckie_BB`). Bis
dahin steht der Bot (`v=0, omega=0`) und meldet `state = WAITING` ans Dashboard. Das
verhindert, dass er sofort auf Default-/Stale-Daten losfährt, während der YOLO-Node beim Start
noch das Modell lädt. Dies ist der einzige legitime Fall von `(0,0)` — der Regler ist noch
nicht „scharf".

### Eingänge und Ausgänge

Eingänge:

```text
/<VEHICLE_NAME>/detect/lane           Float64, Lane-Error [-1,1] (positiv = Zentrum links)
/<VEHICLE_NAME>/detect/lane_borders   JSON: yellow_x, white_x, yellow_valid, white_valid, valid
/<VEHICLE_NAME>/detect/duckie_BB      JSON: Liste aller Duckie-Bounding-Boxes (normiert 0..1)
```

Ausgänge:

```text
/<VEHICLE_NAME>/car_cmd_switch_node/cmd   Twist2DStamped (v, omega; omega positiv = links)
/<VEHICLE_NAME>/debug/free_path_plan       JSON-Debugausgabe für das Dashboard
```

Konventionen (normierte Bildkoordinaten):

```text
x:            0 = linker Bildrand, 1 = rechter Bildrand
duckie ymax:  größer = näher am Bot (unten im Bild)
lane error:   positiv = Fahrbahnzentrum liegt LINKS
omega:        positiv = nach links drehen
```

## Pro Takt berechnete Signale

1. **Korridor `[L, R]`** aus den Linienpositionen, mit `lane_margin` als hartem Abstand.
   Die Linien-Validität wird **entprellt**: Eine Seite wird erst nach `LANE_HOLD_FRAMES`
   aufeinanderfolgenden ungültigen Frames geöffnet (verhindert, dass ein einzelner
   Flacker-Frame eine harte Grenze in eine offene Seite verwandelt).
   Wichtig: Der Lane-Node meldet Gelb immer als linke und Weiß als rechte Spalte, auch nach
   der U-Kurve. Der Korridor wird daher **reihenfolge-unabhängig** aus den beiden
   Wandkandidaten gebildet (`links = min`, `rechts = max`) — es wird keine Farb-Semantik
   „Gelb = linke Wand" angenommen.
2. **Blockierte Intervalle**: Jede Duckie-Box `[xmin, xmax]` wird um
   `duckie_margin_base + duckie_margin_gain * Nähe` verbreitert, auf den Korridor geklemmt
   und überlappende Intervalle werden zusammengeführt. Eine zuletzt gesehene Ente wird noch
   `DUCKIE_HOLD_TIME` gehalten, damit ein naher Blindflug (Ente verlässt das Sichtfeld)
   sicher bleibt.
3. **Freie Intervalle** = Korridor minus blockierte Intervalle. Das **breiteste** freie
   Intervall (`best_gap`) wird gewählt, bei Gleichstand näher am Ziel.
4. **Front-Nähe** (`front_ymax`) = `ymax` der nächsten Ente, deren Intervall den zentralen
   Front-Streifen um das Ziel überlappt.
5. **Zielspalte `goal_x`**: `lane_target_x` wenn die Linien vertrauenswürdig sind, sonst ein
   fester **U-Kurven-Bias nach links** (`heading_bias()`, der Umbau-Punkt für einen späteren
   encoder-gestützten Turn).

## Zustandsmaschine

Eine einzige Funktion (`step()`) besitzt alle Übergänge, getrieben von skalaren Signalen und
Mindest-Verweilzeiten.

| Zustand | v | omega | Kurzbeschreibung |
|---|---|---|---|
| **CRUISE** | `v_cruise` | `k_steer*(0.5 - goal_x)*2` | Freie Fahrbahn, Lane-Following. |
| **AVOID** | Rampe `v_avoid → v_min` nach Nähe (bzw. `v_min` bei unbekannter Geometrie) | Lenkung auf `target_x` (Ziel, in die Lücke geklemmt) | Ente beeinflusst den Fahrweg; um sie herumlenken. |
| **ESCAPE_ROTATE** | `0.0` | `escape_dir * omega_rotate` | Kein Weg nach vorn: auf der Stelle zur freieren Seite drehen. **Immer omega ≠ 0.** |

Übergänge (vereinfacht):

```text
kein befahrbarer Weg voraus (keine Lücke ODER Front blockiert)  -> ESCAPE_ROTATE
Ziel blockiert / Front nah / unbekannte Geometrie               -> AVOID
sonst (Fahrbahn frei, nach kurzer Halte-Zeit)                   -> CRUISE
```

Details:
- **`ESCAPE_ROTATE`-Richtung**: zur Seite mit mehr freier Fläche; bei nur einer sichtbaren
  Linie zur offenen Seite; bei Gleichstand nach links (U-Kurve).
- **Mindest-Verweilzeit** (`escape_min_dwell`): verhindert Zittern zwischen Drehen und Fahren.
- **Relaxation** (`escape_relax_after`): Nach längerem erfolglosen Drehen werden
  `gap_min_width` und die Duckie-Verbreiterung schrittweise verkleinert, bis eine Lücke
  befahrbar wird — die formale Garantie gegen dauerhaftes Feststecken.
- **Ziel-Hysterese**: kleines Links/Rechts-Umschalten zwischen ähnlichen Lücken wird gedämpft.
- **Clear-Hold** (`CLEAR_HOLD_TIME`): AVOID fällt erst nach kurzzeitig durchgehend freier
  Fahrbahn zurück auf CRUISE, damit ein einzelner „alles frei"-Frame mitten im Manöver nicht
  sofort zurückregelt.

## Output Guard (einziger Schreiber)

```text
omega = clamp(omega, -omega_max, +omega_max)
target_x ist bereits in [L, R] geklemmt  -> keine Linie wird überfahren
falls |v| ~ 0 UND |omega| ~ 0:  omega = escape_dir * omega_rotate   # never-freeze
publish(v, omega)
```

## Wichtige Fehlerfälle und ihre Behandlung

| Fehlerfall | Behandlung |
|---|---|
| Nahe Ente verlässt das enge Sichtfeld (Blindflug) | Memory-Hold der Box + langsame Annäherung + Ziel-Bias weg von der zuletzt gesehenen Seite. |
| Offener Bulb, beide Linien ungültig, Lane-Error ≈ 0 | Lane-Error wird nicht als „freie Straße" vertraut: AVOID mit U-Kurven-Bias bei `v_min`. |
| Linien-Flackern | Entprellung (`LANE_HOLD_FRAMES`), bevor eine Seite geöffnet wird. |
| Links/Rechts-Flackern | Richtungs-Hysterese + Mindest-Verweilzeit im ESCAPE. |
| Rückweg der U-Kurve (Wände vertauscht) | Reihenfolge-unabhängiger Korridor, keine Farb-Semantik. |

## Debug-Ausgabe

Der Controller publiziert bei jedem Takt ein JSON auf `/<VEHICLE_NAME>/debug/free_path_plan`,
das vom Dashboard visualisiert wird (Felder u. a. `state`, `reason`, `lane_left`,
`lane_right`, `blocked_intervals`, `free_intervals`, `selected_free_interval`, `target_x`,
`nearest_front_ymax`, `v`, `omega`).

## Testen ohne ROS

Weil `GapPlanner` `rospy`-frei ist, kann die Entscheidungslogik mit einem kleinen Harness
getrieben werden, der synthetische Signale einspeist (freie Fahrbahn, Ente links/rechts,
Ente direkt voraus, drei Enten im Bulb, beide Linien ungültig, Relaxation). Jeder Zweig wird
gegen die Never-Freeze-Invariante geprüft.

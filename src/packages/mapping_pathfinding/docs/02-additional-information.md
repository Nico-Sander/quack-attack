# Zusätzliche Informationen

- Für die “Tore” gibt es Apriltags mit den ids 5-13.
> **Meine Interpretation:** Kreuzungen sind Knoten, Straßen evtl. mit Gates sind Kanten

- Der Graph wird  nicht gerichtet sein, dh eine Spur und die Gegenspur haben den gleichen Tag.
> **Meine Interpretation:** Ein Gate, definiert über seine ID überspannt die gesamte Straße -> Eine Kante zwischen jedem Knoten, evtl. mit Gate.

- Ein Beispiel des Graphen findet ihr im Einführungsfoliensatz (`01-information.pdf`)

- zum Graphen: die Nummerierung der Einmündungen an den Kreuzungen ist konstant. Dh: 1 und 3 sind gegenüber, 2 ist immer rechts von 1,…. (Siehe Bild Folie 10, `01-information.pdf`)

- Apriltags 1-4 gibt es weiterhin an den Kreuzungen.
> Über diese kann identifiziert werden, in welche Richtungen an einer Kreuzung abgebogen werden kann. 
> Diese Information lässt sich jedoch auch komplett aus dem vorgegebenen Graphen auslesen.

## Anforderungen
- Bitte macht auf irgendeine Weiße erkenntlich(in einem Dashboard, oder als Ausgabe oder so):
    a) die erstelle Karte
    b) den gewählten Pfad
    c) wo denkt der Duckiebot dass er ist?
    > Startposition ist bekannt / wird vorgegeben, aktuelle Position in Dashboard dann live updaten, je nach dem wie der Duckiebot an jeden Kreuzungen abgebogen ist.

- das Abfahren des Pfades (nicht das Mapping) ist die einzige Challenge die auf Zeit geht. Versucht also einenmöglichst guten Pfad zu finden.

- den Startpunkt des Pfades (sowohl beim “Tore” durchfahren als auch beim Mapping) könnt ihr frei wählen. Er muss nicht der gleiche sein.

- der Pfad durch die “Tore” darf nicht Hard-gecoded werden.

## Weitere Überlegungen
- U-Turns: also das Einfahren in eine Kreuzung bei z.B A_1 und das Herausfahren an der gleichen Einmündung A_1 sollen zunächst nicht zugelassen werden.

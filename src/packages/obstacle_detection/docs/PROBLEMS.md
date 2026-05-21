# Aktuell bekannte Einschränkungen

## 1. Linienerkennung bei starker Schrägstellung

Wenn der Duckiebot nach einem Ausweichmanöver sehr schräg oder nahezu rechtwinklig vor einer gelben oder weißen Linie steht, kann die Linie im Kamerabild fast horizontal verlaufen. In dieser Perspektive wird sie nicht immer zuverlässig als linke oder rechte Fahrbahnbegrenzung erkannt.

Dadurch kann es passieren, dass der Controller die Begrenzung nicht korrekt berücksichtigt und der Duckiebot die Linie überfährt.

Mögliche Verbesserung:
- Zusätzlicher Sicherheitscheck für stark horizontale oder diagonale Linien.
- Separater Recovery-Zustand zur erneuten Ausrichtung zur Fahrbahn.

## 2. Verhalten bei vollständig blockierter Fahrbahn

Wenn der relevante Fahrbereich vollständig durch Duckies blockiert ist, wechselt der Duckiebot in den `blocked_recovery`-Zustand und sucht durch Drehen nach einer freien Lücke.

Falls real keine befahrbare Lücke vorhanden ist, kann der Duckiebot mehrfach hintereinander in den `blocked_recovery`-Zustand wechseln und wiederholt nach einer freien Lücke suchen. In ungünstigen Fällen kann dabei immer wieder eine scheinbar freie, aber nicht sinnvoll befahrbare Richtung gewählt werden.

Mögliche Verbesserung:
- Timeout oder Zähler für aufeinanderfolgende `blocked_recovery`-Versuche.
- Nach mehreren erfolglosen Versuchen Wechsel in einen sicheren Stop-Zustand.

Offen:
- Es ist noch zu prüfen, ob vollständig blockierte Szenarien in der Challenge überhaupt vorkommen.
- Eine zu harte Abbruchbedingung könnte verhindern, dass der Duckiebot schmale, aber noch gültige Lücken findet durch wiederholende suche im `blocked_recovery`-Zustand.
import numpy as np
import networkx as nx
from collections import defaultdict
import matplotlib
matplotlib.use("Agg")  # lokal: "TkAgg"
import matplotlib.pyplot as plt


# Port-Konvention laut Skizze: 1=Ost, 2=Nord, 3=West, 4=Sued
OPPOSITE = {1: 3, 3: 1, 2: 4, 4: 2}
CCW = [1, 2, 3, 4]  # gegen den Uhrzeigersinn


# ---------------------------------------------------------------------------
# EIN Graph: Topologie + Tore (Kanten-Attribute) + Position (Graph-Attribut)
# ---------------------------------------------------------------------------
def build_graph(city):
    G = nx.MultiGraph()
    G.graph["city"] = city
    G.graph["current"] = None          # (fn, fp, tn, tp)
    seen = set()
    for node, ports in city.items():
        G.add_node(node)
        for port, (target, target_port) in ports.items():
            key = frozenset({(node, port), (target, target_port)})
            if key in seen:
                continue
            seen.add(key)
            # gate_tag/gate_color bleiben leer bis zur Laufzeit-Entdeckung
            G.add_edge(node, target, ports={node: port, target: target_port},
                       gate_tag=None, gate_color=None)
    return G


def _find_edge(G, fn, fp, tn, tp):
    """Liefert (u, v, key) der parallelen Kante mit genau diesen Ports."""
    want = {fn: fp, tn: tp}
    for u, v, key, data in G.edges(keys=True, data=True):
        if data["ports"] == want:
            return u, v, key
    return None


def record_gate(G, tag_id, fn, fp, tn, tp, color=None):
    """Tor auf die port-genaue Kante stempeln (idempotent)."""
    e = _find_edge(G, fn, fp, tn, tp)
    if e is None:
        return False
    u, v, key = e
    if G[u][v][key]["gate_tag"] is None:
        G[u][v][key]["gate_tag"] = tag_id
        G[u][v][key]["gate_color"] = color
    return True


def edge_of_tag(G, tag_id):
    """tag_id -> (u, v, key) oder None."""
    for u, v, key, data in G.edges(keys=True, data=True):
        if data.get("gate_tag") == tag_id:
            return u, v, key
    return None


def reset_gates(G):
    """Alle Tore loeschen (neuer Durchlauf)."""
    for u, v, key in G.edges(keys=True):
        G[u][v][key]["gate_tag"] = None
        G[u][v][key]["gate_color"] = None


# --- Position direkt im Graphen ---
def start_on_edge(G, fn, fp, tn, tp):
    G.graph["current"] = (fn, fp, tn, tp)


def current_edge(G):
    return G.graph["current"]


def exit_port(entry_port, turn):
    heading = OPPOSITE[entry_port]
    if turn == "STRAIGHT":
        return heading
    i = CCW.index(heading)
    if turn == "LEFT":
        return CCW[(i + 1) % 4]
    if turn == "RIGHT":
        return CCW[(i - 1) % 4]
    return heading


def turn_at_node(G, turn):
    """Am Zielknoten der aktuellen Kante abbiegen -> Position aktualisieren."""
    cur = G.graph["current"]
    if cur is None:
        return None
    _, _, node, entry_port = cur
    out_port = exit_port(entry_port, turn)
    nbr = G.graph["city"].get(node, {}).get(out_port)
    if nbr is None:
        return None
    tn, tp = nbr
    G.graph["current"] = (node, out_port, tn, tp)
    return G.graph["current"]


# ---------------------------------------------------------------------------
# Visualisierung - liest ALLES aus dem Graphen
# ---------------------------------------------------------------------------
def bezier_point(p0, p1, t, rad):
    p0, p1 = np.array(p0, float), np.array(p1, float)
    m = (p0 + p1) / 2.0
    d = p1 - p0
    c = m + rad * np.array([d[1], -d[0]])
    return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * c + t ** 2 * p1


def curvature(idx, count, spread=0.5):
    return 0.0 if count == 1 else -spread / 2 + spread * idx / (count - 1)


def draw(G, pos, title="Live-Karte", ax=None):
    own_ax = ax is None
    if own_ax:
        fig, ax = plt.subplots(figsize=(8, 6))
    ax.clear()

    nx.draw_networkx_nodes(G, pos, node_color="#4C9BE8", node_size=1300, ax=ax)
    nx.draw_networkx_labels(G, pos, font_size=15, font_color="white",
                            font_weight="bold", ax=ax)

    cur = G.graph.get("current")
    cur_pp = {cur[0]: cur[1], cur[2]: cur[3]} if cur else None

    groups = defaultdict(list)
    for u, v, key in G.edges(keys=True):
        groups[frozenset((u, v))].append((u, v, key))

    for pair, edges in groups.items():
        count = len(edges)
        for idx, (u, v, key) in enumerate(edges):
            rad = curvature(idx, count)
            data = G[u][v][key]
            eports = data["ports"]
            hl = (cur_pp is not None and eports == cur_pp)

            nx.draw_networkx_edges(G, pos, edgelist=[(u, v)], ax=ax,
                                   connectionstyle=f"arc3,rad={rad}",
                                   width=4 if hl else 2,
                                   edge_color="#E8483C" if hl else "#BBBBBB")

            for tt, port in [(0.15, eports[u]), (0.85, eports[v])]:
                px, py = bezier_point(pos[u], pos[v], tt, rad)
                ax.text(px, py, str(port), fontsize=9, color="#B5179E",
                        ha="center", va="center", fontweight="bold", zorder=6,
                        bbox=dict(boxstyle="circle,pad=0.2", fc="white",
                                  ec="#B5179E", alpha=0.95))

            if data["gate_tag"] is not None:
                mx, my = bezier_point(pos[u], pos[v], 0.5, rad)
                ax.scatter([mx], [my], s=150, c=data["gate_color"] or "#333333",
                           edgecolors="black", zorder=7, marker="s")
                ax.text(mx, my + 0.06, f"#{data['gate_tag']}", fontsize=8,
                        ha="center", va="bottom", zorder=7, color="#222222")

    if cur:
        fn, fp, tn, tp = cur
        rx, ry = bezier_point(pos[fn], pos[tn], 0.45, 0.0)
        ax.scatter([rx], [ry], s=260, c="#FFD400", edgecolors="black",
                   zorder=8, marker="o")
        hx, hy = bezier_point(pos[fn], pos[tn], 0.57, 0.0)
        ax.annotate("", xy=(hx, hy), xytext=(rx, ry), zorder=8,
                    arrowprops=dict(arrowstyle="-|>", color="#FF8C00", lw=2.5))

    ax.set_title(title, fontsize=13)
    ax.axis("off")
    if own_ax:
        plt.tight_layout()
    return ax


# ---------------------------------------------------------------------------
# Demo
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    city = {
        "A": {1: ("B", 1), 2: ("C", 2), 3: ("C", 1), 4: ("B", 2)},
        "B": {1: ("A", 1), 2: ("A", 4), 3: ("C", 4)},
        "C": {1: ("A", 3), 2: ("A", 2), 4: ("B", 3)},
    }
    G = build_graph(city)
    pos = nx.spring_layout(G, seed=42)

    start_on_edge(G, "A", 1, "B", 1)
    record_gate(G, 7, "A", 1, "B", 1, color="red")     # auf aktueller Kante
    record_gate(G, 3, "A", 2, "C", 2, color="blue")
    record_gate(G, 9, "B", 3, "C", 4, color="green")

    print("Tag 7 ->", edge_of_tag(G, 7))
    print("Position:", current_edge(G))

    draw(G, pos, title="Ein Graph: Position + Tore (Tag 7 -> A1->B1)")
    plt.savefig("/home/claude/graph_only.png", dpi=150, bbox_inches="tight")
    print("ok")
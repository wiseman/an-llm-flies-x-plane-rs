# /// script
# requires-python = ">=3.11"
# dependencies = ["duckdb"]
# ///
"""How often does a taxi network need runway pavement to get between
taxiway sub-networks? For each airport: union-find over non-runway 1202
edges; a runway edge whose endpoints land in different non-runway
components (each endpoint touching a taxiway edge) is a bridge. Short
bridges (< 60 m) look like 'jog across the runway' crossings; long ones
are backtaxi-only layouts."""
import sys, math, duckdb
from collections import defaultdict
d = sys.argv[1]
con = duckdb.connect()
nodes = {}
for a, n, lat, lon in con.execute(f"SELECT airport_ident,node_id,latitude_deg,longitude_deg FROM '{d}/taxi_nodes.parquet'").fetchall():
    nodes[(a, n)] = (lat, lon)
edges = defaultdict(list)
for a, f, t, cat, name in con.execute(f"SELECT airport_ident,from_node,to_node,category,name FROM '{d}/taxi_edges.parquet'").fetchall():
    edges[a].append((f, t, cat, name))
def hav(p, q):
    R = 6371000.0
    la1, lo1, la2, lo2 = map(math.radians, (*p, *q))
    h = math.sin((la2-la1)/2)**2 + math.cos(la1)*math.cos(la2)*math.sin((lo2-lo1)/2)**2
    return 2*R*math.asin(math.sqrt(h))
n_with_rwy = 0; short_air = []; long_air = 0
for a, es in edges.items():
    rwy = [e for e in es if e[2] == 'runway']
    if not rwy: continue
    n_with_rwy += 1
    parent = {}
    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x
    touched = set()
    for f, t, cat, _ in es:
        if cat != 'runway':
            touched.add(f); touched.add(t)
            parent[find(f)] = find(t)
    shorts = []; longs = 0
    for f, t, _, name in rwy:
        if f in touched and t in touched and find(f) != find(t):
            L = hav(nodes.get((a, f), (0, 0)), nodes.get((a, t), (0, 0)))
            if L < 60:
                pf, pt = nodes[(a, f)], nodes[(a, t)]
                def side(p):
                    # sign of cross product of (pt-pf) x (p-pf), flat-earth
                    ax, ay = (pt[1]-pf[1])*math.cos(math.radians(pf[0])), pt[0]-pf[0]
                    bx, by = (p[1]-pf[1])*math.cos(math.radians(pf[0])), p[0]-pf[0]
                    c = ax*by - ay*bx
                    return 0 if abs(c) < 1e-12 else (1 if c > 0 else -1)
                def sides(n):
                    out = set()
                    for f2, t2, c2, _ in es:
                        if c2 == 'runway': continue
                        if f2 == n: out.add(side(nodes[(a, t2)]))
                        if t2 == n: out.add(side(nodes[(a, f2)]))
                    return out - {0}
                sf, st = sides(f), sides(t)
                kind = 'opposite' if (sf and st and not (sf & st)) else 'same/mixed'
                shorts.append((name, round(L), kind))
            longs += L >= 60
    if shorts: short_air.append((a, shorts))
    if longs: long_air += 1
print(f"airports with runway taxi edges: {n_with_rwy}")
print(f"airports with a short (<60 m) runway edge bridging taxiway sub-networks: {len(short_air)}")
print(f"airports with a long runway edge bridging taxiway sub-networks: {long_air}")
opp = [a for a, s in short_air if any(k == 'opposite' for _, _, k in s)]
print(f"  of which a short bridge joins taxiways on opposite sides only (jog to cross): {len(opp)}")
print("  e.g.", opp[:30])

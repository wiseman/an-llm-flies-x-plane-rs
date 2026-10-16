# /// script
# requires-python = ">=3.11"
# dependencies = ["pyyaml"]
# ///
"""Write config/places/*.yaml from geometries pulled once from OpenStreetMap
(via Hexbird, 2026-10-15). Roads are one carriageway, simplified to ~80-100 m;
areas are a center plus a rough radius from their OSM area.

The WKT below is pasted from the OSM queries; rerun after editing:
    uv run scripts/places/build_places.py
"""

import math
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]


def line(wkt):
    pts = re.findall(r"(-?\d+\.\d+) (-?\d+\.\d+)", wkt)
    return [[round(float(lat), 6), round(float(lon), 6)] for lon, lat in pts]


def area_radius(m2):
    return round(math.sqrt(m2 / math.pi) * 3.28084 / 100) * 100


def road(name, aliases, wkt, kind="road"):
    return {"name": name, "kind": kind, "aliases": aliases, "coords": line(wkt)}


def area(name, aliases, lat, lon, m2=None, radius_ft=None):
    return {"name": name, "kind": "area", "aliases": aliases, "center": [lat, lon],
            "radius_ft": radius_ft or max(500, area_radius(m2))}


def point(name, aliases, lat, lon):
    return {"name": name, "kind": "point", "aliases": aliases, "coords": [[lat, lon]]}


SFV = [
    road("CA-118", ["the 118", "118", "118 freeway", "Ronald Reagan Freeway", "Reagan Freeway", "Simi Valley Freeway", "SR-118", "CA 118"],
         "-118.4120956 34.2853304,-118.433973 34.2690483,-118.4499458 34.2661074,-118.4810825 34.2664072,-118.4849884 34.2681216,-118.4955601 34.2764984,-118.5018071 34.2781897,-118.5266275 34.2783107,-118.5369161 34.275021,-118.555045 34.272396,-118.5921062 34.2737213,-118.6071126 34.278005,-118.624858 34.2753741,-118.6336295 34.2697088"),
    road("US-101", ["the 101", "101", "101 freeway", "Ventura Freeway", "the Ventura", "US 101"],
         "-118.3705192 34.1471658,-118.3781987 34.1545595,-118.3971811 34.1545215,-118.4054811 34.1568099,-118.4440733 34.1553633,-118.4501691 34.1574962,-118.467348 34.1599806,-118.4752308 34.1652992,-118.4925594 34.1655769,-118.5042376 34.1712596,-118.5288974 34.1712894,-118.5362109 34.1733535,-118.5600717 34.1737024,-118.5697025 34.1726565,-118.5844651 34.1680663,-118.608425 34.1707907,-118.623139 34.164684"),
    road("I-405", ["the 405", "405", "405 freeway", "San Diego Freeway", "I 405"],
         # Santa Monica / LAX segment, then the Sepulveda Pass and the valley.
         "-118.368287 33.9311751,-118.3692621 33.9609378,-118.3712536 33.967586,-118.3925376 33.9796163,-118.473199 34.0809598,-118.477864 34.0987041,"
         "-118.4774221 34.1045047,-118.4819122 34.1115236,-118.4816627 34.1175272,-118.47441 34.1277434,-118.4713028 34.1368821,-118.4710707 34.144263,-118.4679749 34.1520602,-118.469496 34.1651925,-118.4675449 34.1710681,-118.4681377 34.1767934,-118.4740882 34.1872487,-118.4719871 34.2715486,-118.4673616 34.2891137,-118.4684178 34.2924377,-118.472044 34.2959603"),
    road("I-5", ["the 5", "5 freeway", "Golden State Freeway", "I 5"],
         "-118.290456 34.1568796,-118.2944154 34.1630519,-118.305164 34.1712159,-118.3078246 34.1768246,-118.3172704 34.1850239,-118.3371546 34.1983024,-118.3449354 34.2090517,-118.3576844 34.2206883,-118.3694687 34.2224537,-118.3769772 34.2266927,-118.3879538 34.2267934,-118.3959338 34.2302734,-118.4098755 34.233572,-118.4391882 34.2607893,-118.458389 34.2840933,-118.4647589 34.2884077,-118.4775085 34.3012305,-118.4916445 34.317819,-118.503132 34.3255692,-118.5085203 34.3342023"),
    road("I-210", ["the 210", "210", "210 freeway", "Foothill Freeway", "I 210"],
         "-118.2644361 34.2308901,-118.280334 34.2344403,-118.2997152 34.2350943,-118.3096796 34.2410414,-118.3172387 34.2432487,-118.3208188 34.2463433,-118.3243249 34.2541141,-118.337191 34.2675229,-118.3419739 34.2704996,-118.3780353 34.2745687,-118.3970194 34.2786072,-118.4320212 34.3106454,-118.4489739 34.3231025,-118.462084 34.3239276,-118.47946 34.323016,-118.4886968 34.3171872,-118.4957327 34.3208895"),
    road("CA-170", ["the 170", "170", "Hollywood Freeway", "CA 170"],
         "-118.3748016 34.1509317,-118.3791208 34.156581,-118.3843525 34.1682016,-118.3928099 34.1749933,-118.3958395 34.1803338,-118.4012637 34.1860967,-118.4034491 34.2003081,-118.4110145 34.2245315,-118.4098119 34.2320503,-118.4140217 34.2365924"),
    road("CA-134", ["the 134", "134", "134 freeway", "Ventura Freeway east", "CA 134"],
         "-118.384375 34.1544015,-118.3720644 34.1528062,-118.3440445 34.153987,-118.3332045 34.1523188,-118.3030437 34.1564302,-118.2907994 34.1551302,-118.2869785 34.1532563"),
    # Boulevards controllers use as lines in the valley.
    road("Sherman Way", ["Sherman Way"], "-118.5448017 34.2011285,-118.453223 34.2012119"),
    road("Roscoe Boulevard", ["Roscoe", "Roscoe Blvd"], "-118.5925194 34.2198883,-118.5016758 34.2212394,-118.4949821 34.2202818,-118.4934831 34.2210896,-118.4122663 34.2216742"),
    road("Victory Boulevard", ["Victory", "Victory Blvd"], "-118.5622296 34.1865794,-118.4033542 34.1866882"),
    road("Saticoy Street", ["Saticoy"], "-118.5902803 34.2083174,-118.4946984 34.2084494"),
    road("Nordhoff Street", ["Nordhoff"], "-118.5550459 34.2355023,-118.4754105 34.2355853"),
    road("Devonshire Street", ["Devonshire"], "-118.583405 34.2572519,-118.4739437 34.2573401"),
    road("Rinaldi Street", ["Rinaldi"], "-118.5621217 34.2748554,-118.5413271 34.2751822,-118.5245151 34.2794215,-118.4565807 34.279001,-118.4525864 34.2813115"),
    road("Burbank Boulevard", ["Burbank Blvd"], "-118.4681867 34.1721163,-118.3899795 34.1721597"),
    road("Ventura Boulevard", ["Ventura Blvd"], "-118.5638322 34.1724605,-118.5451738 34.1723683,-118.5159814 34.1617678,-118.4098238 34.1447099,-118.4059065 34.1427696,-118.3930908 34.1436908,-118.380676 34.1395615,-118.3743421 34.141862,-118.3710176 34.1407709"),
    road("Sepulveda Boulevard", ["Sepulveda", "Sepulveda Blvd"], "-118.4687493 34.1487134,-118.4662122 34.152942,-118.4661663 34.2142286,-118.467654 34.221254,-118.4671396 34.2699824"),
    road("Balboa Boulevard", ["Balboa", "Balboa Blvd"], "-118.5026711 34.222761,-118.5017343 34.2899356,-118.5027579 34.2978898,-118.500933 34.3042137,-118.502669 34.3080609,-118.4958089 34.3215543"),
    road("Reseda Boulevard", ["Reseda", "Reseda Blvd"], "-118.5375269 34.1657712,-118.5360066 34.1692911,-118.536102 34.2277125,-118.5364409 34.2760295,-118.533877 34.2844791"),
    road("Woodley Avenue", ["Woodley"], "-118.4836921 34.2191292,-118.4851791 34.224594,-118.484649 34.2791797"),
    road("Hayvenhurst Avenue", ["Hayvenhurst"], "-118.4937993 34.2240123,-118.4934085 34.2792391"),
    road("Laurel Canyon Boulevard", ["Laurel Canyon"], "-118.3971434 34.225739,-118.4580086 34.2841459"),
    road("San Fernando Road", ["San Fernando Road"], "-118.35084 34.2067054,-118.3816087 34.2279368,-118.4363938 34.2787187"),
    road("Foothill Boulevard", ["Foothill"], "-118.3817159 34.2743252,-118.3909287 34.272874,-118.3943724 34.2738911,-118.4282474 34.3056685,-118.4501813 34.3227632,-118.4734042 34.3227902"),
    # Water courses (concrete channels, easy to see from the air).
    road("Los Angeles River", ["the LA River", "LA River", "the river"],
         "-118.6018306 34.1952295,-118.5818462 34.1952129,-118.5701464 34.1906179,-118.5582249 34.1902847,-118.5515562 34.1876842,-118.5362335 34.1901528,-118.5264806 34.1854077,-118.5141536 34.1848212,-118.4775852 34.1706293,-118.4683977 34.1623586,-118.4444937 34.1558143,-118.4328983 34.1549286,-118.4210422 34.148178,-118.4114877 34.1465559,-118.40543 34.1435905,-118.3975182 34.1446909,-118.3829478 34.1405591,-118.3713622 34.1437191,-118.3464068 34.1422306,-118.3321149 34.1476654,-118.324423 34.1533368,-118.298686 34.1567445", kind="line"),
    road("Pacoima Wash", ["the Pacoima Wash", "Pacoima Wash"],
         "-118.4089026 34.302945,-118.4202068 34.2945888,-118.4292649 34.2814499,-118.4356042 34.2754899,-118.4442921 34.2619145,-118.4514566 34.2523794,-118.4590906 34.2324722,-118.4598353 34.2225728,-118.4579265 34.2123034,-118.4488067 34.2059656,-118.4465417 34.2009304,-118.4454006 34.1914159,-118.4400536 34.1907003,-118.4400661 34.1555822", kind="line"),
    road("Tujunga Wash", ["the Tujunga Wash", "Tujunga Wash", "the wash"],
         "-118.3856271 34.2602849,-118.3915412 34.250144,-118.40383 34.2377444,-118.4215444 34.2243993,-118.4272737 34.2046768,-118.4260463 34.1991966,-118.4145592 34.1819484,-118.4116831 34.1607461,-118.391868 34.1498188,-118.3886919 34.1448804", kind="line"),
    road("Bull Creek", ["Bull Creek"],
         "-118.4871709 34.2734242,-118.4872496 34.2679396,-118.4929393 34.2442282,-118.495938 34.239728,-118.4969861 34.2283632,-118.4991296 34.2222862,-118.4960738 34.1982099,-118.4977844 34.1785036", kind="line"),
    road("Aliso Canyon Wash", ["Aliso Wash"],
         "-118.5265233 34.2751328,-118.5300691 34.2544281,-118.5450421 34.2437752,-118.5473358 34.239426,-118.5460651 34.2188077,-118.5440014 34.2155539,-118.5439208 34.1925963,-118.5406701 34.1901036", kind="line"),
    # Areas and landmarks.
    area("VA Sepulveda Ambulatory Care Center", ["the VA", "VA", "VA hospital", "the VA hospital", "Veterans Hospital", "the Veterans Hospital", "Sepulveda VA"], 34.24650, -118.48060, 604662),
    area("Sepulveda Basin", ["the basin", "the flood basin", "Sepulveda Basin Recreation Area", "the Sepulveda Basin"], 34.17669, -118.48882, 7851619),
    point("Sepulveda Dam", [], 34.17357, -118.47020),
    area("Lake Balboa", ["Balboa Lake"], 34.18129, -118.49510, 105906),
    area("Woodley Lakes Golf Course", ["Woodley golf course", "Woodley Park"], 34.18015, -118.48821, 744196),
    area("Van Nuys Golf Course", ["the golf course"], 34.19029, -118.48852, 226533),
    area("Encino Reservoir", ["the Encino Reservoir", "Encino Res"], 34.14478, -118.51410, 522785),
    area("Chatsworth Reservoir", ["the Chatsworth Reservoir", "Chatsworth Res"], 34.23304, -118.62471, 1731920),
    area("Northridge Fashion Center", ["Northridge Mall", "the Northridge Mall", "the mall"], 34.23913, -118.55581, 292828),
    area("Cal State Northridge", ["CSUN", "California State University, Northridge", "the university"], 34.24273, -118.52774, 1308307),
    area("Pierce College", ["Pierce"], 34.18368, -118.58011, 1513758),
    area("Westfield Topanga", ["Topanga mall", "the Topanga mall"], 34.19066, -118.60336, 117858),
    area("Panorama Mall", ["Panorama City mall"], 34.22317, -118.45026, 80836),
    area("Sherman Oaks Fashion Square", ["Westfield Fashion Square", "Fashion Square"], 34.15700, -118.43731, 41717),
    area("Van Norman Reservoir", ["Los Angeles Reservoir", "the Van Norman", "Van Norman Lakes", "the reservoir by the 5"], 34.29844, -118.48337, 687801),
    area("O'Melveny Park", ["O'Melveny"], 34.31319, -118.52112, 2815662),
    area("Olive View Medical Center", ["Olive View", "Olive View hospital"], 34.32610, -118.44832, 377969),
    area("Hansen Dam", ["the Hansen Dam", "Hansen Dam Recreation Area"], 34.26686, -118.38090, 4662086),
    area("Hansen Lake", ["the Hansen Lake"], 34.26822, -118.38300, 152499),
    area("Hansen Spreading Grounds", ["the Hansen spreading grounds"], 34.24980, -118.39568, 545842),
    area("Pacoima Spreading Grounds", ["the spreading grounds", "the Pacoima spreading grounds"], 34.25631, -118.44867, 639146),
    area("Tujunga Spreading Grounds", ["the Tujunga spreading grounds"], 34.22731, -118.41303, 737227),
    area("Lopez Canyon", ["Lopez", "Lopez/Kagel Canyons", "Kagel Canyon"], 34.30354, -118.38694, 9336880),
    area("El Cariso Golf Course", ["El Cariso", "El Cariso Park"], 34.31270, -118.41378, 327197),
    area("Hollywood Burbank Airport", ["Burbank Airport", "Burbank", "KBUR", "Bob Hope Airport"], 34.20010, -118.35899, 1835796),
    point("Whiteman Airport", ["Whiteman", "KWHP"], 34.25934, -118.41347),
    area("Hollywood Sign", ["the Hollywood sign"], 34.13410, -118.32165, radius_ft=800),
    area("Griffith Observatory", ["the observatory", "Griffith Park Observatory", "the Griffith Observatory"], 34.11825, -118.30035, radius_ft=600),
]

SMO = [
    road("Shoreline", ["the shoreline", "the beach", "the coast", "coastline", "Santa Monica shoreline"],
         # Seaward edges of Topanga, Will Rogers, Santa Monica and Venice beaches, NW to SE.
         "-118.5846251 34.0383728,-118.5671518 34.0412977,-118.5564756 34.03776,-118.5479388 34.0390015,-118.5428265 34.0375898,-118.5340653 34.0333833,-118.5268656 34.0309638,-118.5176239 34.0248999,-118.5068926 34.0166766,-118.4981681 34.0088807,-118.4973887 34.0082612,-118.4935948 34.0051945,-118.4840202 33.9952895,-118.4772256 33.9871786,-118.4725656 33.9829968,-118.4624996 33.9712447,-118.4600842 33.9666669,-118.460292 33.9635886",
         kind="line"),
    road("I-10", ["the 10", "10", "10 freeway", "Santa Monica Freeway", "I 10"],
         "-118.4907633 34.0125907,-118.4739536 34.0233206,-118.4350057 34.0311556,-118.4156383 34.0319003,-118.3961448 34.0292015,-118.3795133 34.0370076,-118.3610727 34.0338902,-118.3206795 34.0363866"),
    area("Santa Monica Pier", ["the pier", "the Santa Monica Pier"], 34.00901, -118.49737, radius_ft=600),
    area("Getty Center", ["the Getty", "the new Getty", "Getty"], 34.07800, -118.47400, radius_ft=1000),
    point("Topanga", ["Topanga Canyon", "Topanga village"], 34.08970, -118.60296),
    area("Topanga State Park", [], 34.08675, -118.56220, 46643890),
    point("Pacific Palisades", ["the Palisades"], 34.04806, -118.52647),
    point("Brentwood", [], 34.05214, -118.47407),
    point("Century City", [], 34.05736, -118.41461),
    point("Venice", [], 33.99504, -118.46689),
    area("Marina del Rey", ["the marina", "the Marina"], 33.97465, -118.45083, 1699226),
    area("Clover Park", [], 34.01604, -118.45625, 74917),
]


def write(name, region, airports, places):
    out = {"region": region, "airports": airports, "places": places}
    path = ROOT / "config/places" / f"{name}.yaml"
    header = (f"# {region}: places controllers and pilots use, from OpenStreetMap via Hexbird\n"
              "# (2026-10-15). Generated by scripts/places/build_places.py; edit there.\n")
    path.write_text(header + yaml.safe_dump(out, sort_keys=False, width=200, allow_unicode=True))
    print(path, len(places))


write("san_fernando_valley", "San Fernando Valley", ["KVNY", "KWHP", "KBUR"], SFV)
write("santa_monica", "Santa Monica", ["KSMO"], SMO)

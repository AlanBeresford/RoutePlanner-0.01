from flask import Flask, render_template, request, jsonify
import requests
import re

# Optional: OR-Tools für echte TSP-Optimierung
try:
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2
    HAS_OR_TOOLS = True
except ImportError:
    HAS_OR_TOOLS = False

app = Flask(__name__)

# Kontaktinfo für Nominatim (Pflicht laut Nutzungsbedingungen)
CONTACT_EMAIL = "maxmontana@hotmail.de"

# OpenStreetMap Nominatim Geocoding
NOMINATIM_URL = "https://nominatim.openstreetmap.org/search"

# Öffentlicher OSRM-Routingserver
OSRM_BASE_URL = "https://router.project-osrm.org"

# Fester Start-/Endpunkt (Depot)
START_ADDRESS = "Aleksis-Kivi-Straße 1, 18106 Rostock, Deutschland"
START_COORD = None  # wird beim ersten Aufruf ermittelt und gecached


def normalize_address_string(address: str) -> str:
    """
    Einfache Normalisierung für häufige Schreibweisen:
    - str. / str -> straße
    - strasse -> straße
    """
    a = address
    a = re.sub(r'\bstr\.\b', 'straße', a, flags=re.IGNORECASE)
    a = re.sub(r'\bstr\b', 'straße', a, flags=re.IGNORECASE)
    a = re.sub(r'strasse', 'straße', a, flags=re.IGNORECASE)
    return a


def norm_for_match(s: str) -> str:
    """Normalisierung für Adressvergleiche (Case-insensitive, Leerzeichen vereinheitlichen)."""
    return re.sub(r'\s+', ' ', normalize_address_string(s).strip().lower())


def geocode_address(address):
    """Adresse -> (lat, lon) via Nominatim, mit einfacher Fuzzy-Normalisierung."""
    params = {
        "q": address,
        "format": "json",
        "limit": 1
    }
    headers = {
        "User-Agent": f"UwesRoutenplaner/1.0 ({CONTACT_EMAIL})"
    }

    # 1. Versuch: Originaladresse
    r = requests.get(NOMINATIM_URL, params=params, headers=headers, timeout=10)
    if r.status_code == 429:
        raise ValueError("Nominatim: Zu viele Anfragen (429). Bitte kurz warten.")
    r.raise_for_status()
    data = r.json()

    # Wenn nichts gefunden, mit normalisierter Adresse versuchen
    if not data:
        normalized = normalize_address_string(address)
        if normalized.lower() != address.lower():
            params["q"] = normalized
            r2 = requests.get(NOMINATIM_URL, params=params, headers=headers, timeout=10)
            if r2.status_code == 429:
                raise ValueError("Nominatim: Zu viele Anfragen (429). Bitte kurz warten.")
            r2.raise_for_status()
            data = r2.json()

    if not data:
        raise ValueError(f"Keine Geodaten für Adresse: {address}")

    lat = float(data[0]["lat"])
    lon = float(data[0]["lon"])
    return lat, lon


def ensure_start_coord():
    """Startkoordinaten einmal holen und dann cachen."""
    global START_COORD
    if START_COORD is None:
        START_COORD = geocode_address(START_ADDRESS)
    return START_COORD


def get_osrm_table(coords):
    """Matrix (Zeit & Distanz) von OSRM holen."""
    if len(coords) < 2:
        raise ValueError("Mindestens zwei Koordinaten erforderlich.")

    coord_str = ";".join([f"{lon},{lat}" for lat, lon in coords])
    url = f"{OSRM_BASE_URL}/table/v1/driving/{coord_str}"
    params = {"annotations": "duration,distance"}

    r = requests.get(url, params=params, timeout=20)
    r.raise_for_status()
    data = r.json()

    if "durations" not in data or "distances" not in data:
        raise ValueError("OSRM-Table-Antwort unvollständig.")

    return data["distances"], data["durations"]


def greedy_tsp(distance_matrix, roundtrip=True):
    """Einfacher Greedy-Algorithmus für TSP-Lösung (Index 0 = Depot)."""
    n = len(distance_matrix)
    if n == 0:
        return []

    visited = [False] * n
    path = [0]
    visited[0] = True

    for _ in range(n - 1):
        last = path[-1]
        next_idx = None
        best_dist = float("inf")

        for j in range(n):
            if not visited[j]:
                d = distance_matrix[last][j]
                if d is not None and d < best_dist:
                    best_dist = d
                    next_idx = j

        if next_idx is None:
            break

        visited[next_idx] = True
        path.append(next_idx)

    if roundtrip and path[0] != path[-1]:
        path.append(0)

    return path


def ortools_tsp(distance_matrix, roundtrip=True):
    """Echte TSP-Optimierung mit OR-Tools (Index 0 = Depot)."""
    if not HAS_OR_TOOLS:
        return None

    n = len(distance_matrix)
    if n <= 1:
        return [0]

    data = {}
    data["distance_matrix"] = [
        [int(d if d is not None else 0) for d in row] for row in distance_matrix
    ]
    data["num_vehicles"] = 1
    data["depot"] = 0

    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]),
        data["num_vehicles"],
        data["depot"]
    )
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(5)

    solution = routing.SolveWithParameters(search_parameters)

    if solution is None:
        return None

    index = routing.Start(0)
    route = []
    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        route.append(node)
        index = solution.Value(routing.NextVar(index))

    if roundtrip:
        if route[0] != 0:
            route.insert(0, 0)
        if route[-1] != 0:
            route.append(0)

    return route


def get_osrm_route(coords, order):
    """Route-Geometrie für die berechnete Reihenfolge holen."""
    ordered_coords = [coords[i] for i in order]
    coord_str = ";".join([f"{lon},{lat}" for lat, lon in ordered_coords])

    url = f"{OSRM_BASE_URL}/route/v1/driving/{coord_str}"
    params = {
        "overview": "full",  # ggf. "simplified" für kleinere GeoJSONs
        "geometries": "geojson"
    }

    r = requests.get(url, params=params, timeout=25)
    r.raise_for_status()
    data = r.json()

    if "routes" not in data or not data["routes"]:
        raise ValueError("OSRM-Route-Antwort enthält keine Route.")

    return data["routes"][0]


@app.route("/")
def index():
    return render_template("index.html", start_address=START_ADDRESS)


@app.route("/optimize", methods=["POST"])
def optimize():
    data = request.get_json() or {}
    addresses_raw = data.get("addresses", "")
    fixed_start_raw = (data.get("fixed_start") or "").strip()

    # Variante C: Fester erster Stopp ist Pflicht
    if not fixed_start_raw:
        return jsonify({"error": "Bitte einen festen ersten Stopp angeben."}), 400

    # Ziel-Adressen Zeile für Zeile (Rohliste)
    raw_addresses = [a.strip() for a in addresses_raw.split("\n") if a.strip()]

    # Wenn der feste erste Stopp noch nicht in der Liste vorkommt, automatisch hinzufügen
    norm_fixed = norm_for_match(fixed_start_raw)
    if all(norm_for_match(a) != norm_fixed for a in raw_addresses):
        raw_addresses.insert(0, fixed_start_raw)

    if len(raw_addresses) < 1:
        return jsonify({"error": "Bitte mindestens eine Ziel-Adresse eingeben."}), 400

    # Dubletten erkennen & entfernen (normalisiert)
    seen = set()
    user_addresses = []
    duplicates = []

    for addr in raw_addresses:
        key = norm_for_match(addr)
        if key in seen:
            duplicates.append(addr)
        else:
            seen.add(key)
            user_addresses.append(addr)

    if len(user_addresses) < 1:
        return jsonify({"error": "Nach Entfernen der Dubletten blieb keine Adresse übrig."}), 400

    # Startpunkt holen
    try:
        start_lat, start_lon = ensure_start_coord()
    except Exception as e:
        return jsonify({"error": f"Fehler beim Startpunkt-Geocoding: {e}"}), 500

    # Zielkoordinaten holen mit Adressprüfung
    coords_user = []
    valid_addresses = []
    invalid_addresses = []

    for addr in user_addresses:
        try:
            lat, lon = geocode_address(addr)
            coords_user.append((lat, lon))
            valid_addresses.append(addr)
        except Exception:
            invalid_addresses.append(addr)

    if len(coords_user) == 0:
        return jsonify({
            "error": "Keine gültigen Adressen gefunden.",
            "invalid_addresses": invalid_addresses
        }), 400

    # Gesamtliste (Index 0 = Depot, ab 1 = Kunden)
    coords = [(start_lat, start_lon)] + coords_user
    geocoded_addresses = [START_ADDRESS] + valid_addresses

    # Index des festen ersten Stopps suchen (nie 0)
    norm_map = {norm_for_match(addr): idx for idx, addr in enumerate(geocoded_addresses)}
    fixed_start_idx = norm_map.get(norm_fixed)

    if fixed_start_idx is None or fixed_start_idx == 0:
        return jsonify({"error": "Fester erster Stopp konnte nicht eindeutig zugeordnet werden."}), 400

    # OSRM-Matrix abrufen
    try:
        distances, durations = get_osrm_table(coords)
    except Exception as e:
        return jsonify({"error": f"Fehler bei der OSRM-Matrix: {e}"}), 500

    n_points = len(coords)

    # Spezialfall: Nur Depot + 1 Stopp
    if n_points == 2:
        order = [0, 1, 0]
        solver_used = "greedy"
    else:
        # Wir bauen ein Teilproblem: fester erster Stopp ist "Depot" im Teilproblem.
        # Knotenmenge: [fixed_start_idx] + alle anderen Kunden (1..n-1) außer fixed_start_idx
        sub_nodes = [fixed_start_idx] + [i for i in range(1, n_points) if i != fixed_start_idx]

        # Teil-Distanzmatrix aufbauen
        m = len(sub_nodes)
        sub_distances = [[0] * m for _ in range(m)]
        for i in range(m):
            for j in range(m):
                sub_distances[i][j] = distances[sub_nodes[i]][sub_nodes[j]]

        # TSP nur über das Teilproblem
        use_ortools = HAS_OR_TOOLS and m <= 25
        solver_used = "ortools" if use_ortools else "greedy"

        if use_ortools:
            sub_order = ortools_tsp(sub_distances, roundtrip=True)
            if sub_order is None:
                solver_used = "greedy"
                sub_order = greedy_tsp(sub_distances, roundtrip=True)
        else:
            sub_order = greedy_tsp(sub_distances, roundtrip=True)

        # sub_order sind Indizes in sub_nodes; 0 entspricht fixed_start_idx
        # Wir bauen globale Reihenfolge:
        global_order = [0]  # Depot zuerst
        used = set(global_order)

        for k in sub_order:
            node = sub_nodes[k]
            if node not in used:
                global_order.append(node)
                used.add(node)

        # Am Ende zurück zum Depot
        if global_order[-1] != 0:
            global_order.append(0)

        order = global_order

    # Distanz & Zeit summieren
    total_distance = 0
    total_duration = 0

    for i in range(len(order) - 1):
        a = order[i]
        b = order[i + 1]
        total_distance += distances[a][b]
        total_duration += durations[a][b]

    # OSRM-Route holen
    try:
        route = get_osrm_route(coords, order)
    except Exception as e:
        return jsonify({"error": f"Fehler bei der Routenberechnung: {e}"}), 500

    # Ausgabe formatieren
    ordered_list = []
    for idx, i in enumerate(order):
        ordered_list.append({
            "sequence": idx + 1,
            "address": geocoded_addresses[i],
            "lat": coords[i][0],
            "lon": coords[i][1],
            "is_start": (i == 0)
        })

    return jsonify({
        "start_address": START_ADDRESS,
        "ordered_stops": ordered_list,
        "total_distance_km": round(total_distance / 1000, 2),
        "total_duration_min": round(total_duration / 60, 1),
        "route_geojson": route["geometry"],
        "duplicates": duplicates,
        "invalid_addresses": invalid_addresses,
        "solver": solver_used
    })


if __name__ == "__main__":
    app.run(debug=True)

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

# Fester Start-/Endpunkt
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
    """Fallback: einfacher Greedy-Algorithmus für TSP-Lösung."""
    n = len(distance_matrix)
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
    """Echte TSP-Optimierung mit OR-Tools."""
    if not HAS_OR_TOOLS:
        return None

    n = len(distance_matrix)
    if n <= 1:
        return [0]

    # Datenmodell für OR-Tools
    data = {}
    # int-Matrix in Metern
    data["distance_matrix"] = [
        [int(d if d is not None else 0) for d in row] for row in distance_matrix
    ]
    data["num_vehicles"] = 1
    data["depot"] = 0

    manager = pywrapcp.RoutingIndexManager(len(data["distance_matrix"]),
                                           data["num_vehicles"], data["depot"])
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Suchstrategie
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

    # Route extrahieren
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
        "overview": "full",
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
    data = request.get_json()
    addresses_raw = data.get("addresses", "")

    # Ziel-Adressen Zeile für Zeile (Rohliste)
    raw_addresses = [a.strip() for a in addresses_raw.split("\n") if a.strip()]

    if len(raw_addresses) < 1:
        return jsonify({"error": "Bitte mindestens eine Ziel-Adresse eingeben."}), 400

    # Dubletten erkennen & entfernen (Case-insensitive)
    seen = set()
    user_addresses = []
    duplicates = []

    for addr in raw_addresses:
        key = addr.strip().lower()
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

    # Gesamtliste (Index 0 = Start/Ziel)
    coords = [(start_lat, start_lon)] + coords_user
    geocoded_addresses = [START_ADDRESS] + valid_addresses

        # OSRM-Matrix abrufen
    try:
        distances, durations = get_osrm_table(coords)
    except Exception as e:
        return jsonify({"error": f"Fehler bei der OSRM-Matrix: {e}"}), 500

    # Anzahl Punkte (Start/Ziel + Stopps)
    n_points = len(coords)

    # Reihenfolge berechnen:
    # - Bis zu 24 Stopps (n_points <= 25) -> OR-Tools (TSP)
    # - Darüber -> direkt Greedy-Fallback, damit es stabil bleibt
    solver_used = "greedy"
    order = None

    if HAS_OR_TOOLS and n_points <= 25:
        order = ortools_tsp(distances, roundtrip=True)
        if order is not None:
            solver_used = "ortools"

    if order is None:
        order = greedy_tsp(distances, roundtrip=True)

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


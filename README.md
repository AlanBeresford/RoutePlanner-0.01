Der DRK-Routenoptimierer ist eine Webanwendung zur effizienten Planung von Essen-auf-Rädern-Touren.
Es verarbeitet eine Liste von Kundenadressen, berechnet die optimale Reihenfolge der Stopps und stellt die Route auf einer Karte dar – inklusive realer Straßenführung, Fahrzeiten und Distanzen.

Hauptfunktionen:

Geocoding aller eingegebenen Adressen

automatische Erkennung von Dubletten

Routenoptimierung (TSP)

echte Straßenroute (inkl. Einbahnstraßen)

Depot als fester Start- und Endpunkt

optional fester erster Stopp

optional fester letzter Stopp

interaktive Karte (Leaflet)

Ausgabe der gesamten Stoppliste + Marker mit Nummerierung

🌐 Genutzte externe Dienste
Zweck	Dienst	Art	Warum
Geocoding	Nominatim (OSM)	öffentlich, kostenlos	Adresse → Koordinaten
Routing & Fahrzeiten	OSRM	öffentlich, kostenlos	Realistische Straßenwege
Kartendarstellung	OpenStreetMap + Leaflet	öffentlich, kostenlos	Basiskarte
Optimierung	Google OR-Tools	lokal, Open-Source	TSP-Optimierung
Fallback-Optimierung	Greedy	lokal	schnell & stabil

Keiner dieser Dienste kostet Gebühren.
Es werden keine Google-Maps- oder kommerziellen APIs verwendet.

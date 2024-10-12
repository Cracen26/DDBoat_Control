import folium

coordinates = [
    {"lon": -3.0147, "lat": 48.1993},
    {"lon": -3.0148, "lat": 48.1994},
    {"lon": -3.0148, "lat": 48.1995},
    {"lon": -3.0149, "lat": 48.1996},
    {"lon": -3.0150, "lat": 48.1997},
    {"lon": -3.0151, "lat": 48.1997},
    {"lon": -3.0152, "lat": 48.1997},
    {"lon": -3.0153, "lat": 48.1997},
]

m = folium.Map(location=[coordinates[0]['lat'], coordinates[0]['lon']], zoom_start=15)

for coord in coordinates:
    folium.Marker(
        location=[coord['lat'], coord['lon']],
        popup=f"Lon: {coord['lon']}, Lat: {coord['lat']}",
        icon=folium.Icon(color='blue')
    ).add_to(m)

folium.PolyLine(
    locations=[[coord['lat'], coord['lon']] for coord in coordinates],
    color='red',
    weight=2.5,
    opacity=1
).add_to(m)

m.save("path_map.html")

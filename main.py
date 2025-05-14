import streamlit as st
import osmnx as ox
import networkx as nx
import folium
from streamlit_folium import folium_static
from geopy.geocoders import Nominatim
import time

# Function to fetch road network data
def get_road_network(city):
    try:
        G = ox.graph_from_place(city, network_type="drive")
        return G
    except Exception:
        return None


# Function to convert address to coordinates
def get_coordinates(location):
    geolocator = Nominatim(user_agent="route_planner")
    loc = geolocator.geocode(location)
    return (loc.latitude, loc.longitude) if loc else None

# Function to get the nearest graph node
def get_nearest_node(G, location):
    coordinates = get_coordinates(location)
    if coordinates:
        lat, lon = coordinates
        return ox.distance.nearest_nodes(G, lon, lat)
    return None


# Dijkstra's Algorithm for shortest path
def shortest_path_dijkstra(G, start, end):
    return nx.shortest_path(G, source=start, target=end, weight="length")


# A* Algorithm for shortest path
def shortest_path_astar(G, start, end):
    return nx.astar_path(G, source=start, target=end, weight="length")


# Function to calculate total route distance
def calculate_distance(G, path):
    total_distance = 0
    for i in range(len(path) - 1):
        edge_data = G.get_edge_data(path[i], path[i + 1])
        if edge_data is not None:
            total_distance += edge_data[0]["length"]
    return total_distance / 1000  # Convert meters to kilometers


# Function to plot the route on a map
def plot_route(G, path, start, end):
    route_map = folium.Map(location=get_coordinates(start), zoom_start=14)
    route_coordinates = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in path]
    folium.PolyLine(route_coordinates, color="blue", weight=5, opacity=0.7).add_to(route_map)
    folium.Marker(route_coordinates[0], popup="Start", icon=folium.Icon(color="green")).add_to(route_map)
    folium.Marker(route_coordinates[-1], popup="End", icon=folium.Icon(color="red")).add_to(route_map)
    return route_map


# Streamlit UI
def main():
    st.title("🚗 Route Planner Web App")

    city = st.text_input("Enter the city name (e.g., New York, USA):")
    if city:
        G = get_road_network(city)
        if G:
            start_location = st.text_input("Enter the start location:")
            end_location = st.text_input("Enter the end location:")

            if st.button("Find Routes"):
                start_node = get_nearest_node(G, start_location)
                end_node = get_nearest_node(G, end_location)

                if start_node and end_node:
                    path_dijkstra = shortest_path_dijkstra(G, start_node, end_node)
                    dijkstra_distance = calculate_distance(G, path_dijkstra)

                    path_astar = shortest_path_astar(G, start_node, end_node)
                    astar_distance = calculate_distance(G, path_astar)

                    st.success(f"Dijkstra Distance: {dijkstra_distance:.2f} km")
                    st.success(f"A* Distance: {astar_distance:.2f} km")

                    st.subheader("Dijkstra's Algorithm Route")
                    folium_static(plot_route(G, path_dijkstra, start_location, end_location))

                    st.subheader("A* Algorithm Route")
                    folium_static(plot_route(G, path_astar, start_location, end_location))
                else:
                    st.error("Invalid start or end location. Please try again.")
        else:
            st.error("Could not fetch road network for the given city.")


if __name__ == "__main__":

    main()
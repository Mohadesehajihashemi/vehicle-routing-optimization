import pandas as pd
import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import folium
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from math import radians, sin, cos, sqrt, atan2
import time
import warnings

warnings.filterwarnings('ignore')

# =============================================================================
# DATA LOADING
# =============================================================================

customers = pd.read_csv('customer_info.csv')
vehicles = pd.read_csv('vehicle_info.csv')

num_customers = len(customers) - 1
num_vehicles = int(vehicles['number_of_vehicles'].iloc[0])
vehicle_capacity = int(vehicles['capacity'].iloc[0])

locations = list(zip(customers['Latitude'].tolist(), customers['Longitude'].tolist()))
demands = [int(d) for d in customers['Demand'].tolist()]
service_times = [int(s) for s in customers['ServiceTime'].tolist()]


def parse_time_window(tw_str):
    if tw_str == 0 or pd.isna(tw_str):
        return (0, 1440)
    parts = str(tw_str).split(':')
    if len(parts) == 2:
        start_hour, end_hour = int(parts[0]), int(parts[1])
    elif len(parts) == 4:
        start_hour, end_hour = int(parts[0]), int(parts[2])
    else:
        return (0, 1440)
    return (start_hour * 60, end_hour * 60)


time_windows = [parse_time_window(tw) for tw in customers['TimeWindow']]


# =============================================================================
# DISTANCE CALCULATION
# =============================================================================

def haversine(coord1, coord2):
    R = 6371
    lat1, lon1 = radians(coord1[0]), radians(coord1[1])
    lat2, lon2 = radians(coord2[0]), radians(coord2[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    return R * 2 * atan2(sqrt(a), sqrt(1 - a))


num_locations = int(len(locations))
distance_matrix = np.zeros((num_locations, num_locations))
for i in range(num_locations):
    for j in range(num_locations):
        distance_matrix[i][j] = haversine(locations[i], locations[j])

time_matrix = (distance_matrix / 40 * 60).astype(int).tolist()
distance_matrix_int = (distance_matrix * 1000).astype(int).tolist()

# =============================================================================
# OR-TOOLS MODEL
# =============================================================================

manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, 0)
routing = pywrapcp.RoutingModel(manager)


def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return distance_matrix_int[from_node][to_node]


transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


def demand_callback(from_index):
    from_node = manager.IndexToNode(from_index)
    return demands[from_node]


demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
routing.AddDimensionWithVehicleCapacity(
    demand_callback_index, 0, [vehicle_capacity] * num_vehicles, True, 'Capacity')


def time_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return time_matrix[from_node][to_node] + service_times[from_node]


time_callback_index = routing.RegisterTransitCallback(time_callback)
routing.AddDimension(time_callback_index, 240, 1440, False, 'Time')

time_dimension = routing.GetDimensionOrDie('Time')
for location_idx, time_window in enumerate(time_windows):
    if location_idx == 0:
        continue
    index = manager.NodeToIndex(location_idx)
    time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

for vehicle_id in range(num_vehicles):
    routing.AddVariableMinimizedByFinalizer(
        time_dimension.CumulVar(routing.Start(vehicle_id)))
    routing.AddVariableMinimizedByFinalizer(
        time_dimension.CumulVar(routing.End(vehicle_id)))

fixed_cost = 100000
for vehicle_id in range(num_vehicles):
    routing.SetFixedCostOfVehicle(fixed_cost, vehicle_id)

# =============================================================================
# SOLVER
# =============================================================================

search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
search_parameters.time_limit.FromSeconds(60)

print("=" * 60)
print("VEHICLE ROUTING OPTIMIZATION - OptimeAI Task")
print("=" * 60)
print(f"\nProblem Size:")
print(f"  - Customers: {num_customers}")
print(f"  - Available Vehicles: {num_vehicles}")
print(f"  - Vehicle Capacity: {vehicle_capacity}")
print(f"  - Total Demand: {sum(demands)}")
print(f"  - Minimum Vehicles Needed: {int(np.ceil(sum(demands) / vehicle_capacity))}")
print("\nSolving...")

start_time = time.time()
solution = routing.SolveWithParameters(search_parameters)
solve_time = time.time() - start_time

# =============================================================================
# SOLUTION EXTRACTION
# =============================================================================

routes = []
route_distances = []
route_loads = []
route_times = []

if solution:
    total_distance = 0
    total_load = 0
    vehicles_used = 0

    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        route = []
        route_distance = 0
        route_load = 0

        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route.append(node)
            route_load += demands[node]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

        route.append(0)

        if len(route) > 2:
            vehicles_used += 1
            routes.append(route)
            route_distances.append(route_distance / 1000)
            route_loads.append(route_load)
            total_distance += route_distance
            total_load += route_load

            time_var = time_dimension.CumulVar(routing.Start(vehicle_id))
            start_time_val = solution.Min(time_var)
            time_var = time_dimension.CumulVar(routing.End(vehicle_id))
            end_time_val = solution.Max(time_var)
            route_times.append((start_time_val, end_time_val))

    print("\n" + "=" * 60)
    print("SOLUTION RESULTS")
    print("=" * 60)
    print(f"\nKey Metrics:")
    print(f"  - Vehicles Used: {vehicles_used}")
    print(f"  - Total Distance: {total_distance / 1000:.2f} km")
    print(f"  - Total Load Delivered: {total_load}")
    print(f"  - Computation Time: {solve_time:.2f} seconds")

    print(f"\nRoute Details:")
    for i, (route, dist, load, times) in enumerate(zip(routes, route_distances, route_loads, route_times)):
        customers_in_route = [r for r in route if r != 0]
        print(f"\n  Vehicle {i + 1}:")
        print(f"    - Route: Depot -> {' -> '.join(map(str, customers_in_route))} -> Depot")
        print(f"    - Customers: {len(customers_in_route)}")
        print(f"    - Distance: {dist:.2f} km")
        print(f"    - Load: {load}/{vehicle_capacity}")
        print(f"    - Time: {times[0] // 60:02d}:{times[0] % 60:02d} - {times[1] // 60:02d}:{times[1] % 60:02d}")


# =============================================================================
# SPATIAL OVERLAP CALCULATION
# =============================================================================

def calculate_overlap(routes, locations):
    hulls = []
    for route in routes:
        points = [locations[i] for i in route if i != 0]
        if len(points) >= 3:
            try:
                points_array = np.array(points)
                hull = ConvexHull(points_array)
                hulls.append((points_array, hull))
            except:
                pass

    if len(hulls) < 2:
        return 0

    total_area = sum(h[1].volume for h in hulls)

    all_points = np.vstack([h[0] for h in hulls])
    try:
        combined_hull = ConvexHull(all_points)
        combined_area = combined_hull.volume
    except:
        combined_area = total_area

    overlap = max(0, total_area - combined_area) / total_area if total_area > 0 else 0
    return overlap


if solution:
    overlap = calculate_overlap(routes, locations)
    print(f"\nSpatial Analysis:")
    print(f"  - Region Overlap: {overlap * 100:.2f}%")
    print(f"  - Region Separation: {(1 - overlap) * 100:.2f}%")

# =============================================================================
# VISUALIZATION - INTERACTIVE MAP
# =============================================================================

colors = ['red', 'blue', 'green', 'purple', 'orange', 'darkred', 'darkblue', 'darkgreen', 'cadetblue', 'pink']

depot_location = locations[0]
m = folium.Map(location=[depot_location[0], depot_location[1]], zoom_start=11)

folium.Marker(
    location=depot_location,
    popup='Depot',
    icon=folium.Icon(color='black', icon='home', prefix='fa')
).add_to(m)

for i, route in enumerate(routes):
    color = colors[i % len(colors)]
    route_coords = [locations[node] for node in route]

    folium.PolyLine(
        locations=route_coords,
        weight=3,
        color=color,
        opacity=0.8
    ).add_to(m)

    for node in route:
        if node != 0:
            tw = time_windows[node]
            folium.CircleMarker(
                location=locations[node],
                radius=6,
                popup=f'Customer {node}<br>TW: {tw[0] // 60:02d}:00-{tw[1] // 60:02d}:00',
                color=color,
                fill=True,
                fillColor=color,
                fillOpacity=0.7
            ).add_to(m)

    points = [locations[node] for node in route if node != 0]
    if len(points) >= 3:
        try:
            points_array = np.array(points)
            hull = ConvexHull(points_array)
            hull_points = [points_array[i].tolist() for i in hull.vertices]
            hull_points.append(hull_points[0])
            folium.Polygon(
                locations=hull_points,
                color=color,
                weight=2,
                fill=True,
                fillColor=color,
                fillOpacity=0.1
            ).add_to(m)
        except:
            pass

legend_html = '''
<div style="position: fixed; bottom: 50px; left: 50px; z-index: 1000; background-color: white;
     padding: 10px; border: 2px solid grey; border-radius: 5px;">
<h4 style="margin: 0 0 10px 0;">Routes Legend</h4>
'''
for i in range(len(routes)):
    legend_html += f'<p style="margin: 2px;"><span style="color:{colors[i % len(colors)]};">●</span> Vehicle {i + 1}</p>'
legend_html += '</div>'
m.get_root().html.add_child(folium.Element(legend_html))

m.save('route_map.html')
print(f"\nVisualization saved: route_map.html")

# =============================================================================
# VISUALIZATION - STATIC PLOT
# =============================================================================

fig, axes = plt.subplots(1, 2, figsize=(16, 7))

ax1 = axes[0]
ax1.scatter(locations[0][1], locations[0][0], c='black', s=200, marker='s', label='Depot', zorder=5)

for i, route in enumerate(routes):
    color = colors[i % len(colors)]
    route_lons = [locations[node][1] for node in route]
    route_lats = [locations[node][0] for node in route]
    ax1.plot(route_lons, route_lats, c=color, linewidth=2, alpha=0.7)

    for node in route:
        if node != 0:
            ax1.scatter(locations[node][1], locations[node][0], c=color, s=50, zorder=3)

    points = [locations[node] for node in route if node != 0]
    if len(points) >= 3:
        try:
            points_array = np.array(points)
            hull = ConvexHull(points_array)
            hull_lons = [points_array[v][1] for v in hull.vertices] + [points_array[hull.vertices[0]][1]]
            hull_lats = [points_array[v][0] for v in hull.vertices] + [points_array[hull.vertices[0]][0]]
            ax1.fill(hull_lons, hull_lats, color=color, alpha=0.1)
        except:
            pass

ax1.set_xlabel('Longitude')
ax1.set_ylabel('Latitude')
ax1.set_title('Optimized Vehicle Routes with Service Regions')
ax1.legend([f'Vehicle {i + 1}' for i in range(len(routes))], loc='upper left')
ax1.grid(True, alpha=0.3)

ax2 = axes[1]
metrics = ['Vehicles\nUsed', 'Total Distance\n(km)', 'Avg Distance\nper Vehicle (km)', 'Avg Customers\nper Vehicle']
values = [
    vehicles_used,
    total_distance / 1000,
    (total_distance / 1000) / vehicles_used,
    num_customers / vehicles_used
]

bars = ax2.bar(metrics, values, color=['#2ecc71', '#3498db', '#9b59b6', '#e74c3c'])
ax2.set_ylabel('Value')
ax2.set_title('Solution Performance Metrics')

for bar, val in zip(bars, values):
    ax2.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
             f'{val:.1f}', ha='center', va='bottom', fontsize=10)

plt.tight_layout()
plt.savefig('route_analysis.png', dpi=150, bbox_inches='tight')
print(f"Analysis plot saved: route_analysis.png")

# =============================================================================
# EXPORT RESULTS
# =============================================================================

results_data = []
for i, (route, dist, load, times) in enumerate(zip(routes, route_distances, route_loads, route_times)):
    for j, node in enumerate(route):
        if node != 0:
            results_data.append({
                'Vehicle': i + 1,
                'Stop_Order': j,
                'Customer_ID': node,
                'Latitude': locations[node][0],
                'Longitude': locations[node][1],
                'Time_Window': f"{time_windows[node][0] // 60:02d}:00-{time_windows[node][1] // 60:02d}:00"
            })

results_df = pd.DataFrame(results_data)
results_df.to_csv('optimized_routes.csv', index=False)
print(f"Route details saved: optimized_routes.csv")

# =============================================================================
# SUMMARY REPORT
# =============================================================================

print("\n" + "=" * 60)
print("OPTIMIZATION SUMMARY")
print("=" * 60)
print(f"""
Objective Achievement:
  1. Minimizing Vehicles: {vehicles_used} vehicles used (minimum possible: {int(np.ceil(sum(demands) / vehicle_capacity))})
  2. Minimizing Overlap: {(1 - overlap) * 100:.1f}% region separation achieved
  3. Minimizing Distance: {total_distance / 1000:.2f} km total travel distance

Output Files:
  - route_map.html: Interactive map with routes and service regions
  - route_analysis.png: Static visualization and metrics
  - optimized_routes.csv: Detailed route assignments

Algorithm: Guided Local Search with OR-Tools
Computation Time: {solve_time:.2f} seconds
""")

if not solution:
    print("\nWARNING: No feasible solution found. Consider relaxing constraints.")

plt.show()
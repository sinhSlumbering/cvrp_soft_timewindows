#!/usr/bin/env python3
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
TOTAL_DEPO_TIME = 15
MAX_WAIT_TIME = 2009
PENALTY = 10
def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['time_matrix'] = [
        [0, 6, 9, 8, 7, 3, 6, 2, 3, 2],
        [6, 0, 8, 3, 2, 6, 8, 4, 8, 8],
        [9, 8, 0, 11, 10, 6, 3, 9, 5, 8],
        [8, 3, 11, 0, 1, 7, 10, 6, 10, 10],
        [7, 2, 10, 1, 0, 6, 9, 4, 8, 9],
        [3, 6, 6, 7, 6, 0, 2, 3, 2, 2],
        [6, 8, 3, 10, 9, 2, 0, 6, 2, 5],
        [2, 4, 9, 6, 4, 3, 6, 0, 4, 4],
        [3, 8, 5, 10, 8, 2, 2, 4, 0, 3],
        [2, 8, 8, 10, 9, 2, 5, 4, 3, 0],
    ]

    data['time_windows'] = [
        (0, 5),  # depot
        (7, 12),  # 1
        (10, 15),  # 2
        (16, 18),  # 3
        (10, 13),  # 4
        (0, 5),  # 5
        (5, 10),  # 6
        (0, 4),  # 7
        (5, 10),  # 8
        (0, 3),  # 9
    ]

    data['distance_matrix'] = [
        [
            0, 548, 776, 696, 582, 274, 502, 194, 308, 194],
        [
            548, 0, 684, 308, 194, 502, 730, 354, 696, 742
        ],
        [
            776, 684, 0, 992, 878, 502, 274, 810, 468, 742
        ],
        [
            696, 308, 992, 0, 114, 650, 878, 502, 844, 890
        ],
        [
            582, 194, 878, 114, 0, 536, 764, 388, 730, 776
        ],
        [
            274, 502, 502, 650, 536, 0, 228, 308, 194, 240
        ],
        [
            502, 730, 274, 878, 764, 228, 0, 536, 194, 468
        ],
        [
            194, 354, 810, 502, 388, 308, 536, 0, 342, 388
        ],
        [
            308, 696, 468, 844, 730, 194, 194, 342, 0, 274
        ],
        [
            194, 742, 742, 890, 776, 240, 468, 388, 274, 0
        ],
    ]

    # data['pickups_deliveries'] = [
    #     [1, 3],
    #     [2, 6],
    #     [4, 7],
    #     [5, 8],
    #  ]

    data['num_vehicles'] = 4
    data['vehicle_capacities'] = [15, 15, 15, 15]
    # data['demands'] = [0, 1, 1, 2, 4, 2, 4, 8, 8, 1]
    data['demands'] = [0, 1, 1, -1, 4, 2, -1, -4, -2, 0]
    data['depot'] = 0
    assert data['num_vehicles'] == len(data['vehicle_capacities'])
    assert len(data['time_matrix']) == len(data['distance_matrix'])
    assert len(data['time_matrix']) == len(data['time_windows'])
    assert len(data['time_matrix']) == len(data['demands'])
    return data


def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    # Display dropped nodes.
    dropped_nodes = 'Dropped nodes:'
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += ' {}'.format(manager.IndexToNode(node))
    print(dropped_nodes)
    # Print routes
    time_dimension = routing.GetDimensionOrDie('Time')
    distance_dimension = routing.GetDimensionOrDie('Distance')
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    total_time = 0
    total_distance = 0
    total_load=0
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.Start(vehicle_id)
        plan_output = f'Route for vehicle {vehicle_id}:\n'
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            time_var = time_dimension.CumulVar(index)
            distance_var = distance_dimension.CumulVar(index)
            capacity_var = capacity_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2}) Distance:{3} Load:{4}  -> '.format(
                node_index,
                solution.Min(time_var), solution.Max(time_var),
                solution.Value(distance_var),
                solution.Value(capacity_var))
            index = solution.Value(routing.NextVar(index))
        node_index = manager.IndexToNode(index)
        time_var = time_dimension.CumulVar(index)
        distance_var = distance_dimension.CumulVar(index)
        capacity_var = capacity_dimension.CumulVar(index)
        plan_output += '{0} Time({1},{2}) Distance:{3}\n'.format(
            manager.IndexToNode(index),
            solution.Min(time_var), solution.Max(time_var),
            solution.Value(distance_var))
        plan_output += 'Time of the route: {}min\n'.format(solution.Min(time_var))
        plan_output += 'Distance of the route: {}m\n'.format(solution.Value(distance_var))
        print(plan_output)
        total_time += solution.Min(time_var)
        total_distance += solution.Value(distance_var)
        total_load += solution.Value(capacity_var)
    print('Total time of all routes: {}min'.format(total_time))
    print('Total distance of all routes: {}m'.format(total_distance))


def main():
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
            len(data['distance_matrix']),
            data['num_vehicles'],
            data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Allow to drop nodes.
    penalty = 1_000_000
    for node in range(1, len(data['distance_matrix'])):
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    distance_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(distance_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        distance_callback_index,
        0,  # no slack
        10_000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Add Time Windows constraint.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    time_callback_index = routing.RegisterTransitCallback(time_callback)

    time = 'Time'
    routing.AddDimension(
        time_callback_index,
        MAX_WAIT_TIME,  # allow waiting time
        TOTAL_DEPO_TIME,  # maximum time per vehicle HOW TF DID MY DUMB ASS MISS THIS
        False,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.SetCumulVarSoftUpperBound(index, time_window[1], PENALTY)
        time_dimension.SetCumulVarSoftLowerBound(index, time_window[0], PENALTY)

    # Add time window constraints for each vehicle start node.
    depot_idx = data['depot']
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        # Redundant now I added total time per vehicle earlier
        time_dimension.SetSpanUpperBoundForVehicle(TOTAL_DEPO_TIME, vehicle_id)


    # Instantiate route start and end times to produce feasible times.
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    #search_parameters.log_search = True
    search_parameters.time_limit.FromSeconds(5)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution)
    else:
        print('no solution found')

if __name__ == '__main__':
    main() 

import math
from ortools.constraint_solver import pywrapcp
# You need to import routing_enums_pb2 after pywrapcp!
from ortools.constraint_solver import routing_enums_pb2
import googlemaps

gmaps = googlemaps.Client(key='<Your API Key>')
# the locations to be visited by the travelling sales man
locations = [[39.237798, 9.101240],
             [39.224730, 9.127230],
             [40.438650, 9.029527],
             [39.209690, 9.128113],
             [39.237798, 9.101240],
             [39.234753, 9.111678]]
# Cities
city_names = []
for location in locations:
    city_names.append(gmaps.reverse_geocode((location[0], location[1]))[0][u'address_components'][1][u'long_name'])

tsp_size = len(city_names)


def distance(lat1, long1, lat2, long2):
    # Note: The formula used in this function is not exact, as it assumes
    # the Earth is a perfect sphere.

    # Mean radius of Earth in miles
    radius_earth = 3959

    # Convert latitude and longitude to
    # spherical coordinates in radians.
    degrees_to_radians = math.pi / 180.0
    phi1 = lat1 * degrees_to_radians
    phi2 = lat2 * degrees_to_radians
    lambda1 = long1 * degrees_to_radians
    lambda2 = long2 * degrees_to_radians
    dphi = phi2 - phi1
    dlambda = lambda2 - lambda1

    a = haversine(dphi) + math.cos(phi1) * math.cos(phi2) * haversine(dlambda)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = radius_earth * c
    return d


def haversine(angle):
    h = math.sin(angle / 2) ** 2
    return h


# Distance callback

class CreateDistanceCallback(object):
    """Create callback to calculate distances between points."""

    def __init__(self):

        # Latitudes and longitudes of selected U.S. cities

        """Create the distance matrix."""
        size = len(locations)
        self.matrix = {}

        for from_node in xrange(size):
            self.matrix[from_node] = {}
            for to_node in xrange(size):
                if from_node == to_node:
                    self.matrix[from_node][to_node] = 0
                else:
                    x1 = locations[from_node][0]
                    y1 = locations[from_node][1]
                    x2 = locations[to_node][0]
                    y2 = locations[to_node][1]
                    self.matrix[from_node][to_node] = distance(x1, y1, x2, y2)

    def Distance(self, from_node, to_node):
        return self.matrix[from_node][to_node]


def main():
    # Create routing model
    if tsp_size > 0:
        # TSP of size args.tsp_size
        # Second argument = 1 to build a single tour (it's a TSP).
        # Nodes are indexed from 0 to parser_tsp_size - 1, by default the start of
        # the route is node 0.
        routing = pywrapcp.RoutingModel(tsp_size, 1, 0)

        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
        # Setting first solution heuristic (cheapest addition).
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

        # Setting the cost function.
        # Put a callback to the distance accessor here. The callback takes two
        # arguments (the from and to node inidices) and returns the distance between
        # these nodes.
        # Create the distance callback, which takes two arguments (the from and to node indices)
        # and returns the distance between these nodes.

        dist_between_nodes = CreateDistanceCallback()
        dist_callback = dist_between_nodes.Distance
        routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)

        # Solve, returns a solution if any.
        assignment = routing.SolveWithParameters(search_parameters)
        # assignment = routing.Solve()
        if assignment:
            # Solution cost.
            print "Total distance: " + str(assignment.ObjectiveValue()) + " miles\n"
            # Inspect solution.
            # Only one route here; otherwise iterate from 0 to routing.vehicles() - 1
            route_number = 0
            index = routing.Start(route_number)  # Index of the variable for the starting node.
            route = ''
            while not routing.IsEnd(index):
                # Convert variable indices to node indices in the displayed route.
                route += str(city_names[routing.IndexToNode(index)]) + ' -> '
                index = assignment.Value(routing.NextVar(index))
            route += str(city_names[routing.IndexToNode(index)])
            print "Route:\n\n" + route
        else:
            print 'No solution found.'
    else:
        print('Specify an instance greater than 0.')


if __name__ == '__main__':
    main()


def astar(start_state, goal_state, get_neighbors, heuristic):
    # Initialize the start node
    start_node = Node(state=start_state, cost=0, heuristic=heuristic(start_state, goal_state))
    open_list = [start_node]  # Priority queue (min heap) for open nodes
    closed_set = set()  # Set to store closed nodes

    while open_list:
        # Get the node with the lowest total cost from the open list
        current_node = heapq.heappop(open_list)

        if current_node.state == goal_state:
            # Goal reached, reconstruct the path
            path = []
            while current_node:
                path.append((current_node.state, current_node.action))
                current_node = current_node.parent
            path.reverse()
            return path

        closed_set.add(current_node.state)

        for action, neighbor_state, step_cost in get_neighbors(current_node.state):
            if neighbor_state in closed_set:
                continue  # Skip closed nodes

            # Calculate the cost to reach the neighbor from the current node
            neighbor_cost = current_node.cost + step_cost

            # Check if the neighbor is already in the open list
            neighbor_node = None
            for node in open_list:
                if node.state == neighbor_state:
                    neighbor_node = node
                    break

            if neighbor_node is None or neighbor_cost < neighbor_node.cost:
                # Create or update the neighbor node
                neighbor_heuristic = heuristic(neighbor_state, goal_state)
                neighbor_node = Node(
                    state=neighbor_state,
                    parent=current_node,
                    action=action,
                    cost=neighbor_cost,
                    heuristic=neighbor_heuristic,
                )

                if neighbor_node not in open_list:
                    heapq.heappush(open_list, neighbor_node)

    return None  # No path found



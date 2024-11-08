"""Usage of Rapidly-Exploring Random Tree algorithm."""

import math
import random

import matplotlib.pyplot as plt

import numpy as np


class RRT:
    """RRT for path planning."""

    def __init__(
        self,
        start,
        end,
        grid,
        distance=3.0,
        resolution=0.5,
        goal_chance=5,
        max_steps=2000,
    ):
        """Inroduce the RRT algorithm with the given parameters."""
        self.start = {
            'x': start[0],
            'y': start[1],
            'path_x': [],
            'path_y': [],
            'parent': None,
        }
        self.end = {'x': end[0], 'y': end[1]}
        self.distance = distance
        self.resolution = resolution
        self.goal_chance = goal_chance
        self.max_steps = max_steps
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0])
        self.nodes = [self.start]

    def search(self, animate=True):
        """Search for a path from start to goal."""
        for i in range(self.max_steps):
            rand_point = self.get_random_point()
            close_index = self.get_closest_node_index(rand_point)
            close_node = self.nodes[close_index]
            new_node = self.extend(close_node, rand_point)

            # Introduce new node only if it is within bounds and obstacle-free
            if self.in_bounds(new_node) and self.not_in_obstacle(new_node):
                self.nodes.append(new_node)

            # Check if it reached the goal
            if self.get_goal_dist(self.nodes[-1]) <= self.distance:
                final_node = self.extend(self.nodes[-1], self.end)
                if self.not_in_obstacle(final_node):
                    return self.make_path(len(self.nodes) - 1)

            # Animate the plot every few steps if animation is enabled
            if animate and i % 5 == 0:
                self.show_plot(rand_point)

        return None

    def extend(self, start_node, target_node):
        """Extend the tree towards the target node."""
        new_node = {
            'x': start_node['x'],
            'y': start_node['y'],
            'path_x': [start_node['x']],
            'path_y': [start_node['y']],
            'parent': start_node,
        }
        d = self.calc_distance(new_node, target_node)
        theta = self.calc_angle(new_node, target_node)

        max_length = min(d, self.distance)
        num_steps = int(max_length / self.resolution)

        for _ in range(num_steps):
            new_node['x'] += self.resolution * math.cos(theta)
            new_node['y'] += self.resolution * math.sin(theta)
            new_node['path_x'].append(new_node['x'])
            new_node['path_y'].append(new_node['y'])

        d = self.calc_distance(new_node, target_node)
        if d <= self.resolution:
            new_node['x'], new_node['y'] = target_node['x'], target_node['y']
            new_node['path_x'].append(target_node['x'])
            new_node['path_y'].append(target_node['y'])

        return new_node

    def make_path(self, end_index):
        """Build the path from the start to the goal."""
        path = [[self.end['x'], self.end['y']]]
        node = self.nodes[end_index]
        while node['parent'] is not None:
            path.append([node['x'], node['y']])
            node = node['parent']
        path.append([self.start['x'], self.start['y']])
        return path[::-1]

    def get_goal_dist(self, node):
        """Compute distance from the node to the goal."""
        dx = node['x'] - self.end['x']
        dy = node['y'] - self.end['y']
        return math.sqrt(dx**2 + dy**2)

    def get_random_point(self):
        """Obtain a random point, with some bias towards the goal."""
        if random.randint(0, 100) > self.goal_chance:
            return {
                'x': random.random() * (self.width - 1),
                'y': random.random() * (self.height - 1),
            }
        else:
            return self.end

    def in_bounds(self, node):
        """Check if a node is within the bounds of the grid."""
        return 0 <= node['x'] < self.width and 0 <= node['y'] < self.height

    def not_in_obstacle(self, node):
        """Check if any point along the path is free of obstacles."""
        for x, y in zip(node['path_x'], node['path_y']):
            if self.grid[int(round(y))][int(round(x))] == 1:
                return False
        return True

    def calc_distance(self, node1, node2):
        """Compute the Euclidean distance between two nodes."""
        dx = node2['x'] - node1['x']
        dy = node2['y'] - node1['y']
        return math.sqrt(dx * dx + dy * dy)

    def calc_angle(self, node1, node2):
        """Compute the angle from node1 to node2."""
        dx = node2['x'] - node1['x']
        dy = node2['y'] - node1['y']
        return math.atan2(dy, dx)

    def get_closest_node_index(self, rand_node):
        """Find the closest node in the tree to the random node."""
        dists = [
            (node['x'] - rand_node['x']) ** 2
            + (node['y'] - rand_node['y']) ** 2
            for node in self.nodes
        ]
        return dists.index(min(dists))

    def show_plot(self, rand_point=None):
        """Display the current plot of the RRT search process."""
        plt.clf()
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None],
        )

        # Depict the grid and obstacles
        for y, row in enumerate(self.grid):
            for x, val in enumerate(row):
                if val == 1:
                    plt.plot(x, y, 's', color='gray')

        # Depict each node path
        for node in self.nodes:
            if node['parent']:
                plt.plot(node['path_x'], node['path_y'], '-r')

        # Depict start, end, random points
        plt.plot(self.start['x'], self.start['y'], 'o', color='blue')
        plt.plot(self.end['x'], self.end['y'], 'o', color='orange')
        if rand_point:
            plt.plot(rand_point['x'], rand_point['y'], 'x', color='green')
        plt.axis('equal')
        plt.grid(True)
        plt.pause(0.01)


def main():
    """Define map, probability and run RRT."""
    height, width = 80, 80
    grid = np.random.choice([0, 1], size=(height, width), p=[0.85, 0.15])

    start = [0, 0]
    end = [height - 1, width - 1]

    # Make sure that the start and end points are in free space
    while grid[start[1]][start[0]] == 1:
        start = [random.randint(0, width - 1), random.randint(0, height - 1)]
    while grid[end[1]][end[0]] == 1:
        end = [random.randint(0, width - 1), random.randint(0, height - 1)]

    print('Start position:', start)
    print('End position:', end)

    rrt = RRT(start=start, end=end, grid=grid)
    path = rrt.search(animate=True)

    if path is None:
        print('Path could not be found')
    else:
        print('Path found!')
        rrt.show_plot()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-b')
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()

import random
import math

class RRT:
    def __init__(self, start, end, distance=2.0, resolution=0.5, max_steps=2000):
        self.start = {"x": start[0], "y": start[1], "path_x": [], "path_y": [], "parent": None}
        self.end = {"x": end[0], "y": end[1]}
        self.distance = distance
        self.resolution = resolution
        self.max_steps = max_steps
        self.nodes = [self.start]

    def search(self):
        for _ in range(self.max_steps):
            rand_point = self.get_random_point()
            close_index = self.get_closest_node_index(rand_point)
            close_node = self.nodes[close_index]
            new_node = self.extend(close_node, rand_point)

            self.nodes.append(new_node)
            if self.get_goal_dist(self.nodes[-1]) <= self.distance:
                return self.make_path(len(self.nodes) - 1)
        return None  

    def extend(self, start_node, target_node):
        new_node = {
            "x": start_node["x"],
            "y": start_node["y"],
            "path_x": [start_node["x"]],
            "path_y": [start_node["y"]],
            "parent": start_node
        }
        d = self.calc_distance(new_node, target_node)
        theta = self.calc_angle(new_node, target_node)
        max_length = min(d, self.distance)
        num_steps = int(max_length / self.resolution)

        for _ in range(num_steps):
            new_node["x"] += self.resolution * math.cos(theta)
            new_node["y"] += self.resolution * math.sin(theta)
            new_node["path_x"].append(new_node["x"])
            new_node["path_y"].append(new_node["y"])

        return new_node

    def make_path(self, end_index):
        path = [[self.end["x"], self.end["y"]]]
        node = self.nodes[end_index]
        while node["parent"] is not None:
            path.append([node["x"], node["y"]])
            node = node["parent"]
        path.append([self.start["x"], self.start["y"]])
        return path[::-1]

    def get_goal_dist(self, node):
        dx = node["x"] - self.end["x"]
        dy = node["y"] - self.end["y"]
        return math.sqrt(dx ** 2 + dy ** 2)

    def get_random_point(self):
        return {"x": random.random() * 80, "y": random.random() * 80}

    def calc_distance(self, node1, node2):
        dx = node2["x"] - node1["x"]
        dy = node2["y"] - node1["y"]
        return math.sqrt(dx ** 2 + dy ** 2)

    def calc_angle(self, node1, node2):
        dx = node2["x"] - node1["x"]
        dy = node2["y"] - node1["y"]
        return math.atan2(dy, dx)

    def get_closest_node_index(self, rand_node):
        dists = [(node["x"] - rand_node["x"]) ** 2 + (node["y"] - rand_node["y"]) ** 2 for node in self.nodes]
        return dists.index(min(dists))


def main():
    start = [0, 0]
    end = [79, 79]
    
    rrt = RRT(start=start, end=end)
    path = rrt.search()

    if path is None:
        print("Path could not be found")
    else:
        print("Path found:", path)


if __name__ == '__main__':
    main()

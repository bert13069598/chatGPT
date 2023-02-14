import random
import math
import matplotlib.pyplot as plt


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:
    def __init__(self, start, goal, obstacle_list, rand_area, expand_dis=1.0, goal_sample_rate=20, max_iter=1000):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.min_rand_x = rand_area[0]
        self.max_rand_x = rand_area[1]
        self.min_rand_y = rand_area[2]
        self.max_rand_y = rand_area[3]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = [self.start]

    def planning(self):
        for i in range(self.max_iter):
            if random.randint(0, 100) > self.goal_sample_rate:
                rand_node = self.get_random_node()
            else:
                rand_node = self.goal
            nearest_ind = self.get_nearest_node_index(self.node_list, rand_node)
            nearest_node = self.node_list[nearest_ind]
            new_node = self.steer(nearest_node, rand_node, self.expand_dis)
            if not self.check_collision(self.obstacle_list, new_node.x, new_node.y):
                self.node_list.append(new_node)
            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y,
                                      [self.goal.x, self.goal.y]) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.goal, self.expand_dis)
                if not self.check_collision(self.obstacle_list, final_node.x, final_node.y):
                    self.node_list.append(final_node)
                    self.draw_graph(self.node_list, self.start, self.goal, self.obstacle_list)
                    return self.generate_final_course(len(self.node_list) - 1)
            if i % 5 == 0:
                self.draw_graph(self.node_list, self.start, self.goal, self.obstacle_list)
        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y)
        dist, angle = self.calc_distance_and_angle(new_node, to_node)
        if extend_length > dist:
            extend_length = dist
        new_node.x += extend_length * math.cos(angle)
        new_node.y += extend_length * math.sin(angle)
        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def get_random_node(self):
        x = random.uniform(self.min_rand_x, self.max_rand_x)
        y = random.uniform(self.min_rand_y, self.max_rand_y)
        return Node(x, y)

    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        angle = math.atan2(dy, dx)

        return distance, angle

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    def check_collision(self, obstacle_list, x, y):
        for (ox, oy, r) in obstacle_list:
            if math.sqrt((ox - x) ** 2 + (oy - y) ** 2) <= r:
                return True  # collision
        return False  # no collision

    def calc_dist_to_goal(self, x, y, goal):
        return math.sqrt((x - goal[0]) ** 2 + (y - goal[1]) ** 2)

    def draw_graph(self, graph, start, goal, obstacle_list):
        plt.clf()
        plt.plot(start.x, start.y, "bs")
        plt.plot(goal.x, goal.y, "rs")
        for (ox, oy, r) in obstacle_list:
            circle = plt.Circle((ox, oy), r, color="k")
            plt.gcf().gca().add_artist(circle)

        # for (x, y) in graph:
        #     for (nx, ny) in graph[(x, y)]:
        #         plt.plot([x, nx], [y, ny], "-g")
        for node in graph:
            nnode = node.parent
            if nnode:
                plt.plot([node.x, nnode.x], [node.y, nnode.y], "-g")
            plt.scatter(node.x, node.y)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.1)

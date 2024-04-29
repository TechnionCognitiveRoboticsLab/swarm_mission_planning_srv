import math
from copy import copy

from . import State


class Node:
    def __init__(self, state, parent=None):
        self.action = None
        self.path = None
        self.children = []
        self.parent = parent
        if self.parent is None:
            self.depth = 0
        else:
            self.depth = parent.depth + 1

        self.state = state
        self.times_visited = 0

        self.value = 0
        self.high = 0
        self.low = 0

    def get_path(self):
        return self.path

    def __str__(self):
        return " R:" + str(self.value) + " " + str(self.state)

    def uct(self, t, c):
        uct = self.value / self.times_visited + c * math.sqrt(math.log(t) / self.times_visited)
        return uct

    def all_children_visited(self):
        if self.state.is_terminal():
            return True
        if not self.children:
            return False
        for child in self.children:
            if child.times_visited == 0:
                return False
        return True

    def pick_unvisited_child(self):
        for c in self.children:
            if not c.times_visited:
                return c
        raise Exception("All children are visited!")

    def highest_uct_child(self, time, exp_rate=math.sqrt(2)):
        if self.children[0].times_visited == 0:
            return self.children[0]
        max_uct = self.children[0].uct(time, exp_rate)
        max_uct_child = self.children[0]
        for c in self.children:
            if c.times_visited == 0:
                return c
            c_uct = c.uct(time, exp_rate)
            if c_uct > max_uct:
                max_uct = c_uct
                max_uct_child = c
        return max_uct_child

    def highest_value_child(self):
        max_value = self.children[0].value
        max_value_child = self.children[0]
        for c in self.children:
            if c.value > max_value:
                max_value = c.value
                max_value_child = c
        return max_value_child

    def most_visited_child(self):
        max_visits = self.children[0].times_visited
        max_visits_child = self.children[0]
        for c in self.children:
            if c.times_visited > max_visits:
                max_visits = c.times_visited
                max_visits_child = c
        return max_visits_child

    def expand(self, instance):
        actions = instance.actions(self.state, self.path)
        self.children = []
        for action in actions:
            child = Node(instance.make_action(action, self.state), self)
            child.path = {a: self.path[a].copy() for a in self.path}
            for a in action:
                child.path[a].append(action[a])
            child.action = action
            self.children.append(child)

    def backpropagate(self, value):
        backpropagator = self
        while backpropagator.parent is not None:
            backpropagator.value += value
            backpropagator = backpropagator.parent
        backpropagator.value += value

    # Printing Tree
    def add_to_paths(self, paths):
        new_paths = []
        for i in range(len(paths)):
            attachment = None
            if i == 0:
                attachment = self
            path = [attachment]
            path_i = paths[i]
            path.extend(path_i)
            new_paths.append(path)
        return new_paths

    def get_leaf_paths(self):
        if not len(self.children):
            leaf = [[self]]
            return leaf
        else:
            paths = []
            for child in self.children:
                paths.extend(child.get_leaf_paths())
            return self.add_to_paths(paths)

    def get_tree(self):
        for path in self.get_leaf_paths():
            line = ""
            is_first = True
            for n in path:
                if n is not None:
                    if is_first:
                        line += "└── "
                        is_first = False
                    else:
                        line += "── "
                    line += str(n)
                else:
                    line += "                      "
            print(line)

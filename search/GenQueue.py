import heapq


class GenQueue:
    def __init__(self):
        pass

    def push(self, node):
        pass

    def pop(self):
        pass

    def is_empty(self):
        pass


class PrioritizedData:
    def __init__(self, node, priority):
        self.node = node
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority


class RegularQueueNode:
    def __init__(self, data=None):
        self.data = data
        self.next = None


class PriorityQueue(GenQueue):
    def __init__(self, data=None):
        super().__init__()
        self._queue = []
        if data is not None:
            self.push(data)

    def push(self, data):
        heapq.heappush(self._queue, PrioritizedData(data, - data.value - data.high))

    def pop(self):
        if self._queue:
            return heapq.heappop(self._queue).node
        else:
            raise IndexError("pop from an empty priority queue")

    def is_empty(self):
        return len(self._queue) == 0

class AstarQueue(PriorityQueue):
    def __init__(self, data=None):
        super().__init__(data)

    def push(self, data):
        heapq.heappush(self._queue, PrioritizedData(data, - data.value - data.low))


class RegularQueue(GenQueue):
    def __init__(self, data=None):
        super().__init__()
        self.front = None
        self.rear = None
        if data is not None:
            self.push(data)

    def is_empty(self):
        return self.front is None

    def push(self, data):
        new_node = RegularQueueNode(data)
        if self.is_empty():
            self.front = self.rear = new_node
        else:
            self.rear.next = new_node
            self.rear = new_node

    def pop(self):
        if self.is_empty():
            raise IndexError("pop from an empty regular queue")
        else:
            data = self.front.data
            self.front = self.front.next
            if self.front is None:
                self.rear = None
            return data


class Stack(GenQueue):
    def __init__(self, data=None):
        super().__init__()
        self.stack = []
        if data is not None:
            self.push(data)

    def pop(self):
        return self.stack.pop()

    def push(self, data):
        self.stack.append(data)

    def is_empty(self):
        return len(self.stack) == 0

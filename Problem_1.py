# Lab 4 - A*
# Problem 1 - Priority Queue
# Kyle, Cole, Olver, Rafael

# class for Node with data and priority
class Node:

    def __init__(self, info, priority):
        self.info = info
        self.priority = priority

# class for Priority queue
class PriorityQueue:

    def __init__(self):
        self.queue = list()
        # if you want you can set a maximum size for the queue

    def insert(self, node):
        # if queue is empty
        if self.size() == 0:
            # add the new node
            self.queue.append(node)
        else:
            # traverse the queue to find the right place for new node
            for x in range(0, self.size()):
                # if the priority of new node is less
                if node.priority <= self.queue[x].priority:
                    # if we have traversed the complete queue
                    if x == (self.size() - 1):
                        # add new node at the end (most negative)
                        self.queue.insert(x + 1, node)
                    else:
                        continue
                else:
                    self.queue.insert(x, node)

    def delete(self):
        # remove the first node from the queue
        return self.queue.pop(0)

    def show(self):
        return self.queue[0].info, self.queue[0].priority

    def size(self):
        return len(self.queue)


if __name__ == "__main__":
    testQueue = PriorityQueue()

    node1 = Node("C", -3)
    node2 = Node("B", -3)
    node3 = Node("A", -1)
    node4 = Node("Z", -26)
    node5 = Node("Y", -35)
    node6 = Node("L", -2)

    testQueue.insert(node1)
    testQueue.insert(node2)
    testQueue.insert(node3)
    testQueue.insert(node4)
    testQueue.insert(node5)
    testQueue.insert(node6)
    print(testQueue.show())

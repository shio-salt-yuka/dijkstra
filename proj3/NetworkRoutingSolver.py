#!/usr/bin/python3
import heapq

from CS312Graph import *
import time


class NetworkRoutingSolver:
    def __init__(self):
        self.prev = None
        self.dist = None

    def initializeNetwork(self, network):
        assert (type(network) == CS312Graph)
        self.network = network

    def getShortestPath(self, destIndex):
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL 
        #       NEED TO USE
        path_edges = []
        total_length = self.dist[destIndex]
        node = self.prev[destIndex]  # node connect to the destination
        index = destIndex
        # if node is the source or unreachable
        if node is None:
            return {'cost': float('inf'), 'path': path_edges}
            # return 'Error'
        while node is not None:
            edge = None
            for v in node.neighbors:
                if v.dest.node_id == index:
                    edge = v
                    break
            path_edges.append((edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)))
            index = node.node_id
            node = self.prev[node.node_id]

        return {'cost': total_length, 'path': path_edges}

    def computeShortestPaths(self, srcIndex, use_heap=True):
        self.source = srcIndex
        t1 = time.time()
        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)

        self.dijkstra(self.network, self.source, use_heap)

        t2 = time.time()
        return (t2 - t1)

    def dijkstra(self, graph, source, use_heap=False):
        v = set()  # create vertex set Q
        self.dist = []
        self.prev = []
        for node in graph.nodes:
            self.dist.append(float('inf'))
            self.prev.append(None)
            v.add(node)
        self.dist[source] = 0

        H = None

        if use_heap:
            H = BinaryHeapImplementation(self.dist)
            h = H.makeHeap(v)
        else:
            H = ArrayImplementation()
            h = H.makeQueue(v)

        while len(h) != 0:
            u = H.deleteMin(self.dist)
            for neighbor in u.neighbors:
                tempDist = self.dist[u.node_id] + neighbor.length
                if tempDist < self.dist[neighbor.dest.node_id]:
                    self.dist[neighbor.dest.node_id] = tempDist
                    self.prev[neighbor.dest.node_id] = u
                    H.decreaseKey(neighbor.dest.node_id)
        return self.dist, self.prev


class ArrayImplementation:
    def __init__(self):
        self.unsorted_array = None

    def makeQueue(self, v_set):
        self.unsorted_array = list(v_set)
        return self.unsorted_array

    def deleteMin(self, dist):
        x = self.unsorted_array[0]  # node object to nodeID
        for y in self.unsorted_array:
            if dist[y.node_id] < dist[x.node_id]:
                x = y

        self.unsorted_array.remove(x)
        return x

    def decreaseKey(self, node):
        pass


class BinaryHeapImplementation:
    def __init__(self, dist):
        self.heap = None
        self.dist = dist
        self.pointer = []

    def makeHeap(self, v_set):
        size = len(v_set)
        self.heap = []
        for x in v_set:
            self.heap.append(x)
        for i in range(size - 1, 0, -1):
            self.siftDown(self.heap[i], i)

        return self.heap

    def insert(self, x):
        self.heap.append(x)
        self.bubbleUp(x, len(self.heap) - 1)

    def decreaseKey(self, x):
        self.bubbleUp(x, self.pointer[x.node_id])

    def deleteMin(self, dist):
        if len(self.heap) == 0:
            return None;
        else:
            x = self.heap[0]
            self.siftDown(self.heap[len(self.heap)-1], 1)
            return x

    def bubbleUp(self, x, i):
        p = (i - 1) // 2
        while i != 0 and self.dist[self.heap[p].node_id] > self.dist[x.node_id]:
            self.swap(i, p)
            i = p
            p = (i - 1) // 2

    def swap(self, i1, i2):
        item1 = self.heap[i1]
        item2 = self.heap[i2]
        index1 = self.pointer[item1.node_id]
        index2 = self.pointer[item2.node_id]
        self.heap[i2] = item1
        self.heap[i1] = item2
        self.pointer[item1.node_id] = index2
        self.pointer[item2.node_id] = index1

    def siftDown(self, x, i):
        c = self.minChild(i)
        while c != 0 and self.dist[self.heap[c].node_id] < self.dist[x.node_id]:
            self.swap(i, c)
            i = c
            c = self.minChild(i)

    def minChild(self, i):
        left = 2 * i
        right = 2 * i + 1
        if 2 * i >= len(self.heap):  # no children
            return 0
        elif self.dist[left-1] < self.dist[right-1]:  # if left smaller than right child, return index of child
            return left-1
        else:  # return index of right child
            return right-1

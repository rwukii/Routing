####################################################
# LSrouter.py
# Name:
# HUID:
#####################################################

from router import Router
from packet import Packet
import json
import heapq
class LSrouter(Router):
    """Link state routing protocol implementation.

    Add your own class fields and initialization code (e.g. to create forwarding table
    data structures). See the `Router` base class for docstrings of the methods to
    override.
    """

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Initialize base class - DO NOT REMOVE
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        # TODO
        self.seq = 0
        self.neighbors = {}
        self.ports_to_neighbors = {}
        self.lsdb = {} 
        self.forwarding_table = {}
        #   add your own class fields and initialization code here
        # pass

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        # TODO
        if packet.is_traceroute:
            dst = packet.dst_addr
            if dst in self.forwarding_table:
                out_port = self.forwarding_table[dst]
                self.send(out_port, packet)
        else:
            lsa = json.loads(packet.content)
            name = lsa["name"]
            seq = lsa["seq"]
            links = lsa["links"]
            if name not in self.lsdb or seq > self.lsdb[name]["seq"]:
                self.lsdb[name] = {
                    "seq": seq,
                    "links": links
                }
                self.run_dijkstra()
                
                for neighbor, p in self.ports_to_neighbors.items():
                    if p != port:
                        self.send(p, packet)
            

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        # TODO
        self.neighbors[port] = (endpoint, cost) # port -> (neighbor, cost)
        self.ports_to_neighbors[endpoint] = port # neighbor -> port
        self.lsdb[self.addr] = {
            "seq": self.seq,
            "links": {n: c for _, (n, c) in self.neighbors.items()}
        }
        self.seq += 1 
        self.flood_own_lsa() 
        self.run_dijkstra()
        #   update local data structures and forwarding table
        #   broadcast the new link state of this router to all neighbors
        

    def handle_remove_link(self, port):
        """Handle removed link."""
        if port in self.neighbors:
            endpoint, _ = self.neighbors[port]
            del self.neighbors[port]
            if endpoint in self.ports_to_neighbors:
                del self.ports_to_neighbors[endpoint]
        
        self.lsdb[self.addr] = {
            "seq": self.seq,
            "links": {n: c for _, (n, c) in self.neighbors.items()}
        }
        self.seq += 1 
        
        self.flood_own_lsa()
        self.run_dijkstra()

    def handle_time(self, time_ms):
        """Handle current time."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.flood_own_lsa()

    def __repr__(self):
        """Representation for debugging in the network visualizer."""
        # TODO
        #   NOTE This method is for your own convenience and will not be graded
        return f"LSrouter(addr={self.addr})"
    
    def flood_own_lsa(self):
        lsa = {
            "name": self.addr,
            "seq": self.seq,
            "links": {n: c for _, (n, c) in self.neighbors.items()}
        }
        packet = Packet(Packet.ROUTING, self.addr, None, json.dumps(lsa))
        for neighbor, port in self.ports_to_neighbors.items():
            self.send(port, packet)
        

    def run_dijkstra(self):
        graph = {} 
        for router, entry in self.lsdb.items():
            graph[router] = entry["links"]

        dist = {self.addr: 0}
        prev = {}
        visited = set()
        heap = [(0, self.addr)]

        while heap:
            cost_u, u = heapq.heappop(heap)
            if u in visited:
                continue
            visited.add(u)

            for v, cost_uv in graph.get(u, {}).items():
                if v not in dist or cost_u + cost_uv < dist[v]:
                    dist[v] = cost_u + cost_uv
                    prev[v] = u
                    heapq.heappush(heap, (dist[v], v))

        self.forwarding_table = {}

        for dest in dist:
            if dest == self.addr:
                continue
            next_hop = dest
            while prev.get(next_hop) != self.addr:
                next_hop = prev.get(next_hop, next_hop)
            if next_hop in self.ports_to_neighbors:
                port = self.ports_to_neighbors[next_hop]
                self.forwarding_table[dest] = port

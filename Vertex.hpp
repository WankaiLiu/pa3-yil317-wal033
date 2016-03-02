#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <string>
#include <unordered_map>

#include "Edge.hpp"
using namespace std;
/**
 * Represents a Vertex in a graph.
 *
 * Vertices are connected to other Vertices via Edges. Each Vertex
 * maintains a collection of all Edges that originate from it.
 */
class Vertex {
  // Graph needs access to Edge map for Dijkstra/Prim algorithms.
  friend class UndirectedGraph;
    
public:
  /**
   * Initialize the Vertex with the given name.
   */
  Vertex(const std::string &name);
  void Addedge(const std::string node,Edge newEdge);
  void AddMSTedge(Vertex* node,Edge newEdge);
  unsigned int GetVertexCost();
  void SetDist(unsigned int distance);
  void SetVisit(bool visited);
  //Vertex* NearestUnvisitNeighbor();
  bool GetVisit();
  unsigned int GetDist();
  std::unordered_map<Vertex*, Edge> GetMSTedges();  
  std::unordered_map<std::string, Edge> Getedges();
private:

  /**
   * Name of this Vertex.
   */
  std::string name;
    
  /**
   * Distance of this Vertex from initial Vertex.
   * Used by Dijkstra's algorithm.
   */
  unsigned int distance;
    
  /**
   * Whether this node has been visited.
   * Used by Prim's algorithm.
   */
  bool visited;

  /**
   * Map of adjacent Vertex name to Edge describing the adjacency.
   */
  std::unordered_map<std::string, Edge> edges;
  std::unordered_map<Vertex*, Edge> MSTedges;
};

#endif

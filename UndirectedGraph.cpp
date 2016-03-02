#include "UndirectedGraph.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"
#include <vector>
#include <queue>
#include <iostream>

using namespace std;
// Method implementations here
#define NNDEBUG
#ifdef NDEBUG
#  define DEBUG(x) do {} while (0)
#else
#  define DEBUG(x) std::cout << x;
#endif

UndirectedGraph::UndirectedGraph(){
}
//Add a new vertex to graph.
void UndirectedGraph::Addvertex(std::string &name, Vertex* ver){
  this->vertices.insert(make_pair(name,ver));
}
//Delete all vertex.
UndirectedGraph::~UndirectedGraph(){
  std::unordered_map<std::string, Vertex*>::iterator it;
  for(it = vertices.begin(); it != vertices.end(); it++){
    delete (it->second);
  }

}
//Reset the distance of all node to maximum.
void UndirectedGraph::ResetDist(){
  std::unordered_map<std::string, Vertex*>::iterator it;
  for(it = vertices.begin(); it != vertices.end(); it++){
    it->second->distance = 4294967294;
    it->second->SetVisit(0);
  }

}
//Define the operator() which is used for priority_queue.
bool  UndirectedGraph::DijkstraVertexComparator::operator()
  (const std::pair<Vertex*, unsigned int> &left,
   const std::pair<Vertex*, unsigned int> &right){
  return left.second>right.second;
}
//Define the operator() which is used for priority_queue.
bool  UndirectedGraph::DijkstraEdgeComparator::operator()
  (const std::pair<Vertex*, Edge> &left,
   const std::pair<Vertex*, Edge> &right){
  return !(left.second<right.second);
}
/******************************************************************
 Use Dijkstra's Algorithm to compute minmum distance for input vertex.
********************************************************************/
bool UndirectedGraph::SetNearestDist(Vertex* own){
  std::unordered_map<std::string, Edge>::iterator itEdge;
  std::priority_queue<std::pair<Vertex*, unsigned int>, 
    std::vector<std::pair<Vertex*, unsigned int>>, 
    UndirectedGraph::DijkstraVertexComparator> heapVertex; //Store and sort vertices by distance.
  unordered_map<std::string, Edge> tempEdges; //Store edges from each vertex.
  Edge edge;
  Vertex* shortestVertex;// The next shortest vertex.
  Vertex* neighbor; //The adjacent vertex.
  unsigned int tempDist; //The distance of next shortest vertex.
  //Initialize the first vertex.
  own->SetDist(0);
  own->SetVisit(1);
  shortestVertex = own;
  while(1){
    tempEdges = shortestVertex->Getedges();
    //Use iterator to traverse all Vertices.
    for(itEdge = tempEdges.begin();      
	itEdge != tempEdges.end(); itEdge++){   
      tempDist =  itEdge->second.getLength()+shortestVertex->GetDist();
      //Store all its adjacent vertices in heap.
      heapVertex.push(std::make_pair(itEdge->second.getNeighbor(shortestVertex),
				     tempDist));
    }
    while (1){
      if( heapVertex.empty()){
	return 1;
      }
      //After finding the next shortest vertex, set its distance and pop it out.
      if (heapVertex.top().first->GetVisit() == 0){
	neighbor = heapVertex.top().first;
	neighbor->SetDist(heapVertex.top().second);
	neighbor->SetVisit(1);
	heapVertex.pop();
	break;
      }
      else{
	// If the vertex has already been visited, just pop it out.
	heapVertex.pop();
      }
    }
    //Update the next shortest vertex.
    shortestVertex = neighbor; 
  }
}
/******************************************************************
 Use Prim's Algorithm to compute minmum cost for input vertex.
********************************************************************/
bool UndirectedGraph::SetCostMST(){
  std::unordered_map<std::string, Edge>::iterator itEdge;
  std::unordered_map<std::string, Vertex*>::iterator itVertex;
   //Store and sort vertices by cost.
  std::priority_queue<std::pair<Vertex*, Edge>, 
    std::vector<std::pair<Vertex*, Edge>>, 
    UndirectedGraph::DijkstraEdgeComparator> heapVertex;
  unordered_map<std::string, Edge> tempEdges;//Store edges from each vertex.
  Edge edge;
  Edge tempEdge;
  Vertex* shortestVertex;// The next shortest vertex.
  Vertex* neighbor; //The adjacent vertex.
  Vertex* tempVertex;
  itVertex = this -> vertices.begin();
  shortestVertex = vertices.begin()->second;
  shortestVertex->SetDist(0);
  shortestVertex->SetVisit(1);
  while(1){
    tempEdges = shortestVertex->Getedges();
    for(itEdge = tempEdges.begin();      
	itEdge != tempEdges.end(); itEdge++){ 
      //Store all its adjacent vertices in heap.
      heapVertex.push(std::make_pair(itEdge->second.getNeighbor(shortestVertex),
				     itEdge->second));
    }
    while (1){
      if( heapVertex.empty()){
	return 1;
      }
      //After finding the next shortest vertex, set its distance and pop it out.
      if (heapVertex.top().first->GetVisit() == 0){
	neighbor = heapVertex.top().first;
	tempEdge = heapVertex.top().second;
	neighbor->SetDist(tempEdge.getCost());
	neighbor->SetVisit(1);
	heapVertex.pop();
	//Add the MST edge to the corresponding vertex.
	tempVertex = tempEdge.getNeighbor(neighbor);
	neighbor->AddMSTedge(tempVertex, tempEdge);
	tempVertex->AddMSTedge(neighbor, tempEdge);
	break;
      }
      else{
	// If the vertex has already been visited, just pop it out.
	heapVertex.pop();
      }
    }
    //Update the next shortest vertex.
    shortestVertex = neighbor; 
  }
}

/******************************************************************
 Use Prim's Algorithm to compute minmum distance for input vertex.
********************************************************************/
bool UndirectedGraph::SetMSTDist(Vertex* own){
  std::unordered_map<Vertex*, Edge>::iterator itEdge;
   //Store and sort vertices by cost.
  std::priority_queue<std::pair<Vertex*, unsigned int>, 
    std::vector<std::pair<Vertex*, unsigned int>>, 
    UndirectedGraph::DijkstraVertexComparator> heapVertex;
  unordered_map<Vertex*, Edge> tempEdges;//Store edges from each vertex.
  Edge edge;
  Vertex* shortestVertex;// The next shortest vertex.
  Vertex* neighbor; //The adjacent vertex.
  unsigned int tempDist;//The distance of next shortest vertex.
  own->SetDist(0);
  own->SetVisit(1);
  shortestVertex = own;
  while(1){
    tempEdges = shortestVertex->GetMSTedges();
    for(itEdge = tempEdges.begin();      
	itEdge != tempEdges.end(); itEdge++){   
      //Store all its adjacent vertices in heap.
      tempDist =  itEdge->second.getLength()+shortestVertex->GetDist();
      heapVertex.push(std::make_pair(itEdge->second.getNeighbor(shortestVertex),
				     tempDist));
    }
    while (1){
      if( heapVertex.empty()){
	return 1;
      }
      //After finding the next shortest vertex, set its distance and pop it out.
      if (heapVertex.top().first->GetVisit() == 0){
	neighbor = heapVertex.top().first;
	neighbor->SetDist(heapVertex.top().second);
	neighbor->SetVisit(1);
	heapVertex.pop();
	break;
      }
      else{
	// If the vertex has already been visited, just pop it out.
	heapVertex.pop();
      }
    }
    //Update the next shortest vertex.
    shortestVertex = neighbor; 
  }
}

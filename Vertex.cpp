#include "Vertex.hpp"
#include <iostream>
// Method implementations here
using namespace std;
Vertex::Vertex(const std::string &name){
  this->name = name;
  this->visited = 0;
  this->distance = 4294967294;
}
//Add a new edge to the vertex.
void Vertex::Addedge(const std::string node,Edge newEdge){
  this->edges.insert(make_pair(node,newEdge));
}
//Add a new MST edge to the vertex.
void Vertex::AddMSTedge( Vertex* node,Edge newEdge){
  pair <Vertex*, Edge> tempPair = make_pair(node,newEdge);
  this->MSTedges.insert(tempPair);
}
//Compute the total cost of adjacent edges.
unsigned int Vertex::GetVertexCost(){
  std::unordered_map<std::string, Edge>::iterator it;
  Edge edge;
  unsigned int totalCost=0;
  for(it = edges.begin(); it != edges.end(); it++){
    edge=it->second;
    totalCost += edge.getCost();
  }
  return totalCost;
}
//Set the distance.
void Vertex::SetDist(unsigned int distance){
  this->distance = distance;
}
//Set the value of visit.
void Vertex::SetVisit(bool visited){
  this->visited = visited;
}
//Get the value of visit.
bool Vertex::GetVisit(){
  if (this->visited == 0){
    return 0;
  }
  else{
    return 1;
  }
}
//Get the distance of vertex.
unsigned int Vertex::GetDist(){
  return this->distance;
}
//Get the adjacent edge of this vertex.
std::unordered_map<std::string, Edge> Vertex::Getedges(){
  return this->edges;
}
//Get the adjacent MST edge of this vertex.
std::unordered_map<Vertex*, Edge> Vertex::GetMSTedges(){
  return this->MSTedges;
}

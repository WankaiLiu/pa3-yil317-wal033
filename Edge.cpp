#include "Edge.hpp"

// Method implementations here
//Initialize a new edge.
Edge::Edge(Vertex *from, Vertex *to,
	   unsigned int cost,
	   unsigned int length){
  this->from = from;
  this->to = to;
  this->cost = cost;
  this->length = length;
}

Edge::Edge(){

}
//Assign the new value to edge.
void Edge::setEdge(Vertex *from, Vertex *to,
		   unsigned int cost,
		   unsigned int length){
  this->from = from;
  this->to = to;
  this->cost = cost;
  this->length = length;
}
//Get the cost of the edge.
unsigned int Edge::getCost(){
  return cost;
}
//Get the time of the edge.
unsigned int Edge::getLength(){
  return length;
}
//The the adjacent vertex linked by the edge.
Vertex* Edge::getNeighbor(Vertex* own){
  if (own == from){
    return this->to;
  }
  else {
    return this->from;
  }
}
//The operator to compare two edges via cost.
bool Edge::operator<(const Edge &right) const{
  return this->cost < right.cost;
}

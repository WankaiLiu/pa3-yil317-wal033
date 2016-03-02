#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include "UndirectedGraph.hpp"

using namespace std;

#define NDEBUG
#ifdef NDEBUG
#  define DEBUG(x) do {} while (0)
#else
#  define DEBUG(x) std::cout << x;
#endif

/**
 * Entry point into the netplan program.
 *
 * Usage:
 *   ./netplan infile
 *
 */
int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " infile" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::ifstream in(argv[1]);
    UndirectedGraph g;
    string v1; //Name of input vertex "from"
    string v2; //Name of input vertex "to"
    unsigned int cost; //The cost of edge
    unsigned int time; //The time of edge
    Edge edge; 
    Vertex* from;
    Vertex* to;
    unordered_map<string,Vertex*> vertices; //Store input vertex
    unordered_map<string,Vertex*>::iterator itVertices;
    unordered_map<string,Vertex*>::iterator itVertices2;
    if (!in) {
        std::cerr << "Unable to open file for reading." << std::endl;
        return EXIT_FAILURE;
	
    }
    
    // Implementation here
    /************************************************
      Create the vertex in graph and add the edd to vertex.
    **************************************************/
    while(in.good()) {
      //Read the data from original file.
      in >> v1 >> v2 >> cost >> time;
      DEBUG("\t DEBUG:   "); DEBUG(v1 << v2 << cost << time << endl);
      if(!in.good()) break;
      itVertices = vertices.find(v1);
      /* If the vertex is new, create a new vertex to graph. Otherwise, 
	 set the existing vertex as pointer.  */ 
      if (itVertices == vertices.end()){
	from = new Vertex(v1);
	vertices.insert(make_pair(v1,from));
	g.Addvertex(v1,from);
      }
      else{
	from = vertices[v1];
      }
      itVertices = vertices.find(v2);
      if (itVertices == vertices.end()){
	to = new Vertex(v2);
	vertices.insert(make_pair(v2,to));
	g.Addvertex(v2,to);
      }
      else{
	to = vertices[v2];
      }
      // Insert the edge to corresponding vertex.
      edge.setEdge(from,to,cost,time);
      from->Addedge(v2,edge);
      to->Addedge(v1,edge);
    } 
      unsigned int totalCost=0;
      /*****************************************
	Compute the total cost for all vertices
      ******************************************/
      for(itVertices = vertices.begin(); itVertices != vertices.end(); 
	  itVertices++){
	totalCost += (itVertices->second)->GetVertexCost();
      }
      cout << totalCost/2 <<endl;
      /*****************************************
	Compute the minimum cost for all vertices
      ******************************************/
      unsigned int minTotalCost = 0;
      g.SetCostMST();
      for(itVertices2 = vertices.begin(); itVertices2 != vertices.end(); 
	  itVertices2++){
	minTotalCost += (itVertices2->second)->GetDist();
      }
      g.ResetDist();  
      cout << minTotalCost <<endl;
      cout << totalCost/2 - minTotalCost <<endl;
      /*****************************************
	Compute the minimum toal time for all vertices
      ******************************************/
      unsigned int totalDist = 0;
      for(itVertices = vertices.begin(); itVertices != vertices.end(); 
	  itVertices++){
	g.SetNearestDist(itVertices->second);
	for(itVertices2 = vertices.begin(); itVertices2 != vertices.end(); 
	    itVertices2++){
	  totalDist += (itVertices2->second)->GetDist();
	}
	//After computing the toatal distance, reset the variable distance for each vertex.
	g.ResetDist();  	
      }
      cout << totalDist <<endl;
      /*****************************************
	Compute the toal time for all vertices in MST
      ******************************************/
      unsigned int totalMSTDist = 0;
      for(itVertices = vertices.begin(); itVertices != vertices.end(); 
	  itVertices++){
	g.SetMSTDist(itVertices->second);
	for(itVertices2 = vertices.begin(); itVertices2 != vertices.end(); 
	    itVertices2++){
	  totalMSTDist += (itVertices2->second)->GetDist();
	}
	//After computing the toatal distance, reset the variable distance for each vertex.
	g.ResetDist();  	
      }
      cout << totalMSTDist <<endl;  
      cout << totalMSTDist-totalDist <<endl;  
      return EXIT_SUCCESS;
}

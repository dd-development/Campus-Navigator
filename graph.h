// graph.h
//
// Weighted graph class using adjacency list representation (implemented with an unordered map of vertex keys and vectors of edge structs as values)
// Vertices and weights are templated
//

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <unordered_map>

#pragma once

using namespace std;

template<typename VertexT, typename WeightT>

class graph {
  private:
    int vertCount; // Counter variables for vertices and edges
    int edgeCount;

    struct EdgeData { // We will use this struct for edges
      VertexT toVert;
      WeightT toWeight;
    };

    vector<VertexT> Vertices; // Vector storing the vertices to keep track of unique vertices
    unordered_map<VertexT, vector<EdgeData>> edgeMap; // Unordered map of vertex keys and edge data vectors as values

  public:
    
    // Constructor
    //
    // Sets vertCount and edgeCount to 0
    graph() {
      vertCount = 0;
      edgeCount = 0;
    }

    // NumVertices
    //
    // Returns the # of vertices currently in the graph.
    int NumVertices() const {
      return vertCount;
    }

    // NumEdges
    //
    // Returns the # of edges currently in the graph.
    int NumEdges() const {
      return edgeCount;
    }

    // addVertex
    //
    // If vertex exists in graph, returns false. Else, adds vertex to graph and returns true
    bool addVertex(VertexT v) {
      if (edgeMap.count(v)) { // count() returns 1 if the vertex is in the map already and 0 otherwise
        return false; // Return false if it's 1
      }

      vector<EdgeData> blankVectorOfEdges; // Creates a blank vector of edges
      edgeMap.emplace(v, blankVectorOfEdges); // Adds vertex with no edges to map

      Vertices.push_back(v); // Add new vertex to Vertices vector to keep track of unique vertices

      vertCount++; // Make sure to add one to vertCount
      return true; // Return true after adding
    }

    // addEdge
    //
    // Adds the edge (from, to, weight) to the graph, and returns
    // true. If the vertices do not exist, returns false.
    //
    // NOTE: if the edge already exists, the existing edge weight
    // is overwritten with the new edge weight.
    bool addEdge(VertexT from, VertexT to, WeightT weight) {
      if (!edgeMap.count(from) || !edgeMap.count(to)) { // If we can't find from or to vertices do nothing and return false
        return false;
      }

      vector<EdgeData> *searchVector = &edgeMap.at(from); // Using pointer here to modify actual vector in graph's edgeMap
      for (size_t i = 0; i < searchVector->size(); i++) {
        if (searchVector->at(i).toVert == to) { // If the edge already exists
          searchVector->at(i).toWeight = weight; // Overwrite the edge in the vector of edge data in the edgeMap

          return true; // Return true
        }
      }

      EdgeData newEdge; // If edge doesn't exist yet, make a new one
      newEdge.toVert = to;
      newEdge.toWeight = weight;

      searchVector->push_back(newEdge); // Push edge into vector in the edgeMap
      
      edgeCount++; // Raise the edgeCount

      return true; // Return true
    }

    // getWeight
    //
    // Returns the weight associated with a given edge.  If
    // the edge exists, the weight is returned via the reference
    // parameter and true is returned.  If the edge does not
    // exist, the weight parameter is unchanged and false is
    // returned.
    bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
      if (!edgeMap.count(from) || !edgeMap.count(to)) { // Return false if either vertex doesn't exist
        return false;
      }

      vector<EdgeData> searchVector = edgeMap.at(from); // const functions do not like [] indexing so we must use at()
      for (size_t i = 0; i < searchVector.size(); i++) {
        if (searchVector.at(i).toVert == to) { // Once we find the edge, set weight param to the weight in the edge data struct
          weight = searchVector.at(i).toWeight; // weight returned here technically

          return true; // Return true
        }
      }
      
      return false; // Return false if we didn't find the edge
    }

    // neighbors
    //
    // Returns a set containing the neighbors of v, i.e. all
    // vertices that can be reached from v along one edge.
    // Since a set is returned, the neighbors are returned in
    // sorted order; use foreach to iterate through the set.
    set<VertexT> neighbors(VertexT v) const {
      set<VertexT> S; // We will fill this set and return it

      if (!edgeMap.count(v)) { // If the vertex is not in the map, return empty set
        return S;
      }

      vector<EdgeData> searchVector = edgeMap.at(v); // Get the vector of neighboring edges of v
      for (size_t i = 0; i < searchVector.size(); i++) {
        S.insert(searchVector.at(i).toVert); // Iterate through and insert each toVert (what vertex the edge maps to) as a neighbor
      }

      return S; // Return set
    }

    // getVertices
    //
    // Returns a vector containing all the vertices currently in
    // the graph.
    vector<VertexT> getVertices() const {
      return Vertices;
    }
    
    // dump
    //
    // Dumps the internal state of the graph for debugging purposes.
    //
    // Example:
    //    graph<string,int>  G(26);
    //    ...
    //    G.dump(cout);  // dump to console
    void dump(ostream& output) const {
      output << "***************************************************" << endl;
      output << "********************* GRAPH ***********************" << endl;

      output << "**Num vertices: " << NumVertices() << endl;
      output << "**Num edges: " << NumEdges() << endl;

      output << endl;
      output << "**Vertices:" << endl;

      for (int i = 0; i < NumVertices(); i++) {
        output << " " << i << ". " << Vertices[i] << endl;
      }

      output << endl;
      output << "**Edges:" << endl;

      for (int i = 0; i < NumVertices(); i++) {
        output << " row " << Vertices.at(i) << ": ";

        set<VertexT> neighborSet = neighbors(Vertices.at(i));
        WeightT edgeWeight;

        for (int j = 0; j < NumVertices(); j++) {
          if (!neighborSet.count(Vertices.at(j))) {
            output << "F ";
          } 
          else {
            getWeight(Vertices.at(i), Vertices.at(j), edgeWeight);
            output << "(T," << edgeWeight << ") ";
          }
        }

        output << endl;
      }
      
      output << "**************************************************" << endl;
    }
};
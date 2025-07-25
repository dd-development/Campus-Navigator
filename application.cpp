// application.cpp
//
// Implements a bidirectional graph from OSM data and TinyXML tools to find the shortest path to the building closest to the center point
// between two given buildings along the footways of an area.
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <queue>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"


using namespace std;
using namespace tinyxml2;

// A comparator for use in priority queue for Dijkstra's
class prioritize
{
public:
  bool operator()(const pair<long long,double>& p1, const pair<long long,double>& p2) const
  {
    return p1.second > p2.second; 
  }
};

const double INF = numeric_limits<double>::max(); // GLOBAL INFINITY CONSTANT!

// Searches Buildings vector for searchTerm by abbreviations first and then full name and returns the index of matching info in Buildings if found
int searchBuilding(vector<BuildingInfo>& Buildings, string searchTerm) {
  for (int i = 0; i < static_cast<int>(Buildings.size()); i++) { // First loop
    if (Buildings.at(i).Abbrev.find(searchTerm) != string::npos) { // Searches abbreviations
      return i; // If we found a match, return the BuildingInfo for the match
    }
  }

  for (int i = 0; i < static_cast<int>(Buildings.size()); i++) { // Second loop
    if (Buildings.at(i).Fullname.find(searchTerm) != string::npos) { // Searches full names
      return i; // If we found a match, return the BuildingInfo for the match
    }
  }

  return -1; // Return -1 (invalid index) if we didn't find a match
}

// Given a midpoint, returns index of building in Buildings closest to center
int getCenterBuildingIndex(vector<BuildingInfo>& Buildings, Coordinates midpoint, set<string>& invalidCenters) {
  double midLat = midpoint.Lat; // Save midpoint lat and lon for ease of use
  double midLon = midpoint.Lon;

  double distance; // We will store our distance calculations here
  double min = INF; // min starts as globally defined infinity
  int minIndex = -1; // Keeps track of current min index in Buildings, set to -1 initially just in case Buildings is empty (it shouldn't be)

  // Here's our little min algorithm at work
  for (int i = 0; i < static_cast<int>(Buildings.size()); i++) { // Loop through buildings
    if (invalidCenters.count(Buildings.at(i).Fullname)) { // Count returns 1 if this Building's name is present in the invalidCenters set
      continue; // Skip this building and move on to the next
    }

    distance = distBetween2Points(midLat, midLon, Buildings.at(i).Coords.Lat, Buildings.at(i).Coords.Lon); // Find distance from midpt and building

    if (distance < min) { // If distance is less than min
      min = distance; // Set min to distance
      minIndex = i; // Record index of new min
    }
  }

  return minIndex; // Return index of closest building
}

// Returns the Coordinates struct of the footway node that is closest to the building parameter
// This is a similar implementation as the above function but instead of returning the index it returns the Coordinates struct of the node
Coordinates getClosestNode(map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways, BuildingInfo building) {
  double buildLat = building.Coords.Lat;
  double buildLon = building.Coords.Lon;

  double distance;
  double min = INF;
  long long minID; // Stores ID of last minimum

  // Similar min algorithm as above, except we iterate over Footways vector (and proceeding vectors) by range
  for (auto i : Footways) {
    for (auto j : i.Nodes) { // !!! We are now in an individual FootwayInfo's Nodes vector !!! Iterate by range where j are IDs of Nodes in vectors!
      distance = distBetween2Points(buildLat, buildLon, Nodes.at(j).Lat, Nodes.at(j).Lon); // Use Nodes to find coords of footway node at ID
      
      if (distance < min) {
        min = distance;
        minID = j; // j is the ID of a new min node that we found in the Nodes vector (i.Nodes) of a FootwayInfo struct (i). Store it!
      }
    }
  }

  return Nodes.at(minID); // Returns the Coordinates struct at the ID of the minimum node in the map
}

// Runs Dijkstra's algorithm to get shortest weighted path from startV to endV, which is recorded in path vector passed by reference
// After that, returns the double holding the distance of the path
double Dijkstra(graph<long long, double>& G, long long startV, long long endV, vector<long long>& path) {
  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> unvisitedQ; // Here we have our priority queue
  vector<long long> graphVertices = G.getVertices(); // Store our vertices from graph in a vector
  map<long long, double> distances; // Map to distances from start vertex to each vertex (starts at INF for each vertex)
  map<long long, long long> predecessors; // Map to each vertex's predecessor along the shortest path
  set<long long> visitedVertices;

  for (auto v : graphVertices) { // This loop sets up all of the above containers for Dijkstra's, recall v is an ID (long long)
    distances.emplace(v, INF);
    predecessors.emplace(v, -1); // -1 indicates no predecessor yet
    unvisitedQ.push(make_pair(v, distances.at(v))); // All of these distances will be INF
  }

  // We begin at the start vertex, which has a distance of 0 to itself
  distances.at(startV) = 0;
  unvisitedQ.push(make_pair(startV, distances.at(startV))); // Push startV with distance 0 into queue

  long long currentV;
  set<long long> adjacentVertices;
  double edgeWeight;
  double altPathDistance;
  while (!unvisitedQ.empty()) {
    currentV = unvisitedQ.top().first; // Visit vertex with smallest distance from startV
    unvisitedQ.pop();

    if (distances.at(currentV) == INF) { // If we've reached the end of the queue
      break; // Leave the loop, we're done
    }
    else if (visitedVertices.count(currentV)) { // If currentV is in visited set (visited)
      continue; // Skip over currentV and run the loop again
    }
    else { // Mark currentV as visited otherwise
      visitedVertices.insert(currentV); // Add currentV to visited set
    }

    adjacentVertices = G.neighbors(currentV); // Get adjacent vertices to currentV
    for (auto adjV : adjacentVertices) {
      G.getWeight(currentV, adjV, edgeWeight); // Store weight of currentV-->ajV edge in edgeWeight
      altPathDistance = distances.at(currentV) + edgeWeight;

      // If a shorter path is found from startV to adjV, update adjV's distance and predecessor
      if (altPathDistance < distances.at(adjV)) {
        distances.erase(adjV);
        distances.emplace(adjV, altPathDistance);
        predecessors.erase(adjV);
        predecessors.emplace(adjV, currentV);

        unvisitedQ.push(make_pair(adjV, altPathDistance));
      }
    }

    if (currentV == endV) { // If we got to the endV, get out of the loop
      break;
    }
  }

  if (currentV != endV) { // If we are out of the loop and aren't on endV, we didn't reach it
    return -1; // -1 means we found nothing
  }

  path.push_back(endV); // Now we back trace through the predecessors starting from endV and push path into vector
  long long backTrace = predecessors.at(endV);
  while (backTrace != -1) {
    path.push_back(backTrace);
    backTrace = predecessors.at(backTrace);
  }

  return distances.at(endV); // Return the weight
}

//
// Standard application implemented here
//
void application(map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways, vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
  // Main application loop!
  while (true) { // Used to be person1Building != "#"
    string person1Building, person2Building;

    int firstIndex; // Index of first and second buildings, we use searchBuilding() to find them
    int secondIndex;
  
    Coordinates midpoint; // Use centerBetween2Points() to get midpoint between first and second buildings

    BuildingInfo p1Building; // We store the actual buildings in these variables
    BuildingInfo p2Building;
    BuildingInfo centerBuilding; // Use getCenterBuildingIndex() and store the building at that index as the center

    Coordinates p1Coords;
    Coordinates p2Coords;
    Coordinates centerCoords;

    set<string> invalidCenters; // If we ever run into an invalid center building, it gets stored here so we ignore it in getCenterBuildingIndex

    cout << endl;
    
    // FIND BUILDINGS 1 AND 2 -----------------------------------------------------------------------
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    if (person1Building == "#") { // On # input we must exit the loop
      break; // Goodbye, loop!
    }

    firstIndex = searchBuilding(Buildings, person1Building); // Search for first building index

    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    secondIndex = searchBuilding(Buildings, person2Building); // Search for second building index

    if (firstIndex == -1) { // If first building not found, output error and start main loop from beginning
      cout << "Person 1's building not found" << endl; // Error message
      continue; // Start loop over again and get new input
    }

    // Okay we found the first building...

    if (secondIndex == -1) { // If second building not found, output error and start main loop from beginning
      cout << "Person 2's building not found" << endl; // Error message
      continue; // Start loop over again and get new input
    }
    
    // Okay we found the second building...

    p1Building = Buildings.at(firstIndex); // Store both buildings as BuildingInfo structs now
    p2Building = Buildings.at(secondIndex);

    // -------------------------------------------------------------------------------------------
    
    // GET THE MIDPOINT BETWEEN BUILDINGS 1 AND 2 AND FIND CLOSEST BUILDING (CENTER) ----------------
    midpoint = centerBetween2Points(Buildings.at(firstIndex).Coords.Lat, Buildings.at(firstIndex).Coords.Lon, // Get midpoint coords
                Buildings.at(secondIndex).Coords.Lat, Buildings.at(secondIndex).Coords.Lon);
      
    centerBuilding = Buildings.at(getCenterBuildingIndex(Buildings, midpoint, invalidCenters)); // Find center building (closest to midpoint) index and store it
    // ^Recall that if Buildings is empty, this will error by trying to access an element at index -1 returned by getCenterBuildingIndex() 

    // -------------------------------------------------------------------------------------------

    // PRINT BUILDING 1, 2, AND CENTER INFO ----------------------------------------------------------------------
    cout << endl;

    cout << "Person 1's point:" << endl 
    << " " << p1Building.Fullname << endl
    << " (" << p1Building.Coords.Lat << ", " << p1Building.Coords.Lon << ")" << endl;

    cout << "Person 2's point:" << endl 
    << " " << p2Building.Fullname << endl
    << " (" << p2Building.Coords.Lat << ", " << p2Building.Coords.Lon << ")" << endl;

    cout << "Destination Building:" << endl 
    << " " << centerBuilding.Fullname << endl
    << " (" << centerBuilding.Coords.Lat << ", " << centerBuilding.Coords.Lon << ")" << endl;

    cout << endl;

    // END PRINTING ----------------------------------------------------------------------------------------------

    // GET THE NODES CLOSEST TO BUILDINGS 1, 2, AND CENTER ------------------------------------------
    p1Coords = getClosestNode(Nodes, Footways, p1Building); // Store each Coordinates struct returned by getClosestNode in respective variable
    p2Coords = getClosestNode(Nodes, Footways, p2Building);
    centerCoords = getClosestNode(Nodes, Footways, centerBuilding);
    
    // -------------------------------------------------------------------------------------------
    
    // PRINT NODE INFO FOR 1, 2, AND CENTER ----------------------------------------------------------------------
    cout << "Nearest P1 node:" << endl 
    << " " << p1Coords.ID << endl
    << " (" << p1Coords.Lat << ", " << p1Coords.Lon << ")" << endl;

    cout << "Nearest P2 node:" << endl 
    << " " << p2Coords.ID << endl
    << " (" << p2Coords.Lat << ", " << p2Coords.Lon << ")" << endl;

    cout << "Nearest destination node:" << endl 
    << " " << centerCoords.ID << endl
    << " (" << centerCoords.Lat << ", " << centerCoords.Lon << ")" << endl;

    cout << endl;

    // END PRINTING ----------------------------------------------------------------------------------------------

    // DIJKSTRA'S ALGORITHM ------------------------------------------------------------------------
    vector<long long> path1;
    vector<long long> path2;
    vector<long long> validPath; // This one checks if we can path from 1 to 2 directly, not both to center

    double totalDistance1 = Dijkstra(G, p1Coords.ID, centerCoords.ID, path1); // Call Dijkstra from 1 to center
    double totalDistance2 = Dijkstra(G, p2Coords.ID, centerCoords.ID, path2); // Call Dijkstra from 2 to center
    double validPathDistance = Dijkstra(G, p1Coords.ID, p2Coords.ID, validPath); // See if there is indeed a path from 1 to 2

    if (validPathDistance == -1) { // Checks if there is a valid path from 1 to 2, Dijkstra returns 0 if we could not find a path
      cout << "Sorry, destination unreachable" << endl;
      continue;
    }
    else if (totalDistance1 == -1 || totalDistance2 == -1) { // If 1 or 2 can't reach center, output error and find new center

      // FIND NEXT CLOSEST BUILDING ------------------------------------------------------------------------
      while (true) {
        cout << "At least one person was unable to reach the destination building. Finding next closest building..." << endl;
        cout << endl;
        invalidCenters.insert(centerBuilding.Fullname); // Store center building name in invalidCenters set so we don't use it next time

        centerBuilding = Buildings.at(getCenterBuildingIndex(Buildings, midpoint, invalidCenters)); // Establish new centerBuilding
        centerCoords = getClosestNode(Nodes, Footways, centerBuilding); // Establish new centerCoords

        cout << "New destination building: " << endl << " " << centerBuilding.Fullname << endl; // Print new destination building
        cout << " (" << centerBuilding.Coords.Lat << ", " << centerBuilding.Coords.Lon << ")" << endl;

        cout << "Nearest destination node: " << endl << " " << centerCoords.ID << endl; // Print new destination node
        cout << " (" << centerCoords.Lat << ", " << centerCoords.Lon << ")" << endl;

        cout << endl;

        path1.clear(); // Clear our path vectors to prepare for another round of Dijkstra's
        path2.clear();

        totalDistance1 = Dijkstra(G, p1Coords.ID, centerCoords.ID, path1); // Run Dijkstra's again and pray for a center building that works
        totalDistance2 = Dijkstra(G, p2Coords.ID, centerCoords.ID, path2);
        
        if (totalDistance1 == -1 || totalDistance2 == -1) { // If we get another bad center, we run the loop again
          continue;
        }

        break; // Otherwise, we break and move on
      }

      // ----------------------------------------------------------------------------------------
    }

    cout << "Person 1's distance to dest: " << totalDistance1 << " miles" << endl;
    cout << "Path: ";
    for (size_t i = path1.size() - 1; i > 0; i--) { // Goes down to index 1 and prints index 0 after loop without ->
      cout << path1.at(i) << "->";
    }
    cout << path1.at(0) << endl;
    cout << endl;

    cout << "Person 2's distance to dest: " << totalDistance2 << " miles" << endl;
    cout << "Path: ";
    for (size_t i = path2.size() - 1; i > 0; i--) { // Goes down to index 1 and prints index 0 after loop without ->
      cout << path2.at(i) << "->";
    }
    cout << path1.at(0) << endl;
  }
  // --------------------------------------------------------------------------------------------
}

int main() {
  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;

  // ADD VERTICES TO OUR GRAPH ----------------------------------------------------------------------
  graph<long long, double> G;
  
  for (auto i : Nodes) { // For every pair in the Nodes map (long long ID, coordinate structure)
    G.addVertex(i.first); // RECALL: Map is just a pair structure with (key, value) where key is stored as .first and value is stored as .second
  }

  // ---------------------------------------------------------------------------------------------

  // ADD EDGES TO OUR GRAPH -------------------------------------------------------------------------
  long long fromID;
  long long toID;

  Coordinates fromCoords;
  Coordinates toCoords;

  double weightAsDistance;
  for (auto i : Footways) {

    for (size_t j = 0; j < i.Nodes.size() - 1; j++) { // Loop until second to last element
      fromID = i.Nodes.at(j); // Gets ID for node j
      toID = i.Nodes.at(j + 1); // Gets ID for node j + 1

      fromCoords = Nodes.at(fromID); // Gets Coordinates structure for node j (from)
      toCoords = Nodes.at(toID); // Gets Coordinates structure for node j + 1 (to)

      weightAsDistance = distBetween2Points(fromCoords.Lat, fromCoords.Lon, toCoords.Lat, toCoords.Lon); // Calculate distance

      // Bidirectional edge adding
      G.addEdge(fromID, toID, weightAsDistance); // Add from-->to edge to graph with calculated distance as weight
      G.addEdge(toID, fromID, weightAsDistance); // Add to-->from edge to graph with calculated distance as weight
    }
  }

  // ---------------------------------------------------------------------------------------------

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  cout << endl << "LIST OF BUILDINGS" << endl << "------------------------------" << endl;

  for (BuildingInfo b : Buildings) {
    cout << "NAME: " << b.Fullname << ", ABBREVIATION: " << b.Abbrev << endl;
  }

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}

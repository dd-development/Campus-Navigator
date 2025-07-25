# Campus Navigator
A C++ application for locating and pathing towards the optimal building for two students to meet at based on their starting locations on campus.

## Overview
This project was designed primarily as a backend implementation that reads in data from any [OpenStreetMap](https://www.openstreetmap.org/#map=13/40.07827/-88.21906) .osm file export.

Theoretically, this application could be be used with any valid .osm file (as long as there are valid named buildings/locations in the corresponding area), which means it is not limited to just college campuses (you would just need to remove the check for "university" tagged buildings in osm.cpp to do this).

However, the intimate nature of campus infrastructure allows it to serve as great, lightweight input data to showcase the functionality of the application. An OpenStreetMap export of DePaul University's Lincoln Park campus has been provided within the repo (depaul.osm) for example usage/testing, but feel free to use the application with OpenStreetMap exports of other college campuses as well.

## Design and Usage
1. The application can be built and ran using the makefile (*make build, make run*).
2. Upon running the application, the user can input a .osm filename to be read in as map data via the console interface.
3. The OpenStreetMap XML data is parsed via the [TinyXML2](https://github.com/leethomason/tinyxml2) library and read into an adjacency list graph structure.
4. Nodes are stored as vertices, footways (viable paths) are stored as edges.
5. Buildings are assigned a latitude and longitude based on the average of all of the nodes that border that building, and are then stored in a vector.
6. The user can input two different building names/abbreviations that are located on campus (note that some buildings might not have abbreviations depending on the input data).
7. If the input buildings are valid, the building nearest to the midpoint between the two starting buildings is located and designated as the "meeting destination".
8. Dijkstra's algorithm is ran from both starting buildings to find the respective shortest paths to the "meeting destination".
9. The path from each starting building to the center is listed node by node in order of traversal, along with the total distance (in miles) of the resulting path.
10. If there is no valid path from either starting point, a new "meeting destination" is chosen in order of the next closest building to the midpoint, and the process repeats until valid paths are found or there are no more buildings left to choose.

## Example


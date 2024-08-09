# Search Package
A package for maintaining a probability distribution function and generating points from this function to explore

Note: This package was planned to be used for performing object search during the Atonomous Navigation mission,
but was never completed due to time constraints.

## Files
**launch/search.launch.py**
- This file is used to test the search algorithm. It launches two nodes, one being the search node and another that is used to test the node. The test node loops through generated points to "explore" them, then requests more points and does it again.

**search_pkg/mod_search_object.py**
- This is the "module" that contains the abstracted search class the search node uses to manage point generation.

**search_pkg/ros2_search_node.py**
- This is the search node that should be used for search generated points. The points are returned as UrcCustomPath (array of UrcCustomPoint)

## How to use the Search Node
**generate_point**
- Send a UrcCustomPoint to this topic to generate a search area around this point (of a radius given before). N-samples will be returned to the **point_array** topic.
- Note: this resets the algorithm to generate a circle around the provided point. Only use at the beginning of your search

**point_array**
- This topic will receive UrcCustomPath from the search_node (points in order) for the rover to explore

**generate_flag**
- Publish any Int8 to this node to generate more points to the point array topic (without changing and resetting the search)

**search_point**
- To update the PDF (Probability Distribution Function), you must pass the rovers position to this topic periodically (every 0.75 seconds preferably). This topic takes a UrcCustomPoint, and updates the search area around this point.

**reset_flag**
- Publish any Int8 to reset the algorithm. This should not be used!

**display_flag**
- Publish any Int8 to display the algorithm in Matplotlib.
- Warning, this will brick the algorithm because matplotlib sends it into a while-loop.

## Limitations
This package was not thoroughly tested, nor was it ever used on actual hardware. As is, it serves as a proof-of-concept for
the algorithms and functionalities defined within.
# Voronoi diagram creation (work-in-progress)
This implements Fortune's algorithm for determining the Voronoi diagram of a collection of points.

## Voronoi diagrams
A Voronoi diagram processes a given list of points, where space is divided into cells surrounding each point. The area the cell covers is such that everything enclosed in the cell is closer to the associated point than to any other point.

## Fortune's algorithm
The algorithm works with an event queue, where events occur at discrete steps. These events are processed in the order that they occur, and we define a so-called sweepline that has the $y$ value of the current event being processed (see below for more information). There are two event types:
- Site creation events.
- Edge intersection events.

### Site creation
The site creation events are all known ahead of time, and loaded into the queue when the program starts. This just corresponds to each of the given list of points being added to the diagram, which will be done one at at time based on their $y$-component (largest $y$ value first).

### Edge intersection
Edge intersection events can be detected during a site creation event, or another edge intersection event. These correspond to where two edges in the Voronoi diagram will intersect.

### Sweepline
The mediator of the events process is known as the sweepline. This is a horizontal line moving from top to bottom through the $y$ values of all the events.
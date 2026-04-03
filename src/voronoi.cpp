#include <math.h>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "voronoi.hh"

namespace voronoi {

int event_id = 0;
int coast_id = 0;

float parabola_y(float x, Point focus, float directrix) { return pow(x - focus.x, 2) / (2 * (focus.y - directrix)) + (focus.y + directrix) / 2; }

// ----- point definitions start -----
Point::Point() {
	x = 0;
	y = 0;
}

Point::Point(float px, float py) {
	x = px;
	y = py;
}

Point Point::operator+(Point p) const { return Point(x + p.x, y + p.y); }
Point Point::operator-(Point p) const { return Point(x - p.x, y - p.y); }

float Point::norm2() { return x * x + y * y; }
float Point::norm() { return sqrt(norm2()); }
// ----- point definitions end -----

// ----- edge definitions start -----
Edge::Edge(Point start, Point end) : line(start, end) {}
// ----- edge definitions end -----

// ----- event definitions start -----
Event::Event(int event_type, Point event_point, float sweepline) {
	type = event_type;
	point = event_point;
	timestamp = sweepline;

	id = event_id++; // give this a unique identifier
}

// the event with largest timestamp occurs first
bool Event::operator<(const Event &event) const {
	return timestamp < event.timestamp;
}
// ----- event definitions end -----

// ----- coast definitions start -----
Coast::Coast(Point start, int coast_type) : range_x(__FLT_MIN__, __FLT_MAX__) {
	focus = start;
	type = coast_type;

	id = coast_id++;
}

Coast Coast::copy() {
	Coast copied = Coast(*this);
	copied.id = coast_id++;

	return copied;
}

// ----- coast definitions end -----

int Voronoi::get_coast_above(Point point) {
	// TODO: deal with sites that have the same y component
	int min_index = -1;
	for (size_t i = 0; i < coastline.size(); i++) {
		if (coastline[i].type == COAST_EDGE) continue;
		if (coastline[i].range_x.contains(point.x)) {
			if (min_index == -1) min_index = i;
			else if (parabola_y(point.x, coastline[i].focus, point.y) < parabola_y(point.x, coastline[min_index].focus, point.y)) min_index = i;
		}
	}

	return min_index;
}

void Voronoi::get_intersection_event(int index) {
	if (index < 1 || index >= (int)coastline.size() - 1) return; // the left-most arc won't get clamped

	Coast arc = coastline[index];

	// the edges to the side of the arc
	Coast next = coastline[index + 1];
	Coast prev = coastline[index - 1];

	Point dist = next.focus - prev.focus;
	float val = next.direction.x * prev.direction.y - next.direction.y * prev.direction.x;

	if (val == 0) {
		printf("exit 0\n");
		return; // TODO: this should not happen
	}
	float t = (dist.y * next.direction.x - dist.x * next.direction.y) / val;
	float c = (dist.y * prev.direction.x - dist.x * prev.direction.y) / val;

	// next_end = next.focus + t * next.direction
	Point intersection = Point(prev.focus.x + t * prev.direction.x, prev.focus.y + t * prev.direction.y);
	if (t < 0 || c < 0) {
		// if the rays have to move in reverse to intersect
		return;
	}

	float sweepline = intersection.y - sqrt(pow(intersection.x - arc.focus.x, 2) + pow(intersection.y - arc.focus.y, 2));

	if (sweepline > current_sweep) {
		printf("exit old\n"); // TODO: this should not happen
		return;
	}

	Event new_event = Event(EVENT_INTERSECT, intersection, sweepline);
	new_event.event_target_id = arc.id;

	events.push(new_event);
}

void Voronoi::do_site(Event event) {
	// create a new arc to add to the coastline
	Coast new_arc = Coast(event.point, COAST_ARC);

	int coast_above = get_coast_above(event.point); // find the coast that is directly above this point
	if (coast_above == -1) {
		// this should only happen when the coast is empty (when this is the first event)
		coastline.push_back(Coast(new_arc));
	} else {
		// the arc above will be split in two, where the feasible ranges will extend from the newly inserted point
		Coast left_split = coastline[coast_above].copy();
		Coast right_split = coastline[coast_above].copy();
		left_split.range_x.end = event.point.x;
		right_split.range_x.start = event.point.x;

		// where the new arc intersects with the arc above
		Point intersection_point = Point(event.point.x, parabola_y(event.point.x, coastline[coast_above].focus, event.timestamp));
		Coast left = Coast(intersection_point, COAST_EDGE);
		Coast right = Coast(intersection_point, COAST_EDGE);

		Point dr = new_arc.focus - coastline[coast_above].focus;
		left.direction = Point(dr.y, -dr.x);  // rotate by -pi/2
		right.direction = Point(-dr.y, dr.x); // rotate by pi/2

		delete_at(coastline, coast_above);

		insert_at(coastline, coast_above, left_split);
		insert_at(coastline, coast_above + 1, left);
		insert_at(coastline, coast_above + 2, new_arc);
		insert_at(coastline, coast_above + 3, right);
		insert_at(coastline, coast_above + 4, right_split);

		// printf("added %.1f, %.1f and %.1f, %.1f (%.1f, %.1f), (%.1f, %.1f)\n", left.direction.x, left.direction.y, right.direction.x, right.direction.y, new_arc.focus.x, new_arc.focus.y, coastline[coast_above].focus.x, coastline[coast_above].focus.y);
		get_intersection_event(coast_above);	 // the left side
		get_intersection_event(coast_above + 4); // the right side
	}
}

void Voronoi::do_intersect(Event event) {
	int index = -1;
	for (size_t i = 2; i < coastline.size() - 2; i++) {
		if (coastline[i].id == event.event_target_id) {
			index = i;
			break;
		}
	}
	if (index == -1) return;

	Coast edge_left = Coast(coastline[index - 1]);
	Coast edge_right = Coast(coastline[index + 1]);

	delete_at(coastline, index + 1); // delete left edge
	delete_at(coastline, index);	 // delete collapsed arc
	delete_at(coastline, index - 1); // delete right edge

	// these arcs now need an edge between them
	Coast arc_left = coastline[index - 2];
	Coast arc_right = coastline[index - 1];

	Coast new_edge = Coast(event.point, COAST_EDGE);

	Point dr = arc_right.focus - arc_left.focus;
	new_edge.direction = Point(dr.y, -dr.x);

	insert_at(coastline, index - 1, Coast(new_edge));

	// these edges are apart of the voronoi diagram
	edges.push_back(Edge(event.point, edge_left.focus));
	edges.push_back(Edge(event.point, edge_right.focus));

	get_intersection_event(index);	   // the next arc
	get_intersection_event(index - 2); // the previous arc
}

Voronoi::Voronoi(float **pts, int point_count, Point min, Point max) : bounds(min, max) {
	for (int i = 0; i < point_count; i++) {
		points.push_back(Point(pts[i][0], pts[i][1]));
	}
}

Voronoi::Voronoi() : bounds(Point(0, 0), Point(0, 0)) {}

void Voronoi::next_step() {
	if (!events.empty()) {

		Event next_event = events.top(); // get next event from the priority queue
		events.pop();					 // remove this event

		current_sweep = next_event.timestamp;

		if (next_event.type == EVENT_SITE) do_site(next_event);
		else do_intersect(next_event);

		proceessed_events++;
	}
}

void Voronoi::add_remaining_edges() {
	for (size_t i = 0; i < coastline.size(); i++) {
		if (coastline[i].type == COAST_EDGE) {
			float start_x = coastline[i].focus.x;
			float start_y = coastline[i].focus.y;
			float direction_x = coastline[i].direction.x;
			float direction_y = coastline[i].direction.y;

			float x1 = (bounds.start.x - start_x) / direction_x;
			float x2 = (bounds.end.x - start_x) / direction_x;

			float y1 = (bounds.start.y - start_y) / direction_y;
			float y2 = (bounds.end.y - start_y) / direction_y;

			if ((x1 < 0 && x2 < 0) || (y1 < 0 && y2 < 0)) continue;

			float x_used = x1;
			float y_used = y1;
			int use_min_x = 1;
			int use_min_y = 1;
			if (x1 < 0) {
				x_used = x2;
				use_min_x = 0;
			}
			if (y1 < 0) {
				y_used = y2;
				use_min_y = 0;
			}

			if (x_used < y_used) {
				if (use_min_x) {
					edges.push_back(Edge(coastline[i].focus, Point(bounds.start.x, start_y + direction_y * x_used)));
				} else {
					edges.push_back(Edge(coastline[i].focus, Point(bounds.end.x, start_y + direction_y * x_used)));
				}
			} else {
				if (use_min_y) {
					edges.push_back(Edge(coastline[i].focus, Point(start_x + direction_x * y_used, bounds.start.y)));
				} else {
					edges.push_back(Edge(coastline[i].focus, Point(start_x + direction_x * y_used, bounds.end.y)));
				}
			}
		}
	}
}

void Voronoi::clip_edges() {
	for (size_t i = 0; i < edges.size(); i++) {

		Point d_start_min = bounds.start - edges[i].line.start;
		Point d_start_max = bounds.end - edges[i].line.start;
		Point d_end_min = bounds.start - edges[i].line.end;
		Point d_end_max = bounds.end - edges[i].line.end;

		// this edge is completely outside the bounds
		if ((d_start_min.x > 0 && d_end_min.x > 0) ||
			(d_start_max.x < 0 && d_end_max.x < 0) ||
			(d_start_min.y > 0 && d_end_min.y > 0) ||
			(d_start_max.y < 0 && d_end_max.y < 0)) {
			delete_at(edges, i);
			i--;
			continue;
		}

		Point direction = edges[i].line.end - edges[i].line.start;
		float grad = direction.y / direction.x;

		if (d_start_min.x > 0) {
			edges[i].line.start.x += d_start_min.x;
			edges[i].line.start.y += d_start_min.x * grad;
		} else if (d_start_max.x < 0) {
			edges[i].line.start.x = bounds.end.x;
			edges[i].line.start.y += d_start_max.x * grad;
		}

		if (d_end_min.x > 0) {
			edges[i].line.end.x += d_end_min.x;
			edges[i].line.end.y += d_end_min.x * grad;
		} else if (d_end_max.x < 0) {
			edges[i].line.end.x += d_end_max.x;
			edges[i].line.end.y += d_end_max.x * grad;
		}

		if (d_start_min.y > 0) {
			edges[i].line.start.y += d_start_min.y;
			edges[i].line.start.x += d_start_min.y / grad;
		} else if (d_start_max.y < 0) {
			edges[i].line.start.y += d_start_max.y;
			edges[i].line.start.x += d_start_max.y / grad;
		}

		if (d_end_min.y > 0) {
			edges[i].line.end.y += d_end_min.y;
			edges[i].line.end.x += d_end_min.y / grad;
		} else if (d_end_max.y < 0) {
			edges[i].line.end.y += d_end_max.y;
			edges[i].line.end.x += d_end_max.y / grad;
		}
	}
}

void Voronoi::solve_full() {
	solve();

	printf("solving with %li points\n", events.size());
	while (!events.empty()) {
		next_step();
	}
	printf("processed %i events\n", proceessed_events);

	add_remaining_edges();
	clip_edges();
}

void Voronoi::solve() {
	if (points.size() < 1) return;

	current_sweep = points[0].y;

	float x_min = points[0].x;
	float x_max = points[0].x;
	float y_min = points[0].y;
	float y_max = points[0].y;

	// add all the sites as events that will take place
	for (size_t i = 0; i < points.size(); i++) {
		events.push(Event(EVENT_SITE, points[i], points[i].y));

		if (points[i].y > y_max) y_max = points[i].y;
		else if (points[i].y < y_min) y_min = points[i].y;

		if (points[i].x > x_max) x_max = points[i].x;
		else if (points[i].x < x_min) x_min = points[i].x;
	}

	float x_midpoint = (x_max + x_min) / 2;
	float y_above = y_max + (y_max - y_min) / 2; // a height significantly higher than the highest point

	current_sweep = y_above;

	// TODO: this is temporary
	events.push(Event(EVENT_SITE, Point(x_midpoint, current_sweep), current_sweep)); // this is to avoid the issues with the first generated sites
}

}; // namespace voronoi
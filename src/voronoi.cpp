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
Edge::Edge(Point start, Point end) {
	this->start = start;
	this->end = end;
}
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
Coast::Coast(Point start, int coast_type) {
	focus = start;
	range_start = __FLT_MIN__;
	range_end = __FLT_MAX__;
	type = coast_type;

	id = coast_id++;
}

Coast *Coast::split_left(Event event) {
	Coast *left_split = new Coast(focus, COAST_ARC);
	left_split->range_start = range_start;
	left_split->range_end = event.point.x;

	return left_split;
}

Coast *Coast::split_right(Event event) {
	Coast *right_split = new Coast(focus, COAST_ARC);
	right_split->range_end = range_end;
	right_split->range_start = event.point.x;

	return right_split;
}

void Coast::get_intersection_event(Voronoi *solver) {
	if (prev == nullptr || next == nullptr || parent == nullptr || parent->parent == nullptr) return;

	Coast *edge_left = parent->parent;
	Coast *edge_right = parent;
	printf("edges: %i, %i\n", edge_left, edge_right);

	Point dist = edge_right->focus - edge_left->focus;
	float val = edge_right->direction.x * edge_left->direction.y - edge_right->direction.y * edge_left->direction.x;

	if (val == 0) {
		printf("exit 0\n");
		return; // TODO: this should not happen
	}
	float t = (dist.y * edge_right->direction.x - dist.x * edge_right->direction.y) / val;
	float c = (dist.y * edge_left->direction.x - dist.x * edge_left->direction.y) / val;

	// edge_right_end = edge_right->focus + t * edge_right->direction
	Point intersection = Point(edge_left->focus.x + t * edge_left->direction.x, edge_left->focus.y + t * edge_left->direction.y);
	if (t < 0 || c < 0) {
		// if the rays have to move in reverse to intersect
		return;
	}

	float sweepline = intersection.y - sqrt(pow(intersection.x - focus.x, 2) + pow(intersection.y - focus.y, 2));

	if (sweepline > solver->current_sweep) {
		printf("exit old\n"); // TODO: this should not happen
		return;
	}

	Event *new_event = new Event(EVENT_INTERSECT, intersection, sweepline);
	new_event->event_target = this;
	associated_event = new_event;

	solver->events.push(new_event);
}

// the arc above will be split in two, where the feasible ranges will extend from the newly inserted point
void Coast::split(const Event &event, Voronoi *solver) {
	// where the new arc intersects with the arc above
	Point intersection_point = Point(event.point.x, parabola_y(event.point.x, focus, event.timestamp));
	Point dr = event.point - focus; // the direction between the new site and the site that's arc it intersects

	// steps (no particular order):
	// 1. this coast item becomes the left edge
	// 2. the left child becomes the left split
	// 3.1. the right child becomes the right edge
	// 3.2. the right edge's left child becomes the new arc
	// 3.3  the right edge's right child becomes the right split

	left = split_left(event);						   // step 2
	right = new Coast(intersection_point, COAST_EDGE); // step 3.1
	right->direction = Point(-dr.y, dr.x);
	right->left = new Coast(event.point, COAST_ARC); // step 3.2
	right->right = split_right(event);				 // step 3.3

	// step 1
	focus = intersection_point;
	type = COAST_EDGE;
	direction = Point(dr.y, -dr.x);

	// set ranges
	range_start = left->range_start;
	range_end = right->range_end;
	right->range_start = right->left->range_start;
	right->range_end = right->right->range_end;

	left->parent = this;
	right->parent = this;
	right->left->parent = right;
	right->right->parent = right;

	left->prev = prev;
	left->next = right->left;
	right->left->prev = left;
	right->left->next = right->right;
	right->right->prev = right->left;
	right->right->next = next;

	left->get_intersection_event(solver);
	right->right->get_intersection_event(solver);
}

void draw_tree(Coast *coast, int depth) {
	for (int i = 0; i < depth; i++) {
		printf("    ");
	}

	printf("[%s] %i\n", coast->type == 0 ? "arc" : "edg", coast);

	if (coast->right != nullptr) draw_tree(coast->right, depth + 1);
	if (coast->left != nullptr) draw_tree(coast->left, depth + 1);
}

void Coast::intersection(const Event &event, Voronoi *solver) {
	// steps:
	// 1. the target's left edge becomes a new edge
	// 2. the target's right edge gets deleted
	// 3. the target gets deleted

	Coast *edge_left = parent->parent;
	Coast *edge_right = parent;

	// these edges are apart of the voronoi diagram
	solver->edges.push_back(Edge(event.point, edge_left->focus));
	solver->edges.push_back(Edge(event.point, edge_right->focus));

	// the arcs now need an edge between them

	Coast new_edge = Coast(event.point, COAST_EDGE);
	Point dr = next->focus - prev->focus;
	new_edge.direction = Point(dr.y, -dr.x);

	edge_left->focus = new_edge.focus;
	edge_left->direction = new_edge.direction;

	edge_left->right = edge_right->right;
	edge_right->right->parent = edge_left;

	next->prev = prev;
	prev->next = next;

	if (prev != nullptr) prev->get_intersection_event(solver); // the previous arc
	if (next != nullptr) next->get_intersection_event(solver); // the next arc

	draw_tree(solver->root, 0);

	delete edge_right;
	delete this;
}
// ----- coast definitions end -----

// ----- voronoi definitions start -----

// get the coast arc above the given point
Coast *Voronoi::get_coast_above(Point point) {
	Coast *above = root;

	while (1) {
		if (above->left != nullptr && above->left->range_start < point.x && above->left->range_end > point.x) {
			above = above->left;
		} else if (above->right != nullptr) {
			above = above->right;
		} else {
			break;
		}
	}

	return above;
}

void Voronoi::do_site(Event &event) {
	Coast *coast_above = get_coast_above(event.point); // find the coast that is directly above this point
	printf("c: %i -> %i\n", coast_above, coast_above->parent);
	coast_above->split(event, this);
}

void Voronoi::do_intersect(Event &event) {
	// TODO: check validity of event target
	event.event_target->intersection(event, this);
}

Voronoi::Voronoi(float **pts, int point_count, float x_min, float x_max, float y_min, float y_max) {
	for (int i = 0; i < point_count; i++) {
		points.push_back(Point(pts[i][0], pts[i][1]));
	}

	bounds_min = Point(x_min, y_min);
	bounds_max = Point(x_max, y_max);
}

Voronoi::Voronoi() {
}

void Voronoi::next_step() {
	if (!events.empty()) {

		Event *next_event = events.top(); // get next event from the priority queue
		events.pop();					  // remove this event

		if (!next_event->invalid) {
			current_sweep = next_event->timestamp;

			if (next_event->type == EVENT_SITE) do_site(*next_event);
			else do_intersect(*next_event);

			proceessed_events++;
		}
		delete next_event;
	}
}

void Voronoi::add_edge(Coast &edge) {
	float start_x = edge.focus.x;
	float start_y = edge.focus.y;
	float direction_x = edge.direction.x;
	float direction_y = edge.direction.y;

	float x1 = (bounds_min.x - start_x) / direction_x;
	float x2 = (bounds_max.x - start_x) / direction_x;

	float y1 = (bounds_min.y - start_y) / direction_y;
	float y2 = (bounds_max.y - start_y) / direction_y;

	if ((x1 < 0 && x2 < 0) || (y1 < 0 && y2 < 0)) return;

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
			edges.push_back(Edge(edge.focus, Point(bounds_min.x, start_y + direction_y * x_used)));
		} else {
			edges.push_back(Edge(edge.focus, Point(bounds_max.x, start_y + direction_y * x_used)));
		}
	} else {
		if (use_min_y) {
			edges.push_back(Edge(edge.focus, Point(start_x + direction_x * y_used, bounds_min.y)));
		} else {
			edges.push_back(Edge(edge.focus, Point(start_x + direction_x * y_used, bounds_max.y)));
		}
	}
}

void Voronoi::get_edge(Coast *coast) {
	if (coast->left != nullptr || coast->right != nullptr) {
		add_edge(*coast);

		if (coast->left != nullptr) {
			get_edge(coast->left);
		}
		if (coast->right != nullptr) {
			get_edge(coast->right);
		}
	}

	delete coast;
}

void Voronoi::add_remaining_edges() {
	get_edge(root);
}

void Voronoi::clip_edges() {
	for (size_t i = 0; i < edges.size(); i++) {

		Point d_start_min = bounds_min - edges[i].start;
		Point d_start_max = bounds_max - edges[i].start;
		Point d_end_min = bounds_min - edges[i].end;
		Point d_end_max = bounds_max - edges[i].end;

		// this edge is completely outside the bounds
		if ((d_start_min.x > 0 && d_end_min.x > 0) ||
			(d_start_max.x < 0 && d_end_max.x < 0) ||
			(d_start_min.y > 0 && d_end_min.y > 0) ||
			(d_start_max.y < 0 && d_end_max.y < 0)) {
			delete_at(edges, i);
			i--;
			continue;
		}

		Point direction = edges[i].end - edges[i].start;
		float grad = direction.y / direction.x;

		if (d_start_min.x > 0) {
			edges[i].start.x += d_start_min.x;
			edges[i].start.y += d_start_min.x * grad;
		} else if (d_start_max.x < 0) {
			edges[i].start.x = bounds_max.x;
			edges[i].start.y += d_start_max.x * grad;
		}

		if (d_end_min.x > 0) {
			edges[i].end.x += d_end_min.x;
			edges[i].end.y += d_end_min.x * grad;
		} else if (d_end_max.x < 0) {
			edges[i].end.x += d_end_max.x;
			edges[i].end.y += d_end_max.x * grad;
		}

		if (d_start_min.y > 0) {
			edges[i].start.y += d_start_min.y;
			edges[i].start.x += d_start_min.y / grad;
		} else if (d_start_max.y < 0) {
			edges[i].start.y += d_start_max.y;
			edges[i].start.x += d_start_max.y / grad;
		}

		if (d_end_min.y > 0) {
			edges[i].end.y += d_end_min.y;
			edges[i].end.x += d_end_min.y / grad;
		} else if (d_end_max.y < 0) {
			edges[i].end.y += d_end_max.y;
			edges[i].end.x += d_end_max.y / grad;
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
		events.push(new Event(EVENT_SITE, points[i], points[i].y));

		if (points[i].y > y_max) y_max = points[i].y;
		else if (points[i].y < y_min) y_min = points[i].y;

		if (points[i].x > x_max) x_max = points[i].x;
		else if (points[i].x < x_min) x_min = points[i].x;
	}

	float x_midpoint = (x_max + x_min) / 2;
	float y_above = y_max + (y_max - y_min) / 2; // a height significantly higher than the highest point

	current_sweep = y_above;

	// TODO: this is temporary
	root = new Coast(Point(x_midpoint, current_sweep), COAST_ARC); // this is to avoid the issues with the first generated sites
}
// ----- voronoi definitions end -----

}; // namespace voronoi
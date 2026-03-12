#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#define insert_at(vector, index, value) vector.insert(vector.begin() + (index), value);
#define delete_at(vector, index) vector.erase(vector.begin() + (index));

namespace voronoi {

enum EVENT_TYPES {
	EVENT_SITE,
	EVENT_INTERSECT
};

enum COAST_TYPES {
	COAST_ARC,
	COAST_EDGE
};

class Point {
  public:
	float x;
	float y;

	Point() {
		x = 0;
		y = 0;
	}

	Point(float px, float py) {
		x = px;
		y = py;
	}

	Point operator+(Point p) { return Point(x + p.x, y + p.y); }
	Point operator-(Point p) { return Point(x - p.x, y - p.y); }

	float norm2() { return x * x + y * y; }
	float norm() { return sqrt(norm2()); }
};

class Edge {
  public:
	Point start;
	Point end;

	Edge(Point start, Point end) {
		this->start = start;
		this->end = end;
	}
};

class Coast {
  public:
	int id;
	Point focus;

	// this tracks the feasible range of this arc
	float range_start;
	float range_end;

	int type;
	Point direction;
};

class Event {
  public:
	int type;
	Point point;
	float timestamp; // when this event will happen ( largest earliest )
	int event_target_id;

	Event(int event_type, Point event_point, float sweepline) {
		type = event_type;
		point = event_point;
		timestamp = sweepline;
	}
};

class Voronoi {
  private:
	std::vector<Event> events;
	std::vector<Coast> coastline;

	float current_sweep;
	int event_count = 0;
	int id = 0;

	Point bounds_min;
	Point bounds_max;

	Event pop_next_event() {
		size_t soonest = 0;
		for (size_t i = 1; i < events.size(); i++) {
			if (events[i].timestamp > current_sweep) continue; // TODO: this should not happen
			if (events[i].timestamp > events[soonest].timestamp) soonest = i;
		}

		Event soonest_event = events[soonest];
		delete_at(events, soonest);
		return soonest_event;
	}

	// evaluates the height at position x of a parabola with given focus and directrix
	float parabola_y(float x, Point focus, float directrix) {
		return pow(x - focus.x, 2) / (2 * (focus.y - directrix)) + (focus.y + directrix) / 2;
	}

	// get the coast arc above the given point
	int get_coast_above(Point point) {
		// TODO: deal with sites that have the same y component

		int min_index = -1;
		for (size_t i = 0; i < coastline.size(); i++) {
			if (coastline[i].range_start < point.x && coastline[i].range_end > point.x) {
				if (min_index == -1) min_index = i;
				else if (parabola_y(point.x, coastline[i].focus, point.y) < parabola_y(point.x, coastline[min_index].focus, point.y)) min_index = i;
			}
		}

		return min_index;
	}

	void get_intersection_event(int index) {
		if (index < 1 || index >= (int)coastline.size() - 1) return; // the left-most arc won't get clamped

		Coast arc = coastline[index];

		// the edges to the side of the arc
		Coast next = coastline[index + 1];
		Coast prev = coastline[index - 1];

		Point dist = next.focus - prev.focus;
		float val = next.direction.x * prev.direction.y - next.direction.y * prev.direction.x;

		if (val == 0) {
			// TODO: this should not happen
			printf("same\n");
		}
		float t = (dist.y * next.direction.x - dist.x * next.direction.y) / val;
		float c = (dist.y * prev.direction.x - dist.x * prev.direction.y) / val;

		// next_end = next.focus + t * next.direction
		Point intersection = Point(prev.focus.x + t * prev.direction.x, prev.focus.y + t * prev.direction.y);
		if (t < 0 || c < 0) {
			// if the rays move in reverse
			return;
		}

		float sweepline = intersection.y - sqrt(pow(intersection.x - arc.focus.x, 2) + pow(intersection.y - arc.focus.y, 2));

		if (sweepline > current_sweep) {
			return;
		}

		Event new_event = Event(EVENT_INTERSECT, intersection, sweepline);
		new_event.event_target_id = arc.id;

		events.push_back(new_event);
	}

	void do_site(Event event) {
		// create a new arc to add to the coastline
		Coast new_arc = Coast();
		new_arc.focus = event.point;
		new_arc.range_start = __FLT_MIN__;
		new_arc.range_end = __FLT_MAX__;
		new_arc.type = COAST_ARC;
		new_arc.id = id++;

		int coast_above = get_coast_above(event.point); // find the coast that is directly above this point
		if (coast_above == -1) {
			// this should only happen when the coast is empty (when this is the first event)
			coastline.push_back(new_arc);
		} else {
			// the arc above will be split in two, where the feasible ranges will extend from the newly inserted point
			Coast left_split = coastline[coast_above];
			Coast right_split = coastline[coast_above];
			left_split.range_end = event.point.x;
			right_split.range_start = event.point.x;
			left_split.id = id++;
			right_split.id = id++;

			// where the new arc intersects with the arc above
			Point intersection_point = Point(event.point.x, parabola_y(event.point.x, coastline[coast_above].focus, event.timestamp));
			Coast left;
			Coast right;
			left.focus = intersection_point;
			right.focus = intersection_point;
			left.type = COAST_EDGE;
			right.type = COAST_EDGE;
			left.id = id++;
			right.id = id++;

			Point dr = new_arc.focus - coastline[coast_above].focus;
			left.direction = Point(dr.y, -dr.x);  // rotate by -pi/2
			right.direction = Point(-dr.y, dr.x); // rotate by pi/2

			delete_at(coastline, coast_above);

			insert_at(coastline, coast_above, left_split);
			insert_at(coastline, coast_above + 1, left);
			insert_at(coastline, coast_above + 2, new_arc);
			insert_at(coastline, coast_above + 3, right);
			insert_at(coastline, coast_above + 4, right_split);

			get_intersection_event(coast_above);	 // the left side
			get_intersection_event(coast_above + 4); // the right side
		}
	}

	void do_intersect(Event event) {
		int index = -1;
		for (size_t i = 0; i < coastline.size(); i++) {
			if (coastline[i].id == event.event_target_id) {
				index = i;
				break;
			}
		}
		if (index == -1) {
			printf("\n\ncould not find target\n\n\n");
			return;
		}

		Coast collapsed_arc = coastline[index];
		Coast edge_left = coastline[index - 1];
		Coast edge_right = coastline[index + 1];

		delete_at(coastline, index + 1); // delete left edge
		delete_at(coastline, index);	 // delete collapsed arc
		delete_at(coastline, index - 1); // delete right edge

		Coast new_edge;
		new_edge.type = COAST_EDGE;
		new_edge.id = id++;

		// these arcs now need an edge between them
		Coast arc_left = coastline[index - 2];
		Coast arc_right = coastline[index - 1];

		Point dr = arc_right.focus - arc_left.focus;
		new_edge.direction = Point(dr.y, -dr.x);
		new_edge.focus = event.point;

		insert_at(coastline, index - 1, new_edge);

		// these edges are apart of the voronoi diagram
		edges.push_back(Edge(event.point, edge_left.focus));
		edges.push_back(Edge(event.point, edge_right.focus));

		get_intersection_event(index);	   // the next arc
		get_intersection_event(index - 2); // the previous arc
	}

  public:
	std::vector<Point> points;
	std::vector<Edge> edges;

	Voronoi(float **pts, int point_count, float x_min, float x_max, float y_min, float y_max) {
		for (int i = 0; i < point_count; i++) {
			points.push_back(Point(pts[i][0], pts[i][1]));
		}

		bounds_min = Point(x_min, y_min);
		bounds_max = Point(x_max, y_max);
	}

	Voronoi() {
	}

	void next_step() {
		if (events.size() > 0) {
			Event next_event = pop_next_event();

			current_sweep = next_event.timestamp;

			if (next_event.type == EVENT_SITE) do_site(next_event);
			else do_intersect(next_event);
		}
	}

	void solve_full() {
		solve();

		while (events.size() > 0) {
			next_step();
		}

		for (size_t i = 0; i < coastline.size(); i++) {
			if (coastline[i].type == COAST_EDGE) {
				float start_x = coastline[i].focus.x;
				float start_y = coastline[i].focus.y;
				float direction_x = coastline[i].direction.x;
				float direction_y = coastline[i].direction.y;

				float x1 = (bounds_min.x - start_x) / direction_x;
				float x2 = (bounds_max.x - start_x) / direction_x;

				float y1 = (bounds_min.y - start_y) / direction_y;
				float y2 = (bounds_max.y - start_y) / direction_y;

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
						edges.push_back(Edge(coastline[i].focus, Point(bounds_min.x, start_y + direction_y * x_used)));
					} else {
						edges.push_back(Edge(coastline[i].focus, Point(bounds_max.x, start_y + direction_y * x_used)));
					}
				} else {
					if (use_min_y) {
						edges.push_back(Edge(coastline[i].focus, Point(start_x + direction_x * y_used, bounds_min.y)));
					} else {
						edges.push_back(Edge(coastline[i].focus, Point(start_x + direction_x * y_used, bounds_max.y)));
					}
				}
			}
		}

		for (size_t i = 0; i < edges.size(); i++) {
			Point direction = edges[i].end - edges[i].start;
			if (edges[i].start.x < bounds_min.x) {
				float x1 = (bounds_min.x - edges[i].start.x) / direction.x;
				edges[i].start.x = bounds_min.x;
				edges[i].start.y += x1 * direction.y;
			} else if (edges[i].start.x > bounds_max.x) {
				float x1 = (bounds_max.x - edges[i].start.x) / direction.x;
				edges[i].start.x = bounds_max.x;
				edges[i].start.y += x1 * direction.y;
			}

			if (edges[i].end.x < bounds_min.x) {
				float x1 = (bounds_min.x - edges[i].end.x) / direction.x;
				edges[i].end.x = bounds_min.x;
				edges[i].end.y += x1 * direction.y;
			} else if (edges[i].end.x > bounds_max.x) {
				float x1 = (bounds_max.x - edges[i].end.x) / direction.x;
				edges[i].end.x = bounds_max.x;
				edges[i].end.y += x1 * direction.y;
			}

			// if (edges[i].start.y < bounds_min.y) {
			// 	float y1 = (bounds_min.y - start.y) / direction.y;
			// 	edges[i].start.y = bounds_min.x;
			// 	edges[i].start.x += y1 * direction.x;
			// } else if (edges[i].start.y > bounds_max.y) {
			// 	float y1 = (bounds_max.y - start.y) / direction.y;
			// 	edges[i].start.y = bounds_max.y;
			// 	edges[i].start.x += y1 * direction.x;
			// }
		}
	}

	void solve() {
		if (points.size() < 1) return;

		current_sweep = points[0].y;

		float x_min = points[0].x;
		float x_max = points[0].x;
		float y_min = points[0].y;
		float y_max = points[0].y;
		// add all the sites as events that will take place
		for (size_t i = 0; i < points.size(); i++) {
			events.push_back(Event(EVENT_SITE, points[i], points[i].y));

			if (points[i].y > y_max) y_max = points[i].y;
			else if (points[i].y < y_min) y_min = points[i].y;

			if (points[i].x > x_max) x_max = points[i].x;
			else if (points[i].x < x_min) x_min = points[i].x;
		}

		float x_midpoint = (x_max + x_min) / 2;
		float y_above = y_max + (y_max - y_min) / 2; // a height significantly higher than the highest point

		current_sweep = y_above;

		// TODO: this is temporary
		events.push_back(Event(EVENT_SITE, Point(x_midpoint, current_sweep), current_sweep)); // this is to avoid the issues with the first generated sites
	}
};

}; // namespace voronoi
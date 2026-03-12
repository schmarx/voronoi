#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#define rng(max) rand() % ((max) + 1)
#define rngr(min, max) (rng(((max) - (min))) + (min))

#define insert_at(vector, index, value) vector.insert(vector.begin() + (index), value);
#define delete_at(vector, index) vector.erase(vector.begin() + (index));

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

	// Edge *left;
	// Edge *right;
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
  public:
	std::vector<Event> events;
	std::vector<Point> points;
	std::vector<Coast> coastline;

	std::vector<Edge> edges;

	float current_sweep; // TODO: this causes issues with the condition in pop_next_event
	int event_count = 0;
	int id = 0;

	Voronoi(float **pts, int point_count) {
		for (int i = 0; i < point_count; i++) {
			points.push_back(Point(pts[i][0], pts[i][1]));
		}
	}

	Voronoi() {
	}

	Event pop_next_event() {
		size_t soonest = 0;
		for (size_t i = 1; i < events.size(); i++) {
			if (events[i].timestamp > current_sweep) continue; // TODO: this probably will not happen
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

		printf("intersection %.1f, %.1f [%.1f, %.1f] -> %.1f, %.1f [%.1f, %.1f]\n", prev.focus.x, prev.focus.y, prev.direction.x, prev.direction.y, next.focus.x, next.focus.y, next.direction.x, next.direction.y);
		if (val == 0) {
			printf("same\n");
		}
		float t = (dist.y * next.direction.x - dist.x * next.direction.y) / val;
		float c = (dist.y * prev.direction.x - dist.x * prev.direction.y) / val;

		// next_end = next.focus + t * next.direction
		Point intersection = Point(prev.focus.x + t * prev.direction.x, prev.focus.y + t * prev.direction.y);
		printf("> %.1f, %.1f\n", intersection.x, intersection.y);
		if (t < 0 || c < 0) {
			// if the rays move in reverse
			printf("exit\n");
			return;
		}

		float sweepline = intersection.y - sqrt(pow(intersection.x - arc.focus.x, 2) + pow(intersection.y - arc.focus.y, 2));

		printf("incl (%f, %f) ev at %.1f (%.1f)\n", arc.focus.x, arc.focus.y, sweepline, current_sweep);
		// if (sweepline > current_sweep) {
		// 	printf("excl (%.1f > %.1f)\n", sweepline, current_sweep);
		// 	return;
		// }

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

		printf("%.1f, %.1f removed\n", collapsed_arc.focus.x, collapsed_arc.focus.y);
		printf("%.1f, %.1f -> %.1f, %.1f added\n", new_edge.focus.x, new_edge.focus.y, new_edge.direction.x, new_edge.direction.y);
		insert_at(coastline, index - 1, new_edge);

		// these edges are apart of the voronoi diagram
		edges.push_back(Edge(event.point, edge_left.focus));
		edges.push_back(Edge(event.point, edge_right.focus));

		get_intersection_event(index);	   // the next arc
		get_intersection_event(index - 2); // the previous arc
	}

	void output_edges(const char *filename) {
		FILE *file = fopen(filename, "w");

		for (size_t i = 0; i < edges.size(); i++) {
			fprintf(file, "(%.1f, %.1f), (%.1f, %.1f)\n", edges[i].start.x, edges[i].start.y, edges[i].end.x, edges[i].end.y);
		}

		fclose(file);
	}

	void next_step() {
		if (events.size() > 0) {
			Event next_event = pop_next_event();

			current_sweep = next_event.timestamp;
			printf("[%i] @ %f %s event at %.1f, %.1f\n", event_count++, current_sweep, next_event.type == EVENT_SITE ? "site" : "intersection", next_event.point.x, next_event.point.y);

			if (next_event.type == EVENT_SITE) do_site(next_event);
			else do_intersect(next_event);

			for (size_t i = 0; i < coastline.size(); i++) {
				if (i != 0) printf(" -> ");
				printf("(%.1f, %.1f) [%s]", coastline[i].focus.x, coastline[i].focus.y, coastline[i].type == COAST_ARC ? "arc" : "edge");
			}
			printf("\n\n");
		}
	}

	void solve_full() {
		solve();

		while (events.size() > 0) {
			next_step();
		}

		output_edges("./output/edges.txt");
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

int main(int argc, char *argv[]) {
	int point_count = 10;

	float **points = (float **)calloc(point_count, sizeof(float *));
	for (int i = 0; i < point_count; i++) {
		points[i] = (float *)calloc(2, sizeof(float));

		points[i][0] = rngr(0, 100);
		points[i][1] = rngr(0, 100);
	}

	Voronoi voronoi = Voronoi(points, point_count);
	voronoi.solve_full();

	for (int i = 0; i < point_count; i++) {
		free(points[i]);
	}
	free(points);

	return EXIT_SUCCESS;
}
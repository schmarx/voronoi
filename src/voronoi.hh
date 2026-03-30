#include <queue>
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

	Point();
	Point(float px, float py);

	Point operator+(Point p) const;
	Point operator-(Point p) const;

	float norm2();
	float norm();
};

// evaluates the height at position x of a parabola with given focus and directrix
float parabola_y(float x, Point focus, float directrix);

class Edge {
  public:
	Point start;
	Point end;

	Edge(Point start, Point end);
};

class Coast;

class Event {
  public:
	int id;
	int type;
	Point point;
	float timestamp; // the sweepline point at which this event will happen ( largest happens earliest )
	Coast *event_target;

	int invalid = 0;

	Event(int event_type, Point event_point, float sweepline);

	// the event with largest timestamp occurs first
	bool operator<(const Event &event) const;
};

class Voronoi;

class Coast {
  public:
	int id;

	Point focus;
	int type;

	// this tracks the feasible range of an arc
	float range_start;
	float range_end;

	//  this tracks the direction of an edge
	Point direction;

	Coast *left = nullptr;
	Coast *right = nullptr;

	// references to related coast items
	Coast *parent = nullptr;
	Coast *next = nullptr;
	Coast *prev = nullptr;

	Event *associated_event = nullptr;

	Coast(Point start, int coast_type);
	Coast *split_left(Event event);
	Coast *split_right(Event event);
	void get_intersection_event(Voronoi *solver);

	// the arc above will be split in two, where the feasible ranges will extend from the newly inserted point
	void split(const Event &event, Voronoi *solver);
	void intersection(const Event &event, Voronoi *solver);
};

class Comparator {
  public:
	bool operator()(const Event *ev1, const Event *ev2) {
		return *ev1 < *ev2;
	}
};

class Voronoi {
  private:
	int event_count = 0;

	Point bounds_min;
	Point bounds_max;

	// get the coast arc above the given point
	Coast *get_coast_above(Point point);
	void do_site(Event &event);
	void do_intersect(Event &event);

  public:
	std::vector<Point> points;
	std::vector<Edge> edges;

	std::priority_queue<Event *, std::vector<Event *>, Comparator> events;

	Coast *root;
	float current_sweep;
	int proceessed_events = 0;

	Voronoi(float **pts, int point_count, float x_min, float x_max, float y_min, float y_max);
	Voronoi();

	void next_step();
	void add_edge(Coast &edge);
	void get_edge(Coast *coast);
	void add_remaining_edges();
	void clip_edges();
	void solve_full();
	void solve();
};
}; // namespace voronoi
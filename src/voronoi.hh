#pragma once

#include <queue>
#include <vector>

#define insert_at(vector, index, value) vector.insert(vector.begin() + (index), value);
#define delete_at(vector, index) vector.erase(vector.begin() + (index));

namespace voronoi {

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

class Line {
  public:
	Point start;
	Point end;

	Line(Point start, Point end) {
		this->start = start;
		this->end = end;
	}
};

class Range {
  public:
	float start;
	float end;

	Range(float start, float end) {
		this->start = start;
		this->end = end;
	}

	bool contains(float value) {
		return start < value && end > value;
	}
};

// evaluates the height at position x of a parabola with given focus and directrix
float parabola_y(float x, Point focus, float directrix);

class Edge {
  public:
	Line line;

	Edge(Point start, Point end);
};

class Coast;
class Voronoi;

class Event {
  public:
	int id;
	Point point;
	float timestamp; // the sweepline point at which this event will happen ( largest happens earliest )

	Event(Point event_point, float sweepline);
	virtual ~Event() = 0;

	bool operator<(const Event &event) const;								 // the event with largest timestamp occurs first
	virtual void handle(Voronoi &solver, std::vector<Coast> &coastline) = 0; // handles the event
};

class SiteEvent : public Event {
  public:
	SiteEvent(Point event_point, float sweepline);
	void handle(Voronoi &solver, std::vector<Coast> &coastline);
};

class IntersectEvent : public Event {
  public:
	int event_target_id;

	IntersectEvent(Point event_point, float sweepline, int target_id);
	void handle(Voronoi &solver, std::vector<Coast> &coastline);
};

class Coast {
  public:
	int id;

	Point focus;
	int type;

	Range range_x;	 // this tracks the feasible range of an arc
	Point direction; //  this tracks the direction of an edge

	Coast(Point start, int coast_type);
	Coast copy();
};

class Comparator {
  public:
	bool operator()(const Event *ev1, const Event *ev2) {
		return *ev1 < *ev2;
	}
};

class Voronoi {
  private:
	std::priority_queue<Event *, std::vector<Event *>, Comparator> events;
	std::vector<Coast> coastline;

	float current_sweep;
	int event_count = 0;

	Line bounds;

  public:
	std::vector<Point> points;
	std::vector<Edge> edges;

	int proceessed_events = 0;

	Voronoi(float **pts, int point_count, Point min, Point max);
	Voronoi();

	int get_coast_above(Point point); // get the coast arc above the given point
	void get_intersection_event(int index);

	void init_events(); // initialise for handling events step-by-step
	void next_step();	// handle next event
	void solve();		// initialises and steps through all events

	void add_remaining_edges();
	void clip_edges();
};

}; // namespace voronoi
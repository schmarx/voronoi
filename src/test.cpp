#include "voronoi.cpp"

#define rng(max) rand() % ((max) + 1)
#define rngr(min, max) (rng(((max) - (min))) + (min))
#define rngf() ((float)rand() / (float)RAND_MAX)
#define rngfr(min, max) ((min) + ((max) - (min)) * rngf())

void output_edges(const std::vector<voronoi::Point> &points, const std::vector<voronoi::Edge> &edges, const char *filename) {
	FILE *file = fopen(filename, "w");

	for (size_t i = 0; i < points.size(); i++) {
		if (i != 0) fprintf(file, "\t");
		fprintf(file, "(%.1f, %.1f)", points[i].x, points[i].y);
	}

	for (size_t i = 0; i < edges.size(); i++) {
		fprintf(file, "\n");
		fprintf(file, "(%.1f, %.1f)\t(%.1f, %.1f)", edges[i].start.x, edges[i].start.y, edges[i].end.x, edges[i].end.y);
	}

	fclose(file);
}

int main(int argc, char *argv[]) {
	int point_count = 20;

	float x_min = 0;
	float x_max = 100;
	float y_min = 0;
	float y_max = 100;

	float **points = (float **)calloc(point_count, sizeof(float *));
	for (int i = 0; i < point_count; i++) {
		points[i] = (float *)calloc(2, sizeof(float));

		points[i][0] = rngfr(y_min, x_max);
		points[i][1] = rngfr(y_min, y_max);
	}

	voronoi::Voronoi voronoi = voronoi::Voronoi(points, point_count, x_min, x_max, y_min, y_max);
	voronoi.solve_full();

	output_edges(voronoi.points, voronoi.edges, "./output/edges.txt");

	for (int i = 0; i < point_count; i++) {
		free(points[i]);
	}
	free(points);

	return EXIT_SUCCESS;
}
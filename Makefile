
run:
	mkdir bin -p
	mkdir output -p
	g++ -O3 -Wall -pedantic voronoi.cpp -o ./bin/voronoi
	./bin/voronoi
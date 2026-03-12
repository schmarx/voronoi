
run:
	mkdir bin -p
	mkdir output -p
	g++ -O3 -Wall -pedantic ./src/voronoi.cpp -o ./bin/voronoi
	./bin/voronoi
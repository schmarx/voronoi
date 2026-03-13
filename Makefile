
run:
	mkdir bin -p
	mkdir output -p
	g++ -O3 -Wall -pedantic ./src/test.cpp -o ./bin/voronoi
	./bin/voronoi
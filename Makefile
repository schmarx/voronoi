
run:
	mkdir ./demo/bin -p
	mkdir ./demo/output -p
	g++ -O3 -Wall -pedantic ./demo/demo.cpp ./src/voronoi.cpp -o ./demo/bin/voronoi
	./demo/bin/voronoi

clean:
	rm -r ./demo/bin
	rm -r ./demo/output
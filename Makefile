all:
	(cd lib;   make)
	(cd src;   make)

clean:
	(cd lib;   make clean)
	(cd src;   make clean)


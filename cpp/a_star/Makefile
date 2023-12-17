src = $(wildcard *.cpp)
obj = $(src:.c=.o)

CFLAGS = -O3 -shared -std=c++11 -fPIC -Wall $(PYBIND_INCLUDES)

all: $(obj)
	g++ $(CFLAGS) -o a_star.so $^

debug: $(filter-out lib_main.cpp,$(obj))
	g++ -O0 -std=c++14 -Wall -g -o debug.bin debug_main.cc $^

.PHONY: clean
clean:
	rm -f $(obj) all

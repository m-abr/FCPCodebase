src = $(wildcard *.cpp)
obj = $(src:.c=.o)

LDFLAGS = -lgsl -lgslcblas
CFLAGS = -O3 -shared -std=c++11 -fPIC -Wall $(PYBIND_INCLUDES)

all: $(obj)
	g++ $(CFLAGS) -o localization.so $^ $(LDFLAGS) 

debug: $(filter-out lib_main.cpp,$(obj))
	g++ -O0 -std=c++14 -Wall -g -o debug.bin debug_main.cc $^ $(LDFLAGS)

.PHONY: clean
clean:
	rm -f $(obj) all

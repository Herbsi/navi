CXX=g++
CPPFLAGS= -Wall -Wextra -std=c++17

SRCS=main.cpp graph.hpp graph.cpp node.hpp node.cpp

bsp_30: $(SRCS)
	$(CXX) -c $(SRCS) $(CPPFLAGS)

debug: $(SRCS)
	$(CXX) -c $(SRCS) $(CPPFLAGS) -g

clean: $(OBJECTS)
	rm $(OBJECTS)

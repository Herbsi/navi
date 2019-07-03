CXX=g++-9
CPPFLAGS= -Wall -Wextra -pedantic -Wswitch-default -Wmissing-declarations -Wfloat-equal -Wundef -Wredundant-decls
-Wuninitialized -Winit-self -Wnon-virtual-dtor -Weffc++ -Wshadow -Wparentheses -Wunreachable-code -Wfatal-errors -std=c++14

SRCS=main.cpp graph.hpp graph.cpp node.hpp node.cpp
OBJECTS=bsp_30

bsp_30: $(SRCS)
	$(CXX) $(CPPFLAGS) -o bsp_30 $(SRCS)

debug: $(SRCS)
	$(CXX) $(CPPFLAGS) -g -o bsp_30 $(SRCS)

clean: $(OBJECTS)
	rm $(OBJECTS)

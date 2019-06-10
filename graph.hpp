#ifndef GRAPH_H
#define GRAPH_H

#define DEBUG 0

#if DEBUG == 1
#define LOG(x) std::cerr << x << std::endl;
#else
#define LOG(x)
#endif

#include "node.hpp"
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using NodeIndex = unsigned int;
using IndexedNode = std::pair<NodeIndex, NodePtr>;
class Distance;
struct DNodeContainer;

class Graph {
public:
  //! Default constructor
  Graph();

  Graph(const std::string &filename);

  //! Copy constructor
  Graph(const Graph &other) = delete;

  //! Move constructor
  Graph(Graph &&other) noexcept;

  //! Destructor
  virtual ~Graph() noexcept;

  //! Copy assignment operator
  Graph &operator=(const Graph &other);

  //! Move assignment operator
  Graph &operator=(Graph &&other) noexcept;

  std::map<NodeIndex, NodePtr> nodes() const { return _nodes; }

  double edgeLength(const NodeIndex i, const NodeIndex j);

  double pathLength(const std::vector<NodeIndex> &path);

  std::vector<NodeIndex> navi(const NodeIndex i, const NodeIndex j);

protected:
private:
  std::map<NodeIndex, NodePtr> _nodes;
  std::map<NodeIndex, std::vector<NodeIndex>> _neighbours;
  DistanceResult _edgeLength(const NodeIndex i, const NodeIndex j);
  DNodeContainer _initialise(const NodeIndex i);
  Graph &_dijkstra(DNodeContainer &dnc);
  std::vector<NodeIndex>
  _builtPath(const NodeIndex endpoint,
             const std::map<NodeIndex, IndexedNode> &predecessor);
};

///////////////////////////////////////////////////////////////////////////////
//                                  DISTANCE                                 //
///////////////////////////////////////////////////////////////////////////////
class Distance {
public:
  //! Default Constructor
  Distance() : Distance(0, false) {}

  Distance(const double d) : Distance(d, false) {}

  Distance(const double d, const bool b) : distance(d), isInf(b) {}

  //! Destructor
  virtual ~Distance() noexcept {}

  double distance;
  bool isInf;
};

bool operator<(const Distance &a, const Distance &b);
Distance operator+(const Distance &lhs, const Distance &rhs);

///////////////////////////////////////////////////////////////////////////////
//                                   DNodeContainer                          //
///////////////////////////////////////////////////////////////////////////////

struct DNodeContainer {

  std::map<NodeIndex, Distance> distances;
  std::map<NodeIndex, IndexedNode> predecessors;
  std::map<NodeIndex, bool> visited;
};

#endif // GRAPH_H

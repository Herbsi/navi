#ifndef GRAPH_H
#define GRAPH_H

#include "node.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using NodeIndex = unsigned int;

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

  double edgeLength(const NodeIndex i, const NodeIndex j);

  double pathLength(const std::vector<NodeIndex> &path);

  std::vector<NodeIndex> navi(const NodeIndex i, const NodeIndex j);

protected:
private:
  std::map<NodeIndex, NodePtr> _nodes;
  DistanceResult _edgeLength(const NodeIndex i, const NodeIndex j);
};

class DijkstraNode {
public:
  double distance;
  NodePtr predecessor;
}

#endif // GRAPH_H

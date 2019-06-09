#ifndef GRAPH_H
#define GRAPH_H

#include "node.hpp"
#include <algorithm>
#include <cassert>
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

  std::map<NodeIndex, NodePtr> nodes() const { return _nodes; }

  double edgeLength(const NodeIndex i, const NodeIndex j);

  double pathLength(const std::vector<NodeIndex> &path);

  std::vector<NodeIndex> navi(const NodeIndex i, const NodeIndex j);

protected:
private:
  std::map<NodeIndex, NodePtr> _nodes;
  std::map<NodeIndex, std::vector<NodeIndex>> _neighbours;
  DistanceResult _edgeLength(const NodeIndex i, const NodeIndex j);
};

#endif // GRAPH_H

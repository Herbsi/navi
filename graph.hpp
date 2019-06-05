#ifndef GALEYSHAPLEY_H
#define GALEYSHAPLEY_H

#include "node.hpp"
#include <string>
#include <vector>

class Graph {
public:
  //! Default constructor
  Graph();

  Graph(const std::string &filename);

  //! Copy constructor
  Graph(const Graph &other);

  //! Move constructor
  Graph(Graph &&other) noexcept;

  //! Destructor
  virtual ~Graph() noexcept;

  //! Copy assignment operator
  Graph &operator=(const Graph &other);

  //! Move assignment operator
  Graph &operator=(Graph &&other) noexcept;

protected:
private:
  std::vector<Node *> _nodes;
};

#endif // GALEYSHAPLEY_H

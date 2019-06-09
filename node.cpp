#include "node.hpp"

double euclid(const double x1, const double y1, const double x2,
              const double y2) {
  double x = x1 - x2;
  double y = y1 - y2;
  return sqrt(x * x + y * y);
}

///////////////////////////////////////////////////////////////////////////////
//                                    CONSTRUCTORS //
///////////////////////////////////////////////////////////////////////////////

Node::Node() : Node(0, 0, {}) {}

Node::Node(const double x, const double y,
           const std::vector<NodePtr> neighbours)
    : _x(x), _y(y), _neighbours(neighbours) {

  // cache distances to neighbours
  for (auto &n : _neighbours) {
    double dist = euclid(_x, n->getX(), _y, n->getY());
    _cachedDistancesToNeighbours.insert(std::make_pair(n, dist));
  }
}

Node::~Node() noexcept {}

///////////////////////////////////////////////////////////////////////////////
//                                 ASSIGNMENT                                //
///////////////////////////////////////////////////////////////////////////////

Node &Node::operator=(const Node &other) {
  // Check for self-assignment!
  if (this == &other)
    return *this;
  _neighbours = other._neighbours;

  return *this;
}

double calculateDistanceBetweenNodes(const NodePtr a, const NodePtr b) {
  return euclid(a->getX(), a->getY(), b->getX(), b->getY());
}

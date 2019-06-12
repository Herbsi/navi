#include "node.hpp"

double euclid(const double x1, const double y1, const double x2,
              const double y2);
double euclid(const double x1, const double y1, const double x2,
              const double y2) {
  double x = x1 - x2;
  double y = y1 - y2;
  return sqrt(x * x + y * y);
}

double norm(const double x1, const double y1);
double norm(const double x1, const double y1) { return euclid(x1, y1, 0, 0); }

Angle angle(const NodePtr a, const NodePtr b, const NodePtr c) {
  double x1 = c->getX() - b->getX();
  double y1 = c->getY() - b->getY();
  double x2 = a->getX() - b->getX();
  double y2 = a->getY() - b->getY();
  // TODO fix, this sometimes returns a value larger than 1
  double arg = (x1 * x2 + y1 * y1) / (norm(x1, y1) * norm(x2, y2));
  LOGNode(arg);
  double angle = acos(arg);
  bool less_than_ten_degrees = angle * 180.0 / M_PI < 10;
  LOGNode(angle);

  return {angle, less_than_ten_degrees};
}

///////////////////////////////////////////////////////////////////////////////
//                                    CONSTRUCTORS //
///////////////////////////////////////////////////////////////////////////////

Node::Node() : Node(0, 0, {}) {}

Node::Node(const double x, const double y) : Node(x, y, {}) {}

Node::Node(const double x, const double y,
           const std::vector<NodePtr> neighbours)
    : _x(x), _y(y), _neighbours(neighbours) {

  // cache distances to neighbours
  for (auto &n : _neighbours) {
    double dist =
        calculateDistanceBetweenNodes(std::make_shared<Node>(*this), n);
    _cachedDistancesToNeighbours.insert(std::make_pair(n, dist));
  }
}

Node::Node(const Node &other)
    : _x(other._x), _y(other._y), _neighbours(other._neighbours),
      _cachedDistancesToNeighbours(other._cachedDistancesToNeighbours) {}

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

Node &Node::addNeighbour(const NodePtr other) {
  _neighbours.push_back(other);
  double dist =
      calculateDistanceBetweenNodes(std::make_shared<Node>(*this), other);
  _cachedDistancesToNeighbours.insert(std::make_pair(other, dist));
  return *this;
}

DistanceResult Node::getDistancetoNeighbour(const NodePtr other) {
  auto search = _cachedDistancesToNeighbours.find(other);
  if (search != _cachedDistancesToNeighbours.end()) {
    return DistanceResult(true, search->second);
  } else {
    return DistanceResult(false, -1);
  }
}

double calculateDistanceBetweenNodes(const NodePtr a, const NodePtr b) {
  return euclid(a->getX(), a->getY(), b->getX(), b->getY());
}

#include "node.hpp"

///////////////////////////////////////////////////////////////////////////////
//                                    NODE                                   //
///////////////////////////////////////////////////////////////////////////////

Node::Node() : Node(0, 0, {}, {{}}) {}

Node::Node(const double x, const double y, const std::vector<Node *> neighbours,
           const std::map<Node *, Edge *> neighbour_edge_dict)
    : _x(x), _y(y), _neighbours(neighbours),
      _neighbour_edge_dict(neighbour_edge_dict) {}

Node::Node(const Node &other) : _x(other._x), _y(other._y) {

  _neighbours.resize(other._neighbours.size());
  _neighbours = other._neighbours;
  // TODO resize dict to correct size
  _neighbour_edge_dict = other._neighbour_edge_dict;
}

Node::~Node() noexcept {
  for (auto &node : _neighbours) {
    delete node;
  }
  for (auto &t : _neighbour_edge_dict) {
    delete t.first;
    delete t.second;
  }
}

Node &Node::operator=(const Node &other) {
  // Check for self-assignment!
  if (this == &other)
    return *this;
  _neighbours = other._neighbours;
  // TODO resize dict to correct size
  _neighbour_edge_dict = other._neighbour_edge_dict;

  return *this;
}

///////////////////////////////////////////////////////////////////////////////
//                                    EDGE                                   //
///////////////////////////////////////////////////////////////////////////////

Edge::Edge() : Edge(nullptr, nullptr) {}

Edge::Edge(const Node *a, const Node *b) {
  _a = a;
  _b = b;
}

Edge::Edge(const Edge &other) : _length(other._length) {
  // using copy constructor of Node
  _a = new Node(*other._a);
  _b = new Node(*other._b);
}

Edge::~Edge() noexcept {
  delete _a;
  delete _b;
}

Edge Edge::_calculateLength() {
  double x1 = _a->_x;
  double x2 = _a->_y;
  double y1 = _b->_x;
  double y2 = _b->_y;
  _length = sqrt((x1 - y1) * (x1 - y1) + (x2 - y2) * (x2 - y2));

  return *this;
}

#include "node.hpp"

///////////////////////////////////////////////////////////////////////////////
//                              HELPER FUNCTIONS                             //
///////////////////////////////////////////////////////////////////////////////

//! Calculate euclidian distance between (x1, y1) & (x2, y2)
//
//! \param[in] x1 x Coordinate of v1
//! \param[in] y1 y Coordinate of v1
//! \param[in] x2 x Coordinate of v2
//! \param[in] y2 y Coordinate of v2
//
//! \return euclidian distance
double euclid(const double x1, const double y1, const double x2,
              const double y2);
double euclid(const double x1, const double y1, const double x2,
              const double y2) {
  const double x = x1 - x2;
  const double y = y1 - y2;
  return sqrt(x * x + y * y);
}

//! Calculate norm of (x1, y1)
//
//! \param[in] x1 x Coordinate
//! \param[in] y1 y Coordinate
double norm(const double x1, const double y1);
double norm(const double x1, const double y1) { return euclid(x1, y1, 0, 0); }

//! Calculate dot product between (x1, y1) & (x2, y2)
//
//! \param[in] x1 x Coordinate of v1
//! \param[in] y1 y Coordinate of v1
//! \param[in] x2 x Coordinate of v2
//! \param[in] y2 y Coordinate of v2
//
//! \return dot product
constexpr double dot(const double x1, const double x2, const double y1,
                     const double y2);
constexpr double dot(const double x1, const double x2, const double y1,
                     const double y2) {
  return (x1 * x2 + y1 * y2);
}

/**
 * \brief           Calculate euclidian distance between nodes
 *
 * \param[in]       a First node
 * \param[in]       b Second node
 *
 * \return          return type
 */
double calculateDistanceBetweenNodes(const NodePtr &a, const NodePtr &b);
double calculateDistanceBetweenNodes(const NodePtr &a, const NodePtr &b) {
  return euclid(a->getX(), a->getY(), b->getX(), b->getY());
}

///////////////////////////////////////////////////////////////////////////////

Angle angle(const NodePtr &a, const NodePtr &b, const NodePtr &c) {
  const double x1 = c->getX() - b->getX();
  const double y1 = c->getY() - b->getY();
  const double x2 = b->getX() - a->getX();
  const double y2 = b->getY() - a->getY();
  const double arg = dot(x1, x2, y1, y2) / (norm(x1, y1) * norm(x2, y2));
  const double angle = acos(arg);
  const bool less_than_ten_degrees = angle * 180.0 / M_PI < 10;

  // determine direction of turn
  // https://math.stackexchange.com/questions/555198/find-direction-of-angle-between-2-vectors
  // using notation from stack exchange answer
  // coming from u, going to v => v is left of u <=> turning in positive
  // direction

  // u gets assigned - the vector the "car" is coming from because delta
  // contains the direction of the turn when standing on the current node, but
  // facing backwards
  // I am looking to where I came from, then turning say 80 degrees in the
  // positive direction <=> turing right by 80 degrees
  const double &u1 = -x2;
  const double &u2 = -y2;
  const double &v1 = x1;
  const double &v2 = y1;
  const double delta = u1 * v2 - u2 * v1;
  bool turnsPositive;
  if (delta > 0) {
    turnsPositive = true;
  } else {
    turnsPositive = false;
  }
  return {angle, less_than_ten_degrees, turnsPositive};
}

///////////////////////////////////////////////////////////////////////////////
//                                    CONSTRUCTORS //
///////////////////////////////////////////////////////////////////////////////

Node::Node() : Node(0, 0, {}, {{}}) {}

Node::Node(const double x, const double y) : Node(x, y, {}, {{}}) {}

Node::Node(const double x, const double y,
           const std::vector<NodePtr> &neighbours,
           const std::map<NodePtr, double> &cachedDistancesToNeighbours)
    : _x(x), _y(y), _neighbours(neighbours),
      _cachedDistancesToNeighbours(cachedDistancesToNeighbours) {

  // cache distances to neighbours
  for (auto &n : _neighbours) {
    const double dist =
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
  if (this != &other) {
    _neighbours = other._neighbours;
    _cachedDistancesToNeighbours = other._cachedDistancesToNeighbours;
  }

  return *this;
}

///////////////////////////////////////////////////////////////////////////////
//                              MEMBER FUNCTIONS                             //
///////////////////////////////////////////////////////////////////////////////

Node &Node::addNeighbour(const NodePtr &other) {
  _neighbours.push_back(other);
  double dist =
      calculateDistanceBetweenNodes(std::make_shared<Node>(*this), other);
  _cachedDistancesToNeighbours.insert(std::make_pair(other, dist));
  return *this;
}

DistanceResult Node::getDistancetoNeighbour(const NodePtr &other) {
  auto search = _cachedDistancesToNeighbours.find(other);
  DistanceResult dr;
  if (search != _cachedDistancesToNeighbours.end()) {
    dr = DistanceResult(true, search->second);
  } else {
    dr = DistanceResult(false, -1);
  }
  return dr;
}

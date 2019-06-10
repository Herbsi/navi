#ifndef NODE_HPP
#define NODE_HPP

#include <cmath>
#include <map>
#include <memory>
#include <vector>

class Node;
class DistanceResult;

using NodePtr = std::shared_ptr<Node>;
using Angle = double;

class Node {

public:
  //! Default constructor
  Node();

  Node(const double x, const double y);
  Node(const double x, const double y, const std::vector<NodePtr> neighbours);

  //! Copy constructor
  Node(const Node &other);

  //! Move constructor
  Node(Node &&other) noexcept = delete;

  //! Destructor
  virtual ~Node() noexcept;

  //! Copy assignment operator
  Node &operator=(const Node &other);

  //! Move assignment operator
  Node &operator=(Node &&other) noexcept;

  double getX() const { return _x; }
  double getY() const { return _y; }
  std::vector<NodePtr> neighbours() const { return _neighbours; }

  Node &addNeighbour(const NodePtr other);
  DistanceResult getDistancetoNeighbour(const NodePtr other);

protected:
private:
  double _x;
  double _y;
  std::vector<NodePtr> _neighbours;
  std::map<NodePtr, double> _cachedDistancesToNeighbours;
};

double calculateDistanceBetweenNodes(const NodePtr a, const NodePtr b);

class DistanceResult {
public:
  //! Default constructor
  DistanceResult();

  DistanceResult(const bool validResult, const double result)
      : valid(validResult), distance(result) {}

  virtual ~DistanceResult() noexcept {}

  bool valid;
  double distance;
};

Angle angle(const NodePtr a, const NodePtr b, const NodePtr c);

#endif /* NODE_HPP */

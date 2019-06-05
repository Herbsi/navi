#ifndef NODE_HPP
#define NODE_HPP

#include <cmath>
#include <map>
#include <vector>

class Edge;

class Node {

  friend class Edge;

public:
  //! Default constructor
  Node();

  Node(const double x, const double y, const std::vector<Node *> neighbours,
       const std::map<Node *, Edge *> neighbour_edge_dict);

  //! Copy constructor
  Node(const Node &other);

  //! Move constructor
  Node(Node &&other) noexcept;

  //! Destructor
  virtual ~Node() noexcept;

  //! Copy assignment operator
  Node &operator=(const Node &other);

  //! Move assignment operator
  Node &operator=(Node &&other) noexcept;

  double getX() const { return _x; }
  double getY() const { return _y; }

protected:
private:
  double _x;
  double _y;
  std::vector<Node *> _neighbours;
  std::map<Node *, Edge *> _neighbour_edge_dict;
};

class Edge {

public:
  //! Default constructor
  Edge();

  Edge(const Node *a, const Node *b);

  //! Copy constructor
  Edge(const Edge &other);

  //! Move constructor
  Edge(Edge &&other) noexcept;

  //! Destructor
  virtual ~Edge() noexcept;

  //! Copy assignment operator
  Edge &operator=(const Edge &other);

  //! Move assignment operator
  Edge &operator=(Edge &&other) noexcept;

  double getLength() const { return _length; }
  Node *getPointA() const { return _a; }
  Node *getPointB() const { return _b; }

protected:
private:
  Node *_a;
  Node *_b;
  double _length;
  Edge _calculateLength();
};

#endif /* NODE_HPP */

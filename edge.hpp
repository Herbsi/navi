#ifndef EDGE_HPP
#define EDGE_HPP

class Node;

class Edge {

public:
  //! Default constructor
  Edge();

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
  double _calculateLength();
};

#endif // EDGE_HPP

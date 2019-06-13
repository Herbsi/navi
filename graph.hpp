/**
 * Provides Graph class
 */
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "node.hpp"
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using NodeIndex = unsigned int;
using IndexedNode = std::pair<NodeIndex, NodePtr>;
using Path = std::vector<NodeIndex>;

struct Distance;
struct DNodeContainer;
struct NodeWithAngle;

class Graph {
public:
  //! Default constructor
  Graph();

  //! Parameter constructor
  //
  //! \details Throws std::runtime_error if filename could not be read and
  //! prints to std::cerr Throws std::out_of_range if there's a connection to or
  //! from a node that does not exist
  //
  //! \param[in] filename Input file, certain format expected
  Graph(const std::string &filename);

  //! Copy constructor not needed
  Graph(const Graph &other) = delete;

  //! Move constructor not needed
  Graph(Graph &&other) noexcept = delete;

  //! Destructor
  virtual ~Graph() noexcept {};

  //! Copy assignment operator not needed
  Graph &operator=(const Graph &other) = delete;

  //! Move assignment operator not needed
  Graph &operator=(Graph &&other) noexcept = delete;

  /**
   * \brief Nodes getter
   */
  std::map<NodeIndex, NodePtr> nodes() const { return _nodes; }

  /**
   * \brief Euclidean distance between node at index i and node at index j
   *
   * \details Throws std::out_of_range if i or j are invalid indices and prints
   * to std::cerr
   *
   * \param[in] i First node index
   * \param[in] j Second node index
   *
   * \return Euclidean distance or -1 if no edge exists between i & j
   */
  double edgeLength(const NodeIndex i, const NodeIndex j);

  /**
   * \brief           Total path length of nodes in passed vector
   *
   * \details          Throws std::out_of_range if a node index in path was
   * invalid and prints error message to std::cerr \param[in]       path Vector
   * of node indices
   *
   * \return          total euclidian distance between neighbouring nodes or -1
   * if no path exists
   */
  double pathLength(const Path &path);

  /**
   * \brief           Finds shortest path between i & j in Graph
   *
   * \details          Uses Dijkstra algorithm
   *
   *
   * \warning         There is a path from i to j
   *
   * \return          Path from i to j
   */
  Path navi(const NodeIndex i, const NodeIndex j);

  /**
   * \brief           Finds shortest path between i & j in Graph and adds angle
   * information to each node
   *
   * \details          The angle for i and j will always be zero
   *
   * \param[in]       i Start point
   * \param[in]       j End point
   *
   * \return          Vector containng NodeWithAngle
   */
  std::vector<NodeWithAngle> naviWithAngle(const NodeIndex i,
                                           const NodeIndex j);

protected:
private:
  std::map<NodeIndex, NodePtr>
      _nodes; //!< Map that connects each NodeIndex with a corresponding Node
  std::map<NodeIndex, std::vector<NodeIndex>>
      _neighbours; //!< Maps each NodeIndex to a vector of neighbouring indices

  /**
   * \brief           Helper function for calculating edgeLength, uses my
   * DistanceResult class
   *
   * \details          Throws std::out_of_range and prints to std::cerr if i or
   * j are invalid indices
   *
   * \param[in]       i First Node
   * \param[in]       j Second Node
   *
   * \return          Object of type DistanceResult
   */
  DistanceResult _edgeLength(const NodeIndex i, const NodeIndex j);

  /**
   * \brief           Initialieses a DNodeContainer object according to the
   * initial phase of the Dijkstra Algorithm
   *
   * \param[in]       i Start Index for Dijkstra Algorithm
   *
   * \return          DNodeContainer with all distances == infinite except for
   * the distance of i
   */
  DNodeContainer _initialise(const NodeIndex i);

  /**
   * \brief           Performs the dijkstra algorithm on the DNodeContainer
   *
   * \param[in, out]  dnc DNodeContainer that is changed by the dijkstra
   * algorithm
   *
   * \return          Returns this
   */
  Graph &_dijkstra(DNodeContainer &dnc);

  /**
   * \brief           Builds a path backwards from endpoint
   *
   * \param[in]       endpoint Nomen est omen
   * \param[in]       predecessor  map from NodeIndex to its predecessor in
   * finale path
   *
   * \return          Returns path in correct order, so path[0] is start point
   */
  Path _builtPath(const NodeIndex endpoint,
                  const std::map<NodeIndex, IndexedNode> &predecessor);

  /**
   * \brief           Addes angle information at every node in path
   *
   * \details          Start and end point will have angle zero
   *
   * \param[in]       path Path whose angles are to be determined
   *
   * \return          Vector containing Nodes with Angle information
   */
  std::vector<NodeWithAngle> _builtPathWithAngle(const Path &path);
};

///////////////////////////////////////////////////////////////////////////////
//                                  DISTANCE                                 //
///////////////////////////////////////////////////////////////////////////////

struct Distance {
  //! Default Constructor
  Distance() : Distance(0, false) {}

  //! Parameter constructor
  //
  //! \param[in] d distance value
  //
  //! \warning No check if d >= 0
  Distance(const double d) : Distance(d, false) {}

  //! Parameter constructor
  //
  //! \param[in] d distance value
  //! \param[in] isInfinite true if d is infinite
  Distance(const double d, const bool isInfinite)
      : distance(d), isInf(isInfinite) {}

  //! Destructor
  virtual ~Distance() noexcept {}

  double distance; //!< actual distance value
  bool isInf; //!< states if object is infinite, needed for Dijkstra Algorithm
};

/**
 * \brief           < Operator for Distance class
 * \param[in]       a left side
 * \param[in]       b right side
 * \return          true if left side is smaller
 */
bool operator<(const Distance &a, const Distance &b);

/**
 * \brief           + Operator for Distance class
 *
 * \details          inf + a == a + inf == inf
 *
 * \param[in]       lhs left side
 * \param[in]       rhs right side
 *
 * \return          sum of lhs and rhs, if result is infinite, distance member
 * is set to 0.0
 */
Distance operator+(const Distance &lhs, const Distance &rhs);

///////////////////////////////////////////////////////////////////////////////
//                                   DNodeContainer                          //
///////////////////////////////////////////////////////////////////////////////

struct DNodeContainer {
  std::map<NodeIndex, Distance>
      distances; //!< euclidean distance of node at index
  std::map<NodeIndex, IndexedNode>
      predecessors;                  //!< predecessor of node at index
  std::map<NodeIndex, bool> visited; //!< visited state of node at index
};

///////////////////////////////////////////////////////////////////////////////
//                                  NodeWithAngle                            //
///////////////////////////////////////////////////////////////////////////////

struct NodeWithAngle {
  //! Default constructor
  NodeWithAngle() : NodeWithAngle(0.0, Angle()) {}

  //! Parameter costructor
  NodeWithAngle(const NodeIndex i, const Angle &a) : idx(i), angle(a) {}

  //! Parameter constructor
  //
  //! \param[in] i Node index
  //! \param[in] angle_rad angle in rad
  //! \param[in] isStraight bool if angle_rad is straight or not
  NodeWithAngle(const NodeIndex i, const double angle_rad,
                const bool isStraight)
      : idx(i), angle(angle_rad, isStraight) {}

  //! Destructor
  virtual ~NodeWithAngle() {}

  NodeIndex idx; //!< Index of Node
  Angle angle;   //!< Angle at node, depends on where you are coming from
};

#endif // GRAPH_HPP

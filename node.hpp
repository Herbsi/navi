/**
 * Provides Node class to use in Graphs
 */
#ifndef NODE_HPP
#define NODE_HPP

#include <cmath>
#include <map>
#include <memory>
#include <vector>

class Node;
class DistanceResult;

using NodePtr = std::shared_ptr<Node>;

class Node {

public:
  //! Default constructor
  Node();

  //! Parameter constructor
  //
  //! \param[in] x x Coordinate
  //! \param[in] y y Coordinate
  Node(const double x, const double y);

  //! Parameter constructor
  //
  //! \param[in] x x Coordinate
  //! \param[in] y y Coordinate
  //! \param[in] neighbours vector of neighbours
  Node(const double x, const double y, const std::vector<NodePtr> &neighbours,
       const std::map<NodePtr, double> &cachedDistancesToNeighbours);

  //! Copy constructor
  Node(const Node &other);

  //! Destructor
  virtual ~Node() noexcept;

  //! Copy assignment operator
  Node &operator=(const Node &other);

  //! Getter for x coordinate
  double getX() const { return _x; }

  //! Getter for y coordinate
  double getY() const { return _y; }

  //! Getter for neighbours
  std::vector<NodePtr> neighbours() const { return _neighbours; }

  /**
   * \brief           Addes other as neighbour to Node
   *
   *  \param[in]       other Node to be added as neighbour
   *
   *  \return          this
   */
  Node &addNeighbour(const NodePtr &other);

  /**
   * \brief           getsDistance to Neighbour
   *
   * \param[in]       other Neighbour
   *
   * \return          DistanceResult, member 'valid' is false if other is not
   * actually a neighbour
   */
  DistanceResult getDistancetoNeighbour(const NodePtr &other);

protected:
private:
  double _x;                        //!< x coordinate
  double _y;                        //!< y coordinate
  std::vector<NodePtr> _neighbours; //!< vector with all neighbours
  std::map<NodePtr, double>
      _cachedDistancesToNeighbours; //!< vector with all distances to neighbours
                                    //!< cached, so they only have to be
                                    //!< computed once
};

struct DistanceResult {
  //! Default constructor
  DistanceResult() : DistanceResult(true, 0.0) {}

  //! Parameter constructor
  //
  //! \param[in] validResult true if result is valid
  //! \param[in] result actual result value
  DistanceResult(const bool validResult, const double result)
      : valid(validResult), distance(result) {}

  //! Destructor
  virtual ~DistanceResult() noexcept {}

  bool valid;      //!< nomen est omen
  double distance; //!< nomen est omen
};

struct Angle {
public:
  //! Default constructor
  Angle() : Angle(0.0, true) {}

  //! Parameter constructor
  //
  //! \param[in] angle_rad Angle in radiant
  //! \param[in] straight true if angle_rad is considered straight
  Angle(const double angle_rad, const bool straight)
      : angle(angle_rad), isStraight(straight) {}

  //! Destructor
  virtual ~Angle() {}

  double angle;    //!< angle value in radiant
  bool isStraight; //!< Nomen est omen
};

/**
 * \brief           Calculates angle at node b, so angle between vec(b-a) and
 * vec(c-b)
 *
 * \param[in]       a First Node
 * \param[in]       b Second Node
 * \param[in]       c Third Node
 *
 * \warning         Does not check whether a, b & c are actually neighbours
 * \return          Angle object, isStraight == true if angle is less than 10
 * degrees
 */
Angle angle(const NodePtr &a, const NodePtr &b, const NodePtr &c);

#endif /* NODE_HPP */

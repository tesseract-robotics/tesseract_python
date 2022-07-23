namespace tesseract_common
{
class JointState
{
public:
  JointState() = default;
  JointState(std::vector<std::string> joint_names, Eigen::VectorXd position);

  /** @brief The joint corresponding to the position vector. */
  std::vector<std::string> joint_names;

  /** @brief The joint position at the waypoint */
  Eigen::VectorXd position;

  /** @brief The velocity at the waypoint (optional) */
  Eigen::VectorXd velocity;

  /** @brief The Acceleration at the waypoint (optional) */
  Eigen::VectorXd acceleration;

  /** @brief The Effort at the waypoint (optional) */
  Eigen::VectorXd effort;

  /** @brief The Time from start at the waypoint (optional) */
  double time{ 0 };

  bool operator==(const JointState& other) const;

  bool operator!=(const JointState& rhs) const;

};

}  // namespace tesseract_common

%template(JointStates) std::vector<tesseract_common::JointState>;

namespace tesseract_common
{
/** @brief Represents a joint trajectory */
class JointTrajectory
{
public:
  JointTrajectory(std::string description = "");
  JointTrajectory(std::vector<JointState> states, std::string description = "");

  std::vector<JointState> states;
  std::string description;

  bool operator==(const JointTrajectory& other) const;

  bool operator!=(const JointTrajectory& rhs) const;

  ///////////////////////////
  // C++ container support //
  ///////////////////////////

  /** value_type */
  using value_type = JointState;
  /** pointer */
  using pointer = typename std::vector<value_type>::pointer;
  /** const_pointer */
  using const_pointer = typename std::vector<value_type>::const_pointer;
  /** reference */
  using reference = typename std::vector<value_type>::reference;
  /** const_reference */
  using const_reference = typename std::vector<value_type>::const_reference;
  /** size_type */
  using size_type = typename std::vector<value_type>::size_type;
  /** difference_type */
  using difference_type = typename std::vector<value_type>::difference_type;

  %ignore get_allocator;
  %ignore resize;
  %ignore assign;
  %ignore swap;
  %ignore insert;
  %ignore JointTrajectory();
  %ignore JointTrajectory(const JointTrajectory&);
  %ignore JointTrajectory(size_type);
  %ignore JointTrajectory(size_type, value_type const &);
  %swig_vector_methods(tesseract_common::JointTrajectory)
  %std_vector_methods(JointTrajectory)
};

}  // namespace tesseract_common


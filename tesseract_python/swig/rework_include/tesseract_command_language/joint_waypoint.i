namespace tesseract_planning
{
class JointWaypoint
{
public:

  JointWaypoint() = default;

  void print(const std::string& prefix = "") const;

  Eigen::VectorXd waypoint;
  std::vector<std::string> joint_names;
  /** @brief Joint distance below waypoint that is allowed. Each element should be <= 0 */
  Eigen::VectorXd lower_tolerance;
  /** @brief Joint distance above waypoint that is allowed. Each element should be >= 0 */
  Eigen::VectorXd upper_tolerance;

  /**
   * @brief Returns true if waypoint is toleranced
   * @return True if waypoint is toleranced
   */
  bool isToleranced() const;

  bool operator==(const JointWaypoint& rhs) const;
  bool operator!=(const JointWaypoint& rhs) const;

};
}  // namespace tesseract_planning

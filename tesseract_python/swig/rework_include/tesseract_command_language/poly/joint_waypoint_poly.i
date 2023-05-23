namespace tesseract_planning
{
class JointWaypoint;
struct JointWaypointPoly
{
  JointWaypointPoly();
  JointWaypointPoly(const JointWaypointPoly&);
  JointWaypointPoly(const JointWaypoint&);
  void setNames(const std::vector<std::string>& names);
  const std::vector<std::string>& getNames() const;

  void setPosition(const Eigen::VectorXd& position);
  const Eigen::VectorXd& getPosition() const;

  void setUpperTolerance(const Eigen::VectorXd& upper_tol);
  const Eigen::VectorXd& getUpperTolerance() const;

  void setLowerTolerance(const Eigen::VectorXd& lower_tol);
  const Eigen::VectorXd& getLowerTolerance() const;

  void setIsConstrained(bool value);
  bool isConstrained() const;

  void setName(const std::string& name);
  const std::string& getName() const;

  void print(const std::string& prefix = "") const;

  /**
   * @brief Returns true if waypoint has tolerances
   * @return True if waypoint has tolerances
   */
  bool isToleranced() const;

};

}  // namespace tesseract_planning
namespace tesseract_planning
{
class CartesianWaypoint;
struct CartesianWaypointPoly
{
  CartesianWaypointPoly();
  CartesianWaypointPoly(const CartesianWaypointPoly&);
  
  void setTransform(const Eigen::Isometry3d& transform);
  const Eigen::Isometry3d& getTransform() const;

  void setUpperTolerance(const Eigen::VectorXd& upper_tol);
  const Eigen::VectorXd& getUpperTolerance() const;

  void setLowerTolerance(const Eigen::VectorXd& lower_tol);
  const Eigen::VectorXd& getLowerTolerance() const;

  void setSeed(const tesseract_common::JointState& seed);
  const tesseract_common::JointState& getSeed() const;

  void setName(const std::string& name);
  const std::string& getName() const;

  void print(const std::string& prefix = "") const;

  /**
   * @brief Check if it has a seed. If the position or joint names is empty this returns false
   * @return True if it has a seed, otherwise false
   */
  bool hasSeed() const;

  /** @brief Clear the seed to empty data structures */
  void clearSeed();

  /**
   * @brief Returns true if waypoint has tolerances
   * @return True if waypoint has tolerances
   */
  bool isToleranced() const;
};

}  // namespace tesseract_planning

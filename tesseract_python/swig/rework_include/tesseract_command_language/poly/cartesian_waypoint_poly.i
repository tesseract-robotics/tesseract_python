namespace tesseract_planning
{
struct CartesianWaypointPoly
{
  CartesianWaypointPoly();
  void setTransform(const Eigen::Isometry3d& transform);
  Eigen::Isometry3d& getTransform();
  const Eigen::Isometry3d& getTransform() const;

  void setUpperTolerance(const Eigen::VectorXd& upper_tol);
  Eigen::VectorXd& getUpperTolerance();
  const Eigen::VectorXd& getUpperTolerance() const;

  void setLowerTolerance(const Eigen::VectorXd& lower_tol);
  Eigen::VectorXd& getLowerTolerance();
  const Eigen::VectorXd& getLowerTolerance() const;

  void setSeed(const tesseract_common::JointState& seed);
  tesseract_common::JointState& getSeed();
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

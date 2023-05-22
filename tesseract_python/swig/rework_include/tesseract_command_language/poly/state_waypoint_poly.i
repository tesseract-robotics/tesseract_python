namespace tesseract_planning
{

struct StateWaypointPoly
{

  StateWaypointPoly();
  
  void setNames(const std::vector<std::string>& names);
  std::vector<std::string>& getNames();
  const std::vector<std::string>& getNames() const;

  void setPosition(const Eigen::VectorXd& position);
  Eigen::VectorXd& getPosition();
  const Eigen::VectorXd& getPosition() const;

  void setVelocity(const Eigen::VectorXd& velocity);
  Eigen::VectorXd& getVelocity();
  const Eigen::VectorXd& getVelocity() const;

  void setAcceleration(const Eigen::VectorXd& acceleration);
  Eigen::VectorXd& getAcceleration();
  const Eigen::VectorXd& getAcceleration() const;

  void setEffort(const Eigen::VectorXd& effort);
  Eigen::VectorXd& getEffort();
  const Eigen::VectorXd& getEffort() const;

  void setTime(double time);
  double getTime() const;

  void setName(const std::string& name);
  const std::string& getName() const;

  void print(const std::string& prefix = "") const;

};

}  // namespace tesseract_planning

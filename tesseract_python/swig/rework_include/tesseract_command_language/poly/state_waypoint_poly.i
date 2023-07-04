namespace tesseract_planning
{
class StateWaypoint;
struct StateWaypointPoly
{

  StateWaypointPoly();
  StateWaypointPoly(const StateWaypointPoly&);
  
  void setNames(const std::vector<std::string>& names);
  const std::vector<std::string>& getNames() const;

  void setPosition(const Eigen::VectorXd& position);
  const Eigen::VectorXd& getPosition() const;

  void setVelocity(const Eigen::VectorXd& velocity);
  const Eigen::VectorXd& getVelocity() const;

  void setAcceleration(const Eigen::VectorXd& acceleration);
  const Eigen::VectorXd& getAcceleration() const;

  void setEffort(const Eigen::VectorXd& effort);
  const Eigen::VectorXd& getEffort() const;

  void setTime(double time);
  double getTime() const;

  void setName(const std::string& name);
  const std::string& getName() const;

  void print(const std::string& prefix = "") const;

};

}  // namespace tesseract_planning

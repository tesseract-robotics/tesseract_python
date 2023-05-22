namespace tesseract_planning
{

struct WaypointPoly
{
  WaypointPoly();

  void setName(const std::string& name);
  const std::string& getName() const;

  void print(const std::string& prefix = "") const;

  bool isCartesianWaypoint() const;

  bool isJointWaypoint() const;

  bool isStateWaypoint() const;

};

}  // namespace tesseract_planning
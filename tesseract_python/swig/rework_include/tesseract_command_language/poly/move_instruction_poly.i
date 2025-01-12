namespace tesseract_planning
{
struct MoveInstructionPoly;

enum class MoveInstructionType : int
{
  LINEAR = 0,
  FREESPACE = 1,
  CIRCULAR = 2,
};
}  // namespace tesseract_planning

namespace tesseract_planning
{
class MoveInstruction;
struct MoveInstructionPoly
{
  MoveInstructionPoly();
  MoveInstructionPoly(const MoveInstructionPoly&);

  const boost::uuids::uuid& getUUID() const;
  void regenerateUUID();

  const boost::uuids::uuid& getParentUUID() const;
  void setParentUUID(const boost::uuids::uuid& uuid);

  void assignCartesianWaypoint(CartesianWaypointPoly waypoint);
  void assignJointWaypoint(JointWaypointPoly waypoint);
  void assignStateWaypoint(StateWaypointPoly waypoint);
  WaypointPoly& getWaypoint();
  const WaypointPoly& getWaypoint() const;

  void setManipulatorInfo(tesseract_common::ManipulatorInfo info);
  const tesseract_common::ManipulatorInfo& getManipulatorInfo() const;
  tesseract_common::ManipulatorInfo& getManipulatorInfo();

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  void setPathProfile(const std::string& profile);
  const std::string& getPathProfile() const;

  void setProfileOverrides(ProfileOverrides profile_overrides);
  const ProfileOverrides getProfileOverrides() const;

  void setPathProfileOverrides(ProfileOverrides profile_overrides);
  const ProfileOverrides getPathProfileOverrides() const;

  void setMoveType(MoveInstructionType move_type);
  MoveInstructionType getMoveType() const;

  const std::string& getDescription() const;
  void setDescription(const std::string& description);

  void print(const std::string& prefix = "") const;

  CartesianWaypointPoly createCartesianWaypoint() const;
  JointWaypointPoly createJointWaypoint() const;
  StateWaypointPoly createStateWaypoint() const;

  // MoveInstructionPoly methods

  MoveInstructionPoly createChild() const;

  bool isLinear() const;

  bool isFreespace() const;

  bool isCircular() const;

  bool isChild() const;

};

}  // namespace tesseract_planning

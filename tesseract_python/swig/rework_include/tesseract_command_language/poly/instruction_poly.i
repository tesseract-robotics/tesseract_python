namespace tesseract_planning
{
struct InstructionPoly
{
  InstructionPoly();
  InstructionPoly(const InstructionPoly&);
  const boost::uuids::uuid& getUUID() const;
  void regenerateUUID();

  const boost::uuids::uuid& getParentUUID() const;
  void setParentUUID(const boost::uuids::uuid& uuid);

  const std::string& getDescription() const;

  void setDescription(const std::string& description);

  void print(const std::string& prefix = "") const;

  bool isCompositeInstruction() const;

  bool isMoveInstruction() const;

};

}  // namespace tesseract_planning

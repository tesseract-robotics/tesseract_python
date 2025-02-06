namespace tesseract_planning
{
enum class CompositeInstructionOrder
{
  ORDERED,               // Must go in forward
  UNORDERED,             // Any order is allowed
  ORDERED_AND_REVERABLE  // Can go forward or reverse the order
};

class flattenFilterFn;
class locateFilterFn;

class CompositeInstruction
{
public:

  CompositeInstruction(std::string profile = DEFAULT_PROFILE_KEY,
                       tesseract_common::ManipulatorInfo manipulator_info = tesseract_common::ManipulatorInfo(),
                       CompositeInstructionOrder order = CompositeInstructionOrder::ORDERED);

  CompositeInstructionOrder getOrder() const;

  const boost::uuids::uuid& getUUID() const;
  void regenerateUUID();

  const boost::uuids::uuid& getParentUUID() const;
  void setParentUUID(const boost::uuids::uuid& uuid);

  void setDescription(const std::string& description);
  const std::string& getDescription() const;

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  /** @brief Dictionary of profiles that will override named profiles for a specific task*/
  void setProfileOverrides(ProfileOverrides profile_overrides);
  ProfileOverrides getProfileOverrides() const;

  void setManipulatorInfo(tesseract_common::ManipulatorInfo info);
  const tesseract_common::ManipulatorInfo& getManipulatorInfo() const;
  tesseract_common::ManipulatorInfo& getManipulatorInfo();

  void setInstructions(std::vector<tesseract_planning::InstructionPoly> instructions);
  std::vector<tesseract_planning::InstructionPoly>& getInstructions();
  const std::vector<tesseract_planning::InstructionPoly>& getInstructions() const;

  void appendMoveInstruction(const MoveInstructionPoly& mi);
  void appendMoveInstruction(const MoveInstructionPoly&& mi);

  void print(const std::string& prefix = "") const;

  bool operator==(const CompositeInstruction& rhs) const;

  bool operator!=(const CompositeInstruction& rhs) const;

  /**
   * @brief Get the first Move Instruction in a Composite Instruction
   * This does not consider the start instruction in child composite instruction
   * @param composite_instruction Composite Instruction to search
   * @return The first Move Instruction (Non-Const)
   */
  MoveInstructionPoly* getFirstMoveInstruction();

  /**
   * @brief Get the first Move Instruction in a Composite Instruction
   * This does not consider the start instruction in child composite instruction
   * @param composite_instruction Composite Instruction to search
   * @return The first Move Instruction (Const)
   */
  const MoveInstructionPoly* getFirstMoveInstruction() const;

  /**
   * @brief Get the last Move Instruction in a Composite Instruction
   * This does not consider the start instruction in child composite instruction
   * @param composite_instruction Composite Instruction to search
   * @return The last Move Instruction (Non-Const)
   */
  MoveInstructionPoly* getLastMoveInstruction();

  /**
   * @brief Get the last Move Instruction in a Composite Instruction
   * This does not consider the start instruction in child composite instruction
   * @param composite_instruction Composite Instruction to search
   * @return The last Move Instruction (Const)
   */
  const MoveInstructionPoly* getLastMoveInstruction() const;

  /**
   * @brief Get number of Move Instruction in a Composite Instruction
   * This does not consider the start instruction in the child composite instruction
   * @param composite_instruction The Composite Instruction to process
   * @return The number of Move Instructions
   */
  long getMoveInstructionCount() const;

  /**
   * @brief Get the first Instruction in a Composite Instruction that is identified by the filter
   * @param composite_instruction Composite Instruction to search
   * @param locate_filter The filter to indicate if an instruction should be considered
   * @param process_child_composites Indicate if child Composite Instructions should be searched
   * @return The first Instruction (Const)
   */
  const InstructionPoly* getFirstInstruction(/*const locateFilterFn& locate_filter = nullptr,
                                             bool process_child_composites = true*/) const;

  /**
   * @brief Get the first Instruction in a Composite Instruction that is identified by the filter
   * @param composite_instruction Composite Instruction to search
   * @param locate_filter The filter to indicate if an instruction should be considered
   * @param process_child_composites Indicate if child Composite Instructions should be searched
   * @return The first Instruction (Non-Const)
   */
  InstructionPoly* getFirstInstruction(/*const locateFilterFn& locate_filter = nullptr,
                                       bool process_child_composites = true*/);

  /**
   * @brief Get the last Instruction in a Composite Instruction that is identified by the filter
   * @param composite_instruction Composite Instruction to search
   * @param locate_filter The filter to indicate if an instruction should be considered
   * @param process_child_composites Indicate if child Composite Instructions should be searched
   * @return The Last Instruction (Const)
   */
  const InstructionPoly* getLastInstruction(/*const locateFilterFn& locate_filter = nullptr,
                                            bool process_child_composites = true*/) const;

  /**
   * @brief Get the last Instruction in a Composite Instruction that is identified by the filter
   * @param composite_instruction Composite Instruction to search
   * @param locate_filter The filter to indicate if an instruction should be considered
   * @param process_child_composites Indicate if child Composite Instructions should be searched
   * @return The Last Instruction (Non-Const)
   */
  InstructionPoly* getLastInstruction(/*const locateFilterFn& locate_filter = nullptr,
                                      bool process_child_composites = true*/);

  /**
   * @brief Get number of Instruction in a Composite Instruction
   * @param composite_instruction The Composite Instruction to process
   * @param locate_filter The filter to indicate if an instruction should be considered
   * @param process_child_composites Indicate if child Composite Instructions should be searched
   * @return The number of Instructions
   */
  long getInstructionCount(/*const locateFilterFn& locate_filter = nullptr, bool process_child_composites = true*/) const;

  /**
   * @brief Flattens a CompositeInstruction into a vector of Instruction
   * @param composite_instruction Input composite instruction to be flattened
   * @param filter Used to filter only what should be considered. Should return true to include otherwise false
   * @return A new flattened vector referencing the original instruction elements
   */
  std::vector<std::reference_wrapper<InstructionPoly>> flatten(/*const flattenFilterFn& filter = nullptr*/);

  /**
   * @brief Flattens a CompositeInstruction into a vector of Instruction&
   * @param instruction Input composite instruction to be flattened
   * @param filter Used to filter only what should be considered. Should return true to include otherwise false
   * @return A new flattened vector referencing the original instruction elements
   */
  std::vector<std::reference_wrapper<const InstructionPoly>> flatten(/*const flattenFilterFn& filter = nullptr*/) const;

  // C++ container support

  /** value_type */
  using value_type = tesseract_planning::InstructionPoly;
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
  %ignore CompositeInstruction();
  %ignore CompositeInstruction(const CompositeInstruction&);
  %ignore CompositeInstruction(size_type);
  %ignore CompositeInstruction(size_type, value_type const &); 
  %swig_vector_methods(tesseract_planning::CompositeInstruction)
  %std_vector_methods(CompositeInstruction)

};

}  // namespace tesseract_planning

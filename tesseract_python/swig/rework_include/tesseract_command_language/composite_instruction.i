namespace tesseract_planning
{
enum class CompositeInstructionOrder
{
  ORDERED,               // Must go in forward
  UNORDERED,             // Any order is allowed
  ORDERED_AND_REVERABLE  // Can go forward or reverse the order
};

class CompositeInstruction
{
public:
  

  CompositeInstruction(std::string profile = DEFAULT_PROFILE_KEY,
                       CompositeInstructionOrder order = CompositeInstructionOrder::ORDERED,
                       ManipulatorInfo manipulator_info = ManipulatorInfo());

  CompositeInstructionOrder getOrder() const;

  void setDescription(const std::string& description);
  const std::string& getDescription() const;

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  /** @brief Dictionary of profiles that will override named profiles for a specific task*/
  ProfileDictionary::Ptr profile_overrides;

  void setManipulatorInfo(ManipulatorInfo info);
  const ManipulatorInfo& getManipulatorInfo() const;
  ManipulatorInfo& getManipulatorInfo();

  void setStartInstruction(Instruction instruction);

  void resetStartInstruction();
  const Instruction& getStartInstruction() const;
  Instruction& getStartInstruction();
  bool hasStartInstruction() const;

  void setInstructions(std::vector<tesseract_planning::Instruction> instructions);
  std::vector<tesseract_planning::Instruction>& getInstructions();
  const std::vector<tesseract_planning::Instruction>& getInstructions() const;

  void print(const std::string& prefix = "") const;

  bool operator==(const CompositeInstruction& rhs) const;

  bool operator!=(const CompositeInstruction& rhs) const;

  // C++ container support

  /** value_type */
  using value_type = Instruction;
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

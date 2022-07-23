namespace tesseract_kinematics
{
/**
 * @brief Structure containing the data required to solve inverse kinematics
 * @details This structure provides the ability to specify IK targets for arbitrary tool links and defined with respect
 * to arbitrary reference frames. Under the hood, the KinematicGroup class will transform these poses appropriately into
 * the correct working frame required by the inverse kinematics solver. This improves the flexibility and ease-of-use of
 * this class for performing IK
 */
struct KinGroupIKInput
{

  KinGroupIKInput(const Eigen::Isometry3d& p, std::string wf, std::string tl);
  KinGroupIKInput() = default;

  /** @brief The desired inverse kinematic pose */
  Eigen::Isometry3d pose;

  /**
   * @brief The link name the pose is relative to
   * @details The provided working frame must be listed in InverseKinematics::getWorkingFrames()
   */
  std::string working_frame;

  /**
   * @brief The tip link of the kinematic object to solve IK
   * @details The provided tip link name must be listed in InverseKinematics::getTipLinkNames()
   */
  std::string tip_link_name;  // This defines the internal kinematic group the information belongs to
};

// clang-format off
%tesseract_aligned_vector_using(KinGroupIKInputs,tesseract_kinematics::KinGroupIKInput)
// clang-format on


class KinematicGroup : public JointGroup
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<KinematicGroup>;
  using ConstPtr = std::shared_ptr<const KinematicGroup>;
  using UPtr = std::unique_ptr<KinematicGroup>;
  using ConstUPtr = std::unique_ptr<const KinematicGroup>;

  ~KinematicGroup() override = default;
  KinematicGroup(const KinematicGroup& other);
  KinematicGroup& operator=(const KinematicGroup& other);
  KinematicGroup(KinematicGroup&&) = default;
  KinematicGroup& operator=(KinematicGroup&&) = default;

  /**
   * @brief Create a kinematics group with inverse kinematics
   * @param name The name of the kinematic group
   * @param inv_kin The inverse kinematics object to create kinematic group from
   * @param scene_graph The scene graph
   * @param scene_state The scene state
   */
  KinematicGroup(std::string name,
                 std::vector<std::string> joint_names,
                 InverseKinematics::UPtr inv_kin,
                 const tesseract_scene_graph::SceneGraph& scene_graph,
                 const tesseract_scene_graph::SceneState& scene_state);

  /**
   * @brief Calculates joint solutions given a pose.
   * @details If redundant solutions are needed see utility function getRedundantSolutions.
   * @param tip_link_poses The input information to solve inverse kinematics for. There must be an input for each link
   * provided in getTipLinkNames
   * @param seed Vector of seed joint angles (size must match number of joints in robot chain)
   * @return A vector of solutions, If empty it failed to find a solution (including uninitialized)
   */
  IKSolutions calcInvKin(const KinGroupIKInputs& tip_link_poses, const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  /**
   * @brief Calculates joint solutions given a pose.
   * @details If redundant solutions are needed see utility function getRedundantSolutions.
   * @param tip_link_pose The input information to solve inverse kinematics for. This is a convenience function for
   * when only one tip link exists
   * @param seed Vector of seed joint angles (size must match number of joints in robot chain)
   * @return A vector of solutions, If empty it failed to find a solution (including uninitialized)
   */
  IKSolutions calcInvKin(const KinGroupIKInput& tip_link_pose, const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  /** @brief Returns all possible working frames in which goal poses can be defined
   * @details The inverse kinematics solver requires that all poses be defined relative to a single working frame.
   * However if this working frame is static, a pose can be defined in another static frame in the environment and
   * transformed into the IK solver working frame. Similarly if the working frame is an active link (e.g. attached to a
   * part positioner), a target pose can also be defined relative to any child link of the working frame and transformed
   * into the IK solver working frame. This function identifies all of these other possible working frames and performs
   * the appropriate transformations internally when solving IK.
   */
  std::vector<std::string> getAllValidWorkingFrames() const;

  /** @brief Get the tip link name
   * @details The inverse kinematics solver requires that all poses be solved for a set of tip link frames that
   * terminate a kinematic chain or tree. In the case of some closed-form IK solvers, this tip link must the tool flange
   * of the robot rather than another preferred link. If the desired tip link is a statically connected child of the
   * required IK solver tip link, then the target IK pose can be transformed into a target pose for the IK solver tip
   * link. This function identifies all possible tip links that can be used for IK (i.e. static child links of the IK
   * solver tip link(s)) and internally performs the appropriate transformations when solving IK
   */
  std::vector<std::string> getAllPossibleTipLinkNames() const;

};

}  // namespace tesseract_kinematics



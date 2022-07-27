namespace tesseract_planning
{
class CartesianWaypoint
{
public:
  

  CartesianWaypoint() = default;

  void print(const std::string& prefix = "") const;

  /////////////////////
  // Eigen Container //
  /////////////////////

  using ConstLinearPart = Eigen::Isometry3d::ConstLinearPart;
  using LinearPart = Eigen::Isometry3d::LinearPart;
  using ConstTranslationPart = Eigen::Isometry3d::ConstTranslationPart;
  using TranslationPart = Eigen::Isometry3d::TranslationPart;

  ////////////////////////
  // Eigen Constructors //
  ////////////////////////

  // This constructor allows you to construct from Eigen expressions
  template <typename OtherDerived>
  CartesianWaypoint(const Eigen::MatrixBase<OtherDerived>& other);

  CartesianWaypoint(const Eigen::Isometry3d& other);

  ///////////////////
  // Eigen Methods //
  ///////////////////

  Eigen::Matrix3d linear() const;
  Eigen::Matrix3d linear();
  Eigen::Vector3d translation() const;
  Eigen::Vector3d translation();


  /** @returns true if two are approximate */
  inline bool isApprox(const Eigen::Isometry3d& other, double prec = 1e-12) const;

  /////////////////////
  // Eigen Operators //
  /////////////////////

  // This method allows you to assign Eigen expressions to MyVectorType
  template <typename OtherDerived>
  inline CartesianWaypoint& operator=(const Eigen::MatrixBase<OtherDerived>& other);

  template <typename OtherDerived>
  inline CartesianWaypoint& operator*=(const Eigen::MatrixBase<OtherDerived>& other);

  template <typename OtherDerived>
  inline CartesianWaypoint operator*(const Eigen::MatrixBase<OtherDerived>& other) const;

  inline CartesianWaypoint& operator=(const Eigen::Isometry3d& other);

  inline CartesianWaypoint& operator*=(const Eigen::Isometry3d& other);

  inline CartesianWaypoint operator*(const Eigen::Isometry3d& other) const;

  //////////////////////////
  // Implicit Conversions //
  //////////////////////////

  /** @return Implicit Conversions to read-only Eigen::Isometry3d */
  inline operator const Eigen::Isometry3d&() const;

  /** @return Implicit Conversions to writable Eigen::Isometry3d */
  inline operator Eigen::Isometry3d&();

  //////////////////////////////////
  // Cartesian Waypoint Container //
  //////////////////////////////////

  /** @brief The Cartesian Waypoint */
  Eigen::Isometry3d waypoint;
  /** @brief Distance below waypoint that is allowed. Should be size = 6. First 3 elements are dx, dy, dz. The last 3
   * elements are angle axis error allowed (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle()) */
  Eigen::VectorXd lower_tolerance;
  /** @brief Distance above waypoint that is allowed. Should be size = 6. First 3 elements are dx, dy, dz. The last 3
   * elements are angle axis error allowed (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle())*/
  Eigen::VectorXd upper_tolerance;

  /**
   * @brief Waypoint seed associated with this Cartesian waypoint
   * @details The waypoint seed can be used for purposes like:
   *   - providing a joint state seed to an IK solver
   *   - providing a seed to the IK solver with modified limits using the tolerances
   *   - providing a joint state to be used by a motion planner for interpolation to avoid performing IK
   */
  Waypoint seed;

  /**
   * @brief Returns true if waypoint has tolerances
   * @return True if waypoint has tolerances
   */
  bool isToleranced() const;
  bool operator==(const CartesianWaypoint& rhs) const;
  bool operator!=(const CartesianWaypoint& rhs) const;

};

}  // namespace tesseract_planning



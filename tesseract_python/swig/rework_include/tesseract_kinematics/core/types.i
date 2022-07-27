namespace tesseract_kinematics
{
/** @brief The inverse kinematics solutions container */
using IKSolutions = std::vector<Eigen::VectorXd>;

/** @brief The Universal Robot kinematic parameters */
struct URParameters
{
  URParameters();
  URParameters(double d1, double a2, double a3, double d4, double d5, double d6);

  double d1;
  double a2;
  double a3;
  double d4;
  double d5;
  double d6;
};

/** @brief The UR10 kinematic parameters */
const URParameters UR10Parameters;

/** @brief The UR5 kinematic parameters */
const URParameters UR5Parameters;

/** @brief The UR3 kinematic parameters */
const URParameters UR3Parameters;

/** @brief The UR10e kinematic parameters */
const URParameters UR10eParameters;

/** @brief The UR5e kinematic parameters */
const URParameters UR5eParameters;

/** @brief The UR3e kinematic parameters */
const URParameters UR3eParameters;

}  // namespace tesseract_kinematics

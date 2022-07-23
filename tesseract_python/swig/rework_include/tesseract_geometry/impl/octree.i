namespace tesseract_geometry
{
class Octree : public Geometry
{
public:
  using Ptr = std::shared_ptr<Octree>;
  using ConstPtr = std::shared_ptr<const Octree>;

  enum SubType
  {
    BOX,
    SPHERE_INSIDE,
    SPHERE_OUTSIDE
  };

  Octree() = default;
  ~Octree() override = default;

  SubType getSubType() const { return sub_type_; }

  bool getPruned() const { return pruned_; }

  Geometry::Ptr clone() const override final { return std::make_shared<Octree>(octree_, sub_type_); }
  bool operator==(const Octree& rhs) const;
  bool operator!=(const Octree& rhs) const;

  /**
   * @brief Octrees are typically generated from 3D sensor data so this method
   * should be used to efficiently update the collision shape.
   */
  void update() { assert(false); }  // NOLINT

  /**
   * @brief Calculate the number of sub shapes that would get generated for this octree
   *
   * This is expensive and should not be called multiple times
   *
   * @return number of sub shapes
   */
  long calcNumSubShapes() const;


};
}  // namespace tesseract_geometry


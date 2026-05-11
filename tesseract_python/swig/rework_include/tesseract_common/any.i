namespace tesseract::common
{
using AnyBase = tesseract::common::TypeErasureBase<TypeErasureInterface, detail_any::AnyInstance>;

struct Any : AnyBase
{
  using AnyBase::AnyBase;
};

}  // namespace tesseract::common

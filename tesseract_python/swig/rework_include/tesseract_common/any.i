namespace tesseract_common
{
using AnyBase = tesseract_common::TypeErasureBase<TypeErasureInterface, detail_any::AnyInstance>;

struct Any : AnyBase
{
  using AnyBase::AnyBase;
};

}  // namespace tesseract_common

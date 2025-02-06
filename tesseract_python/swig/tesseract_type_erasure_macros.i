%define %tesseract_erasure_ctor(class_type,class_namespace,inner_type,inner_namespace)
%inline  {
  class_namespace::class_type class_type ## _wrap_ ## inner_type (inner_namespace::inner_type  inner_waypoint)
  {
     return class_namespace::class_type (inner_waypoint);
  }
}
%enddef

%define %tesseract_erasure_ctor_shared_ptr(class_type,class_namespace,inner_type,inner_namespace)
%inline  {
  class_namespace::class_type class_type ## _wrap_ ## inner_type (std::shared_ptr<inner_namespace::inner_type>  inner_waypoint)
  {
     return class_namespace::class_type (inner_waypoint);
  }

  class_namespace::class_type class_type ## _wrap_ ## inner_type ## Const (std::shared_ptr<const inner_namespace::inner_type>  inner_waypoint)
  {
     return class_namespace::class_type (inner_waypoint);
  }
}
%enddef

%define %tesseract_erasure_ctor2(class_type,class_namespace,inner_type)
%inline  {
  class_namespace::class_type class_type ## _wrap_ ## inner_type (inner_type  inner_waypoint)
  {
     return class_namespace::class_type (inner_waypoint);
  }
}
%enddef

%define %tesseract_erasure_as(source_class_type,source_class_namespace,dest_class_type,dest_class_namespace)

%inline {
  dest_class_namespace::dest_class_type source_class_type ## _as_ ## dest_class_type (source_class_namespace::source_class_type& self)
  {
    return self.as<dest_class_namespace::dest_class_type>();
  }
}
%enddef

%define %tesseract_erasure_as_shared_ptr(source_class_type,source_class_namespace,dest_class_type,dest_class_namespace)

%inline {
  std::shared_ptr<dest_class_namespace::dest_class_type> source_class_type ## _as_ ## dest_class_type (source_class_namespace::source_class_type& self)
  {
    return self.as<std::shared_ptr<dest_class_namespace::dest_class_type>>();
  }

  std::shared_ptr<const dest_class_namespace::dest_class_type> source_class_type ## _as_ ## dest_class_type ## Const (source_class_namespace::source_class_type& self)
  {
    return self.as<std::shared_ptr<const dest_class_namespace::dest_class_type>>();
  }
}
%enddef

%define %tesseract_erasure_as2(source_class_type,source_class_namespace,dest_class_type)

%inline {
  dest_class_type source_class_type ## _as_ ## dest_class_type (source_class_namespace::source_class_type& self)
  {
    return self.as<dest_class_type>();
  }
}
%enddef

%define %tesseract_erasure_is(source_class_type,source_class_namespace,dest_class_type,dest_class_namespace)
%inline {
  bool source_class_type ## _is_ ## dest_class_type (const source_class_namespace::source_class_type& self)
  {
    return self.getType() == typeid(dest_class_namespace::dest_class_type);
  }
}
%enddef

%define %tesseract_erasure_is_shared_ptr(source_class_type,source_class_namespace,dest_class_type,dest_class_namespace)
%inline {
  bool source_class_type ## _is_ ## dest_class_type (const source_class_namespace::source_class_type& self)
  {
    return self.getType() == typeid(std::shared_ptr<dest_class_namespace::dest_class_type>);
  }

  bool source_class_type ## _is_ ## dest_class_type ## Const (const source_class_namespace::source_class_type& self)
  {
    return self.getType() == typeid(std::shared_ptr<const dest_class_namespace::dest_class_type>);
  }
}
%enddef

%define %tesseract_erasure_is2(source_class_type,source_class_namespace,dest_class_type)
%inline {
  bool source_class_type ## _is_ ## dest_class_type (const source_class_namespace::source_class_type& self)
  {
    return self.getType() == typeid(dest_class_type);
  }
}
%enddef

%define %tesseract_any_poly_type(TYPE,NAMESPACE)
%tesseract_erasure_ctor(AnyPoly,tesseract_common,TYPE,NAMESPACE)
%tesseract_erasure_as(AnyPoly,tesseract_common,TYPE,NAMESPACE)
%tesseract_erasure_is(AnyPoly,tesseract_common,TYPE,NAMESPACE)
%enddef

%define %tesseract_any_poly_type_shared_ptr(TYPE,NAMESPACE)
%tesseract_erasure_ctor_shared_ptr(AnyPoly,tesseract_common,TYPE,NAMESPACE)
%tesseract_erasure_as_shared_ptr(AnyPoly,tesseract_common,TYPE,NAMESPACE)
%tesseract_erasure_is_shared_ptr(AnyPoly,tesseract_common,TYPE,NAMESPACE)
%enddef

%define %tesseract_any_poly_type2(TYPE)
%tesseract_erasure_ctor2(AnyPoly,tesseract_common,TYPE)
%tesseract_erasure_as2(AnyPoly,tesseract_common,TYPE)
%tesseract_erasure_is2(AnyPoly,tesseract_common,TYPE)
%enddef
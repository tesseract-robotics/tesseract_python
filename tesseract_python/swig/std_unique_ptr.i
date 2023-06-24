// https://stackify.dev/940798-how-to-handle-unique-ptrs-with-swig

%include <swigmove.i>

namespace std {
  // %feature("novaluewrapper") unique_ptr;
  template <typename Type>
  struct unique_ptr {
     typedef Type* pointer;

     explicit unique_ptr( pointer Ptr );
     unique_ptr (unique_ptr&& Right);
     template<class Type2, Class Del2> unique_ptr( unique_ptr<Type2, Del2>&& Right );
     unique_ptr( const unique_ptr& Right) = delete;


     pointer operator-> () const;
     pointer release ();
     void reset (pointer __p=pointer());
     void swap (unique_ptr &__u);
     pointer get () const;
     operator bool () const;

     ~unique_ptr();
  };
}

%define %unique_ptr_value_wrapper(Type)
  %{
    // Override SwigValueWrapper<std::unique_ptr<Type>>

    template<> class SwigValueWrapper<std::unique_ptr< Type >> {
    
      struct SwigSmartPointer {
        std::unique_ptr< Type > *ptr;
        SwigSmartPointer(std::unique_ptr< Type > *p) : ptr(p) { }
        ~SwigSmartPointer() { delete ptr; }
        SwigSmartPointer& operator=(SwigSmartPointer& rhs) { std::unique_ptr< Type >* oldptr = ptr; ptr = 0; delete oldptr; ptr = rhs.ptr; rhs.ptr = 0; return *this; }
        void reset(std::unique_ptr< Type > *p) { std::unique_ptr< Type >* oldptr = ptr; ptr = 0; delete oldptr; ptr = p; }
      } pointer;
      SwigValueWrapper& operator=(const SwigValueWrapper& rhs);
      SwigValueWrapper(const SwigValueWrapper& rhs);
    public:
      SwigValueWrapper() : pointer(0) { }
      SwigValueWrapper& operator=(std::unique_ptr< Type >& t) {
        std::unique_ptr<Type>* tmp1 = new std::unique_ptr<Type>();
        t.swap(*tmp1);
        SwigSmartPointer tmp(tmp1); 
        pointer = tmp; 
        return *this; 
      }
      SwigValueWrapper& operator=(std::unique_ptr< Type >&& t) { SwigSmartPointer tmp(new std::unique_ptr< Type >(std::move(t))); pointer = tmp; return *this; }
      operator std::unique_ptr< Type >&&() const { return std::move(*pointer.ptr); }
      std::unique_ptr< Type > *operator&() const { return pointer.ptr; }
      static void reset(SwigValueWrapper& t, std::unique_ptr< Type > *p) { t.pointer.reset(p); }
    };

  %}
%enddef

%define %wrap_unique_ptr(Name, Type)

  %unique_ptr_value_wrapper(Type);

  // %typemap(out) std::unique_ptr<Type> (std::unique_ptr<Type>* temp) %{
  //   temp = new $1_ltype();
  //   $1.swap((std::unique_ptr<Type>&&)temp);
  //   $result = SWIG_NewPointerObj(temp, $&1_descriptor, SWIG_POINTER_OWN);
  // %}


  // #if defined(__cplusplus) && defined(%implicitconv_flag)
  // %typemap(in,implicitconv=1) std::unique_ptr<Type> (void *argp, int res = 0) {
  //   res = SWIG_ConvertPtr($input, &argp, $&descriptor, %convertptr_flags | %implicitconv_flag);
  //   if (!SWIG_IsOK(res)) {
  //     %argument_fail(res, "$type", $symname, $argnum); 
  //   }  
  //   if (!argp) { 
  //     %argument_nullref("$type", $symname, $argnum);
  //   } else {
  //     $&ltype temp = %reinterpret_cast(argp, $&ltype);
  //     $1.swap(*temp);
  //     if (SWIG_IsNewObj(res)) %delete(temp);
  //   }
  // }
  // #else
  // %typemap(in) std::unique_ptr<Type> (void *argp, int res = 0) {
  //   res = SWIG_ConvertPtr($input, &argp, $&descriptor, %convertptr_flags);
  //   if (!SWIG_IsOK(res)) {
  //     %argument_fail(res, "$type", $symname, $argnum); 
  //   }  
  //   if (!argp) { 
  //     %argument_nullref("$type", $symname, $argnum);
  //   } else {
  //     $1.swap(*(%reinterpret_cast(argp, $&ltype)));
  //   }
  // }
  // #endif

  %typemap(memberin) std::unique_ptr<Type> %{
    $1 = std::move(*$input);
  %}


  %valuewrapper std::unique_ptr<Type>;
  %apply SWIGTYPE MOVE { std::unique_ptr<Type> }
  %template(Name) std::unique_ptr<Type>;
  %newobject std::unique_ptr<Type>::release;



  /*%typemap(out) std::unique_ptr<Type> const %{
    $result = SWIG_NewPointerObj(new $1_ltype(std::move($1)), $&1_descriptor, SWIG_POINTER_OWN);
  %}*/

  

%enddef

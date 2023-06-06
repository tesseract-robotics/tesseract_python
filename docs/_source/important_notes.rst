Important Usage Notes
=====================

This page contains important notes for using the Tesseract Python bindings.

SWIG Generation
---------------

The Python bindings are generated using the Simplified Wrapper and Interface Generator (SWIG). SWIG is used
to wrap C++, and provides an interface somewhat like having C++ directly in Python. Almost the entire Tesseract
API is available in Python thanks to the SWIG-generated bindings. To simplify the wrapping process, no attempt
has been made to make the interface "Pythonic", in the sense that the wrapped objects still behave like C++ in terms
of memory management and passing parameters. This means that the underlying C++ objects can still be destroyed
even if the Python object is valid. This can lead to crashes or corrupted data if not used carefully. It is also
sometimes not obvious what the memory lifecycle of any particular object is. It is important to carefully read the
C++ documentation to see what the return type of an object is and what its lifecycle is. For example, if a reference 
to an object owned by another object is returned, that value will become invalid once the owning class is destroyed,
even though Python still thinks it has a valid object. For this reason, it is recommended to use the Python bindings
only be used by programmers experienced with both C++ and Python.

C++ Storage and Pointer Types
-------------

The Tesseract library uses several common C++ storage and pointer patterns, including pass-by-copy values, pointers,
references, shared pointers, and unique pointers. These are all wrapped using Python with some clever SWIG techniques.
The lifecycle of all of these objects can be confusing. While it is best to consult the documentation for SWIG and the
C++ wrappers, here is a simple breakdown of the lifecycle of each type:

* pass-by-copy: These objects are wrapped a Python objects, and are copied into the Python object. They are destroyed
  when the Python object is destroyed.
* pointers: These are wrapped as Python objects. If the Python object "owns" the pointer, it will be destroyed when
  the Python object is destroyed. If it is not owned, it will not be destroyed. Context is important to determine
  if the pointer is owned. The `thisown` variable can sometimes be used to determine if the pointer is owned. These
  pointers will become invalid if the owning object is destroyed.
* references: These are wrapped as Python objects. They are not owned by the Python object, and will not be destroyed
  when the Python object is destroyed. They will become invalid if the object they reference is destroyed.
* shared pointers: These are wrapped as Python objects. They are owned by the Python object, and will normally follow
  the typical shared pointer behavior where they are destroyed when the reference count goes to zero. However, there
  are some cases where there will be a "weak" shared pointer returned that does not have ownership. This is rare
  and should not be encountered by the user.
* unique pointers: Unique pointers work by having a single object that is passed between unique pointers to the object.
  Reference counts are not kept, since there is always only one owning unique pointer. These are wrapped in Python
  with the suffix `UPtr` after the name of the class. There is also typically another definition in Python for the class
  without the unique pointer suffix. The real pointer can be accessed by calling the `get()` method on the unique pointer.
  The `UPtr` Python type can be used to construct a new object contained in a unique pointer.

Using the various pointer types can be confusing, and it is important to read the C++ documentation to understand
the lifecycle of any particular object. It is also important to understand the difference between a pointer and a
reference, and how they are used in C++.

It is very easy to make a mistake and have corruption without an error. Check and test your work! Segfaults are also 
common. It may be necessary to use the `del` Python command to delete objects in the correct order to avoid segfaults.

Template Containers
-------------------

The Tesseract library uses C++ container templates extensively. These are wrapped in SWIG. Typical example of 
wrapped templates include `std::vector`, `std::map`, and `std::unordered_map`, etc. Each specific 
implementation of the template is wrapped, for example `std::vector<std::string>` is `StringVector` in Python.
Most of these templates are contained in `tesseract_common`, however they can be found in many of the modules.
These template types roughly correspond to lists and dictionaries in Python. They can normally be implicitly 
created using lists and dictionaries, and when returned have roughly the same accessor methods as lists and
dictionaries. However, they are not the same as lists and dictionaries, and cannot be used interchangeably.

Each module documentation page contains a list of the wrapped template types. The C++ type will sometimes
appear in the Python documentation. Refer to these lists to determine the corresponding Python type.

Eigen Types
-----------

Eigen matrices and vectors are used for extensively in Tesseract. These are automatically converted to NumPy
arrays by the SWIG wrappers. This conversion works for `float64` types (`double` in C++), and `int32` types
(`int` in C++). Other types are not supported. The conversion is done by copying the data. The vectors must
always be column vectors to be converted correctly. See the C++ documentation for the expected shape of the
matrix or vector.

Some Eigen geometry types are wrapped as classes in the `tesseract_common` module. See the `tesseract_common`
documentation for more information and the examples.

Documentation Generation
------------------------

The Python documentation is semi-automatically generated using the SWIG doxygen-docstring feature, Sphinx
autodoc, and several custom scripts. The documentation is not perfect, and may contain errors. Because of the large
number of classes and methods, it is not possible to manually check all of the documentation. If you find an error,
please submit a pull request to fix it.

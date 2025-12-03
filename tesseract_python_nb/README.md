# Tesseract Python (nanobind)

Python bindings for Tesseract robotics using nanobind and scikit-build-core.

## Status

**Prototype Phase**: Currently implementing `tesseract_common` module as proof-of-concept.

✅ Implementation complete, ready for testing with tesseract C++ installed.

## Quick Start

**See [QUICKSTART.md](QUICKSTART.md) for complete setup instructions.**

```bash
# 1. Setup environment
./setup_conda_env.sh
conda activate tesseract_nb

# 2. Build tesseract C++ (from parent dir)
cd .. && ./build_tesseract_cpp.sh

# 3. Build Python bindings
cd tesseract_python_nb
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(pwd)/../ws/install
pip install -e .

# 4. Test
pytest tests/
```

## Requirements

- Python 3.9+
- CMake 3.15+
- nanobind 1.3.2+
- **Tesseract C++ libraries (must be installed first)**
- Eigen, Boost, Bullet, FCL, OMPL, console-bridge, etc.

See [DEPENDENCIES.md](DEPENDENCIES.md) for detailed installation instructions.

## Test

```bash
pytest tests/
```

## Architecture

Migration from SWIG to nanobind:
- Native Eigen support (no custom typemaps needed)
- Trampolines instead of SWIG directors
- Scikit-build-core for modern Python packaging
- Stable ABI support for broader compatibility

## Modules

- [x] tesseract_common (prototype)
- [ ] tesseract_geometry
- [ ] tesseract_scene_graph
- [ ] ... (14 more modules)

## Reference

- Template: `../compas_nanobind_package_template`
- Original SWIG: `../tesseract_python`

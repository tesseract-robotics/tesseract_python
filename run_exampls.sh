cd /Users/jelle/Code/CADCAM/tesseract_python_nanobind
source /opt/miniconda3/etc/profile.d/conda.sh && conda activate tesseract_nb
export DYLD_LIBRARY_PATH=$PWD/ws/install/lib:$DYLD_LIBRARY_PATH
export TESSERACT_RESOURCE_PATH=$PWD/ws/src/tesseract/
python tesseract_viewer_python/examples/abb_irb2400_viewer.py
#python tesseract_viewer_python/examples/shapes_viewer.py
#python tesseract_viewer_python/examples/shapes_viewer.py

import re
from pathlib import Path
import copy
import sys

# py_module_name = "tesseract_common"

build_dir = Path(sys.argv[1]).absolute()

script_dir = Path(__file__).parent.absolute()
repo_root_dir = script_dir.parent.parent.absolute()

src_dir = repo_root_dir / "tesseract_python"
python_dir = build_dir / "tesseract_python/python"
if sys.platform == "win32":
    assert (python_dir / "tesseract_robotics" / "tesseract_common" / "_tesseract_common_python.pyd").is_file()
else:
    assert (python_dir / "tesseract_robotics" / "tesseract_common" / "_tesseract_common_python.so").is_file()
# work_dir = Path("extract_python_docs_work").absolute()
out_dir1 = repo_root_dir / "docs" / "_source" / "modules"
print(out_dir1)
assert out_dir1.is_dir()



def generate_api_docs(py_module_name):
    out_dir = out_dir1  / py_module_name

    # Create work directory
    out_dir.mkdir(exist_ok=True)
    print("Created work directory: ", out_dir)
    with open(src_dir / "swig" / f"{py_module_name}_python.i", "r") as f:
        swig_lines = f.readlines()

    with open(python_dir / "tesseract_robotics" / py_module_name / f"{py_module_name}_python.py", "r") as f:
        py_lines = f.readlines()

    # print(swig_lines)
    # print(py_lines)


    def find_all_class_names():

        # Find name of all class definitions in py_lines that don't start with _
        class_names = []
        for line in py_lines:
            if line.startswith("class ") and not line.startswith("class _"):
                class_names.append(line.split()[1].split("(")[0])

        # Ignore classes that begin with swig or Swig
        class_names = [x for x in class_names if not x.lower().startswith("swig")]

        # Sort class_names
        class_names.sort()

        # print(class_names)
        return class_names

    def find_all_templates():
        # Find all templates in swig_lines that begin with %template(name) and return name and rest of line
        templates = []
        template_re = re.compile(r"^\s*\%template\((\w+)\)\s+(.*);")
        for line in swig_lines:
            match = template_re.match(line)
            if match:
                if match.group(1) == "name":
                    continue
                templates.append((match.group(1), match.group(2)))

        # Sort templates by first element
        templates.sort(key=lambda x: x[0])

        return templates

    def find_container_templates():
        all_templates = find_all_templates()
        container_templates = []
        cpp_template_types = ["std::vector", "std::list", "std::set", "std::unordered_set", "std::map", "std::unordered_map", "std::pair", "std::array"]
        for template in all_templates:
            for cpp_template_type in cpp_template_types:
                if template[1].startswith(cpp_template_type):
                    container_templates.append(template)
                    break

        return container_templates

    def find_eigen_aligned_container_templates():
        template_macros = ["%tesseract_aligned_vector","%tesseract_aligned_map", "%tesseract_aligned_map_of_aligned_vector", "%tesseract_aligned_unordered_map"]
        # Search swig lines for template macros
        template_re = re.compile(r"^\s*\%tesseract_aligned_(\w+)\((\w+),(.*)\)")
        container_templates = []
        for line in swig_lines:
            match = template_re.match(line)
            if match:
                if match.group(2) == "name":
                    continue
                if match.group(1) == "vector":
                    container_templates.append((match.group(2), f"tesseract_common::AlignedVector<{match.group(3)}>"))
                elif match.group(1) == "map":
                    container_templates.append((match.group(2), f"tesseract_common::AlignedMap<{match.group(3)}>"))
                elif match.group(1) == "map_of_aligned_vector":
                    m_key_type, v_vector_type = match.group(3).split(",")
                    container_templates.append((match.group(2), f"tesseract_common::AlignedMap<{m_key_type}, tesseract_common::AlignedVector<{v_vector_type}> >"))
                elif match.group(1) == "unordered_map":
                    container_templates.append((match.group(2), f"tesseract_common::AlignedUnorderedMap<{match.group(3)}>"))
                elif match.group(1) == "vector_using" or match.group(1) == "map_using" or match.group(1) == "unordered_map_using":
                    continue
                else:
                    assert False, f"Invalid eigen aligned template macro type: {match.group(1)}"
        # Sort templates by first element
        container_templates.sort(key=lambda x: x[0])
        return container_templates

    def filter_containers_from_list(class_list, template_containers):
        # Remove template types from class_list that are in template_containers
        filtered_class_list = copy.deepcopy(class_list)
        for template_container in template_containers:
            filtered_class_list = [x for x in filtered_class_list if not x==(template_container[0])]

        return filtered_class_list

    def find_all_def():
        # Find all python function definitions in py_lines that don't start with _ or swig or Swig
        def_names = []
        for line in py_lines:
            if line.startswith("def ") and not line.startswith("def _") and not line.startswith("def swig") and not line.startswith("def Swig"):
                def_names.append(line.split()[1].split("(")[0])

        # Sort def_names
        def_names.sort()

        return def_names
        
    def select_any_poly(all_def):
        ret = []
        for a in all_def:
            if a.startswith("AnyPoly"):
                ret.append(a)
        return ret

    def filter_def_from_list(def_list, filter_list):
        # Remove template types from class_list that are in template_containers
        filtered_def_list = copy.deepcopy(def_list)
        for filter in filter_list:
            filtered_def_list = [x for x in filtered_def_list if not x==(filter)]

        return filtered_def_list
        
    def find_constants():
        # Find all constants with pattern (^[A-Z]\w+_\w+) = 
        constants = []
        constant_re = re.compile(r"^\s*([A-Z]\w+_\w+)\s*=")
        for line in py_lines:
            match = constant_re.match(line)
            if match:
                constants.append(match.group(1))
        
        # remove SHARED_PTR_DISOWN
        constants = [x for x in constants if not x=="SHARED_PTR_DISOWN"]

        # Sort constants
        constants.sort()

        return constants

    # print(find_all_class_names())
    # print(find_container_templates())
    class_list = filter_containers_from_list(find_all_class_names(), find_container_templates())
    class_list = filter_containers_from_list(class_list, find_eigen_aligned_container_templates())

    all_def = find_all_def()
    # filtered_all_def = filter_def_from_list(all_def, select_any_poly(all_def))

    containers = find_container_templates() + find_eigen_aligned_container_templates()

    # any_poly = select_any_poly(all_def)
    any_poly=[]

    jinja_variables = {
        "classes": class_list,
        "constants": find_constants(),
        "functions": all_def,
        "containers": containers,
        "tesseract_module": py_module_name,
        "any_poly": any_poly
    }

    # Run jinja2 tempalate file api_docs_generated.rst.j2
    from jinja2 import Environment, FileSystemLoader, select_autoescape
    env = Environment(
        loader=FileSystemLoader(searchpath=f"{script_dir}"),
        autoescape=select_autoescape(['html', 'xml'])
    )

    template = env.get_template("api_docs_generated.rst.j2")
    output_from_parsed_template = template.render(jinja_variables)


    # print(output_from_parsed_template)

    # Write output to file
    print(f"Writing api_docs_generated.rst to {out_dir}")
    with open(out_dir / "api_docs_generated.rst", "w") as f:
        f.write(output_from_parsed_template)

def extract_py_module_names():
    cmake_filename = src_dir / "CMakeLists.txt"
    with open(cmake_filename, "r") as f:
        lines = f.readlines()

    py_module_names = []
    # match tesseract_python_module(module_name
    re_module = re.compile(r"^\s*tesseract_python_module\((\w+)_python")
    for line in lines:
        match = re_module.match(line)
        if match:
            py_module_names.append(match.group(1))
    
    return py_module_names

py_module_names = extract_py_module_names()

for py_module_name in py_module_names:
    generate_api_docs(py_module_name)




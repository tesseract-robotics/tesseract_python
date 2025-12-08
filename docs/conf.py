# Configuration file for Sphinx documentation builder.

import os
import sys
import pathlib

# Add docs directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Try to get git branch for versioning
try:
    import git
    repo = git.Repo(os.path.abspath('.'))
    current_branch = repo.active_branch.name
except (git.exc.InvalidGitRepositoryError, ImportError):
    current_branch = 'main'

# -- Project information -----------------------------------------------------

project = 'Tesseract Robotics Python (nanobind)'
copyright = '2024, Wason Technology LLC, Jelle Feringa'
author = 'John Wason, Jelle Feringa'
version = current_branch
release = current_branch

# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',  # Google-style docstrings
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'myst_parser',  # Markdown support
]

# Markdown configuration
myst_enable_extensions = [
    'colon_fence',
    'deflist',
    'tasklist',
]
myst_heading_anchors = 3

# Napoleon settings for Google-style docstrings
napoleon_google_docstring = True
napoleon_numpy_docstring = False
napoleon_include_init_with_doc = True
napoleon_include_private_with_doc = False

# Source file settings
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}
master_doc = 'index'

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', 'readme.md']

# -- Options for HTML output -------------------------------------------------

html_theme = 'sphinx_rtd_theme'

html_theme_options = {
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    'collapse_navigation': True,
    'sticky_navigation': False,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False,
}

html_static_path = ['_static']

html_context = {
    'display_github': True,
    'github_user': 'tesseract-robotics',
    'github_repo': 'tesseract_nanobind',
    'github_version': current_branch,
    'conf_py_path': 'docs/',
}

htmlhelp_basename = 'TesseractNanobindDocumentation'

# Copy README.md to docs
readme_path = pathlib.Path(__file__).parent.resolve().parent / 'README.md'
readme_target = pathlib.Path(__file__).parent / 'readme.md'

if readme_path.exists():
    with readme_target.open('w') as outf:
        outf.write('# Readme\n\n')
        for line in readme_path.read_text().split('\n'):
            if line.startswith('# '):
                continue  # Skip title
            line = line.replace('docs/figures', 'figures/')
            line = line.replace('docs/', '')
            outf.write(line + '\n')

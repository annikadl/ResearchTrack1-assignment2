# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import os
import subprocess
import sys

sys.path.insert(0, os.path.abspath('../'))
subprocess.call('doxygen Doxyfile.in', shell=True)
show_authors = True

project = 'assignment_2_2023'
copyright = '2024, Annika Delucchi'
author = 'Annika Delucchi'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

autodoc_mock_imports = ["rospy", "actionlib", "actionlib_msgs", "assignment_2_2023"]

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    "sphinx.ext.napoleon",
    'sphinx.ext.inheritance_diagram',
    'breathe'
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

source_suffix = '.rst'
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# -- Options for intersphinx extension ---------------------------------------
# Example configuration for intersphinx: refer to the Python standard library.

intersphinx_mapping = {'python': ('https://docs.python.org/3', None)}

# -- Options for todo extension ----------------------------------------------
# If true, todo and todoList produce output, else they produce nothing.

todo_include_todos = True

# -- Options for breathe -----------------------------------------------------
breathe_projects = {
    "assignment_2_2023": "../build/xml/"
}
breathe_default_project = "assignment_2_2023"
breathe_default_members = ('members', 'undoc-members')

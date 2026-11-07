# Configuration file for the Sphinx documentation builder.

import subprocess, os
read_the_docs_build = os.environ.get('READTHEDOCS', None) == 'True'

# Build doxygen XML if on building on RTD server
if read_the_docs_build:
     subprocess.call('cd ../doxygen; doxygen', shell=True)

# -- Project information

project = 'JOVER'
copyright = '2026, Space and Satellite Systems'
author = 'SSS'

release = '0.1'
version = '0.1.0'

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'myst_parser',
]

templates_path = ['_templates']

# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'

# -- Options for EPUB output
epub_show_urls = 'footnote'

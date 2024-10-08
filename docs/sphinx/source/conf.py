# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "hddc2b"
copyright = "2024, Sven Schneider"
author = "Sven Schneider"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["sphinx.ext.mathjax", "sphinx.ext.todo", "breathe"]

templates_path = ["_templates"]
exclude_patterns = []

numfig = True
math_numfig = True
numfig_format = {
    "figure": "Figure %s",
    "table": "Table %s",
    "code-block": "Listing %s",
    "section": "Section %s"
}

todo_include_todos = True

# -- Options for LaTeX output ------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-latex-output

latex_elements = {
    "preamble": r"\newcommand{\vect}{\boldsymbol}"
} 

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_css_files = ["css/custom.css"]
html_js_files = ["js/load-mathjax.js"]

# -- MathJax -----------------------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/math.html

mathjax_path = "https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"

# -- Breathe -----------------------------------------------------------------
# https://breathe.readthedocs.io/en/latest/quickstart.html

breathe_domain_by_extension = {
    "c": "c",
    "h": "c"
}

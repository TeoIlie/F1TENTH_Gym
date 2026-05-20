import os
import sys

sys.path.insert(0, os.path.abspath(".."))

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}
source_encoding = "utf-8-sig"

# -- Theme -------------------------------------------------------------------
html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "logo_only": True,
    "collapse_navigation": False,
    "prev_next_buttons_location": "bottom",
}
html_context = {
    "display_github": True,
    "github_user": "TeoIlie",
    "github_repo": "Gym-Khana",
    "github_version": "main",
    "conf_py_path": "/docs/",
}

html_favicon = "assets/favicon.png"
html_logo = "assets/gymkhana_flipped.svg"

# -- Project information -----------------------------------------------------

project = "Gym-Khana"
copyright = "2021-2026, Teodor Ilie, Hongrui Zheng, Matthew O'Kelly, Aman Sinha"
author = "Teodor Ilie"

# Pull version from pyproject.toml via installed package metadata
try:
    from importlib.metadata import version as get_version

    release = get_version("gymkhana")
except Exception:
    release = os.environ.get("READTHEDOCS_VERSION", "latest")
version = ".".join(release.split(".")[:2])


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx_rtd_theme",
    "sphinx.ext.autosectionlabel",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "myst_parser",
]
pygments_style = "emacs"
autodoc_member_order = "bysource"
autosectionlabel_prefix_document = True
autosectionlabel_maxdepth = 2  # Avoid duplicate labels from repeated headings (e.g. changelog)

# Napoleon settings
napoleon_google_docstring = True
napoleon_numpy_docstring = False
napoleon_use_param = True
napoleon_use_rtype = True

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "plan"]

html_static_path = ["_static"]
html_css_files = ["css/custom.css"]

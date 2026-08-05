.. image:: assets/logo.png
   :width: 20
   :align: left
   :alt: logo

Known Issues
============

Windows
-------

.. warning::

   Library support issues on Windows. You must use Python 3.8 as of 10-2021.

macOS Big Sur and above
-----------------------

.. warning::

   When rendering is turned on, you might encounter:

   .. code::

      ImportError: Can't find framework /System/Library/Frameworks/OpenGL.framework.

   Fix by installing a newer version of pyglet:

   .. code:: bash

      pip3 install pyglet==1.5.11

   You might see a warning like:

   .. code::

      gym 0.17.3 requires pyglet<=1.5.0,>=1.4.0, but you'll have pyglet 1.5.11 which is incompatible.

   This can be safely ignored. The environment will still work without error.

.. _acados-vcs-versioning:

acados_template install fails with ``No module named 'vcs_versioning'``
-----------------------------------------------------------------------

.. warning::

   ``pip install -e ~/software/acados/interfaces/acados_template`` fails during metadata generation:

   .. code::

      ModuleNotFoundError: No module named 'vcs_versioning'
      error: metadata-generation-failed

   **Cause.** The acados ``setup.py`` declares ``setup_requires=['setuptools_scm']``, which setuptools
   resolves through the legacy ``fetch_build_eggs()`` path. That downloads the dependency as a bare
   ``.egg`` into a local ``.eggs/`` directory *without* resolving its own dependencies. Since
   ``setuptools_scm`` 10.x moved its core into a separate ``vcs-versioning`` distribution, the result is a
   half-installed egg. Setuptools then imports every ``distutils.setup_keywords`` entry point it finds on
   ``sys.path``, hits the broken egg, and aborts. Nothing is wrong with the cmake build or the environment
   variables.

   **Fix.** Install ``setuptools_scm`` with pip first — pip resolves ``vcs-versioning`` correctly — so the
   legacy path sees the requirement already satisfied and skips the egg download. The stale ``.eggs/``
   directory must also be removed, since it stays on ``sys.path`` and reproduces the error otherwise.

   .. code:: bash

      rm -rf ~/software/acados/interfaces/acados_template/.eggs
      pip install setuptools_scm
      pip install -e ~/software/acados/interfaces/acados_template

   Run all three inside the target virtual environment. Recreating the virtual environment drops the
   pre-installed ``setuptools_scm`` and brings the failure back, so repeat these steps after any rebuild.

.. note::

   ``~/software/acados/bin/t_renderer`` is absent after a source build. This is not an error —
   ``acados_template`` downloads the binary automatically on the first ``AcadosOcpSolver(...)`` call.
   Expect a one-time network fetch when you first build a solver.

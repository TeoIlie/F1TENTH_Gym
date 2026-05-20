.. image:: assets/logo.png
   :width: 20
   :align: left
   :alt: logo

Installation
============

Gym-Khana is a pure Python package.

.. tip::

   We recommend installing inside a virtual environment to avoid dependency conflicts.

Using pip (recommended)
-----------------------

.. code:: bash

   virtualenv gym_env
   source gym_env/bin/activate
   git clone --recurse-submodules https://github.com/TeoIlie/Gym-Khana.git
   cd Gym-Khana
   pip install -e .

Using poetry
------------

.. code:: bash

   poetry install
   source $(poetry env info -p)/bin/activate  # or prefix commands with `poetry run`

.. _additional-dependencies:

Additional dependencies
-----------------------

.. note::

   MPC controllers require dependencies that cannot be installed via pip alone. These are optional — the core environment and RL training work without them.

For the reference MPC implementation see the ForzaETH `race_stack <https://github.com/ForzaETH/race_stack>`_.

**acados** (build from source) — see the official `installation docs <https://docs.acados.org/installation/index.html>`_ and `Python interface docs <https://docs.acados.org/python_interface/index.html>`_:

.. code:: bash

   # Clone and build (~/software is only an example install directory)
   git clone https://github.com/acados/acados.git --recurse-submodules ~/software/acados
   cd ~/software/acados && mkdir build && cd build
   cmake -DACADOS_WITH_QPOASES=ON ..
   make install -j$(nproc)

   # Environment variables (add to shell profile)
   export ACADOS_SOURCE_DIR=~/software/acados
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/software/acados/lib

Install the ``acados_template`` Python interface inside your virtual environment:

.. code:: bash

   pip install -e ~/software/acados/interfaces/acados_template

.. tip::

   **VSCode debugging**: If your IDE reports ``ModuleNotFoundError`` for ``acados_template`` or ``casadi`` when debugging, ensure that:

   1. Your virtual environment is selected as the Python interpreter (``Ctrl+Shift+P`` → *Python: Select Interpreter*).
   2. ``acados_template`` is installed into that virtual environment with ``pip install -e`` as shown above (this lets acados use the virtualenv's ``casadi``).

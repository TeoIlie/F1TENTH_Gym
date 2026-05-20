Sim2Real Transfer
=================

Tools for narrowing the gap between simulated training and real-vehicle
deployment.

Domain Randomization
--------------------

At each reset, selected vehicle parameters are perturbed by multiplicative
Gaussian noise:

.. math::

   \theta' = \theta \cdot X, \quad X \sim \mathcal{N}(1, \sigma)

clipped at :math:`\pm` ``dr_clip_k`` :math:`\cdot \sigma` (default
``dr_clip_k = 3.0``). This widens the distribution of dynamics the policy
sees during training, improving robustness to unknown real-world parameter
offsets.

Configured via the ``domain_randomization`` gym key as a flat
``{param: sigma}`` dict (:math:`\sigma` in :math:`(0, 0.2)`); absent or
``None`` disables DR. Train configs (``get_drift_train_config``,
``get_recovery_train_config``) inject DR from
``train/config/gym_config.yaml``; eval always uses nominal parameters.

.. code-block:: yaml

   # train/config/gym_config.yaml
   domain_randomization:
     m: 0.05         # mass
     I_z: 0.02       # yaw inertia
     lf: 0.05        # CoG fore-distance; lr auto-tracks (wheelbase preserved)
     s_max: 0.03     # max steering angle; s_min auto-mirrors
     sv_max: 0.02    # max steering velocity; sv_min auto-mirrors

For coupled pairs, randomize only the canonical member. The derived partner
is recomputed each reset; listing it raises ``ValueError`` pointing to the
canonical key.

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Canonical
     - Derived
     - Invariant
   * - ``lf``
     - ``lr``
     - ``lf + lr = wheelbase``
   * - ``s_max``
     - ``s_min``
     - ``s_min = -s_max``
   * - ``sv_max``
     - ``sv_min``
     - ``sv_min = -sv_max``

| Item | Symbol / sketch setting | Value | Unit |
|---|---|---:|---|
| Pole pairs | `MOTOR_POLE_PAIRS` |  | — |
| Stored zero electrical angle | `STORED_ZERO_ELECTRIC_ANGLE` |  | rad |
| Stored sensor direction | `STORED_SENSOR_DIRECTION` |  | — |
| Torque constant | `TORQUE_CONSTANT`, $k$ |  | N·m/V |
| Back-EMF constant, low-speed estimate | `MOTOR_KE`, $K_e$ |  | V·s/rad |
| Rod mass | `ROD_MASS`, $m_r$ |  | kg |
| Point mass | `POINT_MASS`, $m_p$ |  | kg |
| Length, axis to point-mass centre | `PENDULUM_LENGTH`, $l$ |  | m |
| Inertia | $J$ |  | kg·m² |
| Gravity-torque coefficient | $G$ |  | N·m |
| Natural frequency | $\sqrt{G/J}$ |  | rad/s |
| Predicted period | $2\pi\sqrt{J/G}$ |  | s |
| Mechanical viscous friction | `VISCOUS_FRICTION`, $b$ |  | N·m·s/rad |
| Mechanical Coulomb friction | `COULOMB_FRICTION`, $\tau_c$ |  | N·m |
| Voltage limit planned for controller labs | `MOTOR_VOLTAGE_LIMIT` | **3 (suggested)** | V |
| Estimated maximum torque at 3 V | $\tau_{max}=3k$ |  | N·m |
| Torque-to-gravity ratio | $\tau_{max}/G$ |  | — |

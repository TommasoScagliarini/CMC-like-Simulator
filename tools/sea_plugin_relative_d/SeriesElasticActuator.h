#ifndef SERIESELASTICACTUATOR_H
#define SERIESELASTICACTUATOR_H

/* -------------------------------------------------------------------------- *
 * SeriesElasticActuator.h                                                    *
 * Inherits from CoordinateActuator for native OpenSim gradient support.      *
 * -------------------------------------------------------------------------- */

#include <OpenSim/OpenSim.h>
// CoordinateActuator is already included transitively by OpenSim/OpenSim.h

using namespace OpenSim;
using namespace SimTK;

class SeriesElasticActuator : public OpenSim::CoordinateActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(SeriesElasticActuator, CoordinateActuator);

public:
    // -----------------------------------------------------------------------
    // Properties
    // NOTE: optimal_force and coordinate socket are already provided by
    //       CoordinateActuator - do NOT redeclare them here.
    // -----------------------------------------------------------------------
    OpenSim_DECLARE_PROPERTY(motor_inertia,  double, "Rotor inertia Jm [kg*m^2]");
    OpenSim_DECLARE_PROPERTY(motor_damping,  double, "Viscous damping Bm [N*m*s/rad]");
    OpenSim_DECLARE_PROPERTY(stiffness,      double, "Spring stiffness K [N*m/rad]");
    OpenSim_DECLARE_PROPERTY(Kp,             double, "Inner torque-loop proportional gain");
    OpenSim_DECLARE_PROPERTY(Kd,             double, "Inner torque-loop derivative gain");
    OpenSim_DECLARE_PROPERTY(Ki,             double, "Inner torque-loop integral gain");
    OpenSim_DECLARE_PROPERTY(integral_torque_limit, double, "Maximum absolute integral torque contribution [N*m]");
    OpenSim_DECLARE_PROPERTY(Kd_Imp,         double, "Middle-loop derivative gain (impedance mode)");
    OpenSim_DECLARE_PROPERTY(Impedence,      bool,   "If true the SEA is controlled by an impedence controller, otherwise by a PD torque controller");

    // -----------------------------------------------------------------------
    // Outputs exposed to the Python simulator
    // These allow the runner to read plugin-computed dynamics without
    // reimplementing the SEA physics in Python.
    // -----------------------------------------------------------------------
    OpenSim_DECLARE_OUTPUT(tau_input,       double, getMotorTorque,    SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(motor_angle_dot, double, getMotorAngleDot,  SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(motor_speed_dot, double, getMotorSpeedDot,  SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(torque_error_integral_dot, double, getTorqueErrorIntegralDot, SimTK::Stage::Dynamics);

    // -----------------------------------------------------------------------
    // Constructors
    // -----------------------------------------------------------------------
    SeriesElasticActuator();

    SeriesElasticActuator(const std::string& name,
                          double inertia,
                          double damping,
                          double k,
                          double Kp,
                          double Kd,
                          double optimal_force,
                          bool   impedence);

    // -----------------------------------------------------------------------
    // Core OpenSim overrides
    // -----------------------------------------------------------------------

    /** Returns the spring torque (non-ideal) or u*F_opt (ideal). */
    double computeActuation(const SimTK::State& s) const override;

    // computeForce() is NOT overridden: CoordinateActuator::computeForce()
    // already calls computeActuation() and applies the generalised force
    // correctly, giving OpenSim the analytic gradient it needs.

    /** Motor dynamics: d/dt [theta_m, omega_m]. Delegates to the three
     *  methods below so the same logic is accessible via Output. */
    void computeStateVariableDerivatives(const SimTK::State& s) const override;

    // -----------------------------------------------------------------------
    // Plugin output methods (also bound to OpenSim Outputs above)
    // -----------------------------------------------------------------------

    /** Motor input torque tau_input [N*m], clamped to +/-500 N*m. */
    double getMotorTorque(const SimTK::State& s) const;

    /** Motor angle derivative = omega_m [rad/s]. */
    double getMotorAngleDot(const SimTK::State& s) const;

    /** Motor speed derivative = alpha_m [rad/s^2]. */
    double getMotorSpeedDot(const SimTK::State& s) const;

    /** Inner torque-error integral derivative = tau_ref - tau_spring. */
    double getTorqueErrorIntegralDot(const SimTK::State& s) const;

    // -----------------------------------------------------------------------
    // Accessors / Utilities
    // -----------------------------------------------------------------------
    double getSpeed (const SimTK::State& s) const override;
    double getStress(const SimTK::State& s) const override;
    double getPower (const SimTK::State& s) const override;

protected:
    // -----------------------------------------------------------------------
    // OpenSim component pipeline overrides
    // -----------------------------------------------------------------------
    void extendAddToSystem            (SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s)               const override;

private:
    void constructProperties();
    double getTorqueError(const SimTK::State& s) const;
    double getIntegralTorqueContribution(const SimTK::State& s) const;
    double getMotorTorqueRaw(const SimTK::State& s) const;
};

#endif // SERIESELASTICACTUATOR_H

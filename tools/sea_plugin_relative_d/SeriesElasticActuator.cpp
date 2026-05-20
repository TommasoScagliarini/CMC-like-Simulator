/* -------------------------------------------------------------------------- *
 * SeriesElasticActuator.cpp                                                  *
 * Inherits from CoordinateActuator for native OpenSim gradient support.      *
 * -------------------------------------------------------------------------- */

#include "SeriesElasticActuator.h"
#include <OpenSim/OpenSim.h>
#include <algorithm>
#include <cmath>

using namespace OpenSim;
using namespace SimTK;

namespace {
constexpr double kMaxMotorTorqueNm = 500.0;
}

//==============================================================================
// CONSTRUCTORS
//==============================================================================

SeriesElasticActuator::SeriesElasticActuator() {
    setAuthors("Tommaso Scagliarini");
    setReferences("Series Elastic Actuator plugin for OpenSim");
    constructProperties();
}

SeriesElasticActuator::SeriesElasticActuator(const std::string& name,
                                             double inertia,
                                             double damping,
                                             double k,
                                             double Kp,
                                             double Kd,
                                             double optimal_force,
                                             bool   impedence)
{
    constructProperties();
    setName(name);
    set_motor_inertia(inertia);
    set_motor_damping(damping);
    set_stiffness(k);
    set_Kp(Kp);
    set_Kd(Kd);
    // CoordinateActuator owns optimal_force; use the native setter.
    setOptimalForce(optimal_force);
    set_Impedence(impedence);
}

//==============================================================================
// PROPERTY CONSTRUCTION
//==============================================================================

void SeriesElasticActuator::constructProperties() {
    constructProperty_motor_inertia(0.01);
    constructProperty_motor_damping(0.1);
    constructProperty_stiffness(250.0);
    constructProperty_Kp(1000.0);
    constructProperty_Kd(20.0);
    constructProperty_Ki(0.0);
    constructProperty_integral_torque_limit(100.0);
    constructProperty_Kd_Imp(20.0);
    constructProperty_Impedence(false);
    // NOTE: do NOT call constructProperty_optimal_force - it belongs to
    //       CoordinateActuator and is already constructed by the parent.
}

//==============================================================================
// 1. REGISTER IN MULTIBODY SYSTEM
//==============================================================================

void SeriesElasticActuator::extendAddToSystem(MultibodySystem& system) const {
    SeriesElasticActuator* mutableThis = const_cast<SeriesElasticActuator*>(this);
    mutableThis->addStateVariable("motor_angle", Stage::Dynamics);
    mutableThis->addStateVariable("motor_speed", Stage::Dynamics);
    mutableThis->addStateVariable("torque_error_integral", Stage::Dynamics);

    // Always call parent last so the coordinate actuator sets itself up
    // after we have registered our extra state variables.
    Super::extendAddToSystem(system);
}

//==============================================================================
// 2. INITIALISE STATE FROM PROPERTIES
//==============================================================================

void SeriesElasticActuator::extendInitStateFromProperties(SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);

    const Coordinate* coord = getCoordinate();
    double start_angle = (coord != nullptr) ? coord->getValue(s) : 0.0;
    // Motor starts at the same angle as the joint so the spring is unloaded.
    setStateVariableValue(s, "motor_angle", start_angle);
    setStateVariableValue(s, "motor_speed", 0.0);
    setStateVariableValue(s, "torque_error_integral", 0.0);
}

//==============================================================================
// 3. PLUGIN OUTPUT METHODS
//==============================================================================

double SeriesElasticActuator::getTorqueError(const SimTK::State& s) const {
    double K = get_stiffness();
    double F_opt = getOptimalForce();

    if (!std::isfinite(K) || std::abs(K) < 1e-9) K = 1e-9;
    if (!std::isfinite(F_opt)) F_opt = 0.0;

    const Coordinate* coord = getCoordinate();
    double theta_joint = (coord != nullptr) ? coord->getValue(s) : 0.0;
    double theta_m = getStateVariableValue(s, "motor_angle");
    double u = getControl(s);

    if (!std::isfinite(theta_joint)) theta_joint = 0.0;
    if (!std::isfinite(theta_m)) theta_m = theta_joint;
    if (!std::isfinite(u)) u = 0.0;

    const double tau_ref = u * F_opt;
    const double tau_spring = K * (theta_m - theta_joint);
    return tau_ref - tau_spring;
}

double SeriesElasticActuator::getIntegralTorqueContribution(const SimTK::State& s) const {
    double Ki = get_Ki();
    double limit = get_integral_torque_limit();
    double xi = getStateVariableValue(s, "torque_error_integral");

    if (!std::isfinite(Ki)) Ki = 0.0;
    if (!std::isfinite(limit)) limit = 0.0;
    if (!std::isfinite(xi)) xi = 0.0;

    limit = std::max(0.0, limit);
    const double tau_i = Ki * xi;
    return std::max(-limit, std::min(limit, tau_i));
}

double SeriesElasticActuator::getMotorTorqueRaw(const SimTK::State& s) const {
    double Bm    = get_motor_damping();
    double K     = get_stiffness();
    double F_opt = getOptimalForce();
    double Kp    = get_Kp();
    double Kd    = get_Kd();
    double KdImp = get_Kd_Imp();

    if (!std::isfinite(Bm)) Bm = 0.0;
    if (!std::isfinite(K) || std::abs(K) < 1e-9) K = 1e-9;
    if (!std::isfinite(F_opt)) F_opt = 0.0;
    if (!std::isfinite(Kp)) Kp = 0.0;
    if (!std::isfinite(Kd)) Kd = 0.0;
    if (!std::isfinite(KdImp)) KdImp = Kd;

    const Coordinate* coord = getCoordinate();
    double theta_joint = (coord != nullptr) ? coord->getValue(s) : 0.0;
    double omega_joint = (coord != nullptr) ? coord->getSpeedValue(s) : 0.0;
    double theta_m = getStateVariableValue(s, "motor_angle");
    double omega_m = getStateVariableValue(s, "motor_speed");
    double u = getControl(s);

    if (!std::isfinite(theta_joint)) theta_joint = 0.0;
    if (!std::isfinite(omega_joint)) omega_joint = 0.0;
    if (!std::isfinite(theta_m)) theta_m = theta_joint;
    if (!std::isfinite(omega_m)) omega_m = 0.0;
    if (!std::isfinite(u)) u = 0.0;

    const double tau_spring = K * (theta_m - theta_joint);
    double tau_input = 0.0;

    if (get_Impedence()) {
        const double tau_ref = u * F_opt;
        const double theta_m_ref = theta_joint + tau_ref / K;
        const double tau_ff = tau_spring + Bm * omega_m;
        tau_input = tau_ff
                  + Kp * (theta_m_ref - theta_m)
                  + KdImp * (omega_joint - omega_m);
    } else {
        const double tau_ref = u * F_opt;
        tau_input = tau_ref
                  + Kp * (tau_ref - tau_spring)
                  + getIntegralTorqueContribution(s)
                  - Kd * omega_m;
    }

    return tau_input;
}

double SeriesElasticActuator::getMotorTorque(const SimTK::State& s) const {
    const double tau_input = getMotorTorqueRaw(s);
    return std::max(-kMaxMotorTorqueNm, std::min(kMaxMotorTorqueNm, tau_input));
}

double SeriesElasticActuator::getMotorAngleDot(const SimTK::State& s) const {
    const double omega_m = getStateVariableValue(s, "motor_speed");
    return std::isfinite(omega_m) ? omega_m : 0.0;
}

double SeriesElasticActuator::getMotorSpeedDot(const SimTK::State& s) const {
    double Jm = get_motor_inertia();
    double Bm = get_motor_damping();
    double K  = get_stiffness();

    if (!std::isfinite(Jm) || Jm < 1e-9) Jm = 1e-9;
    if (!std::isfinite(Bm)) Bm = 0.0;
    if (!std::isfinite(K) || std::abs(K) < 1e-9) K = 1e-9;

    const Coordinate* coord = getCoordinate();
    double theta_joint = (coord != nullptr) ? coord->getValue(s) : 0.0;
    double theta_m = getStateVariableValue(s, "motor_angle");
    double omega_m = getStateVariableValue(s, "motor_speed");

    if (!std::isfinite(theta_joint)) theta_joint = 0.0;
    if (!std::isfinite(theta_m)) theta_m = theta_joint;
    if (!std::isfinite(omega_m)) omega_m = 0.0;

    const double tau_spring = K * (theta_m - theta_joint);
    const double tau_input = getMotorTorque(s);
    const double omega_m_dot = (tau_input - tau_spring - Bm * omega_m) / Jm;
    return std::isfinite(omega_m_dot) ? omega_m_dot : 0.0;
}

double SeriesElasticActuator::getTorqueErrorIntegralDot(const SimTK::State& s) const {
    if (get_Impedence()) {
        return 0.0;
    }

    double Ki = get_Ki();
    double limit = get_integral_torque_limit();
    double xi = getStateVariableValue(s, "torque_error_integral");
    if (!std::isfinite(Ki) || std::abs(Ki) < 1e-12) {
        return 0.0;
    }
    if (!std::isfinite(limit)) limit = 0.0;
    if (!std::isfinite(xi)) xi = 0.0;
    limit = std::max(0.0, limit);

    const double error = getTorqueError(s);
    if (!std::isfinite(error)) {
        return 0.0;
    }

    const double tau_i_unclamped = Ki * xi;
    const double tau_raw = getMotorTorqueRaw(s);
    const bool integral_high = tau_i_unclamped >= limit && error > 0.0;
    const bool integral_low = tau_i_unclamped <= -limit && error < 0.0;
    const bool torque_high = tau_raw >= kMaxMotorTorqueNm && error > 0.0;
    const bool torque_low = tau_raw <= -kMaxMotorTorqueNm && error < 0.0;
    if (integral_high || integral_low || torque_high || torque_low) {
        return 0.0;
    }

    return error;
}

//==============================================================================
// 4. MOTOR DYNAMICS
//==============================================================================

void SeriesElasticActuator::computeStateVariableDerivatives(const SimTK::State& s) const {
    setStateVariableDerivativeValue(s, "motor_angle", getMotorAngleDot(s));
    setStateVariableDerivativeValue(s, "motor_speed", getMotorSpeedDot(s));
    setStateVariableDerivativeValue(s, "torque_error_integral", getTorqueErrorIntegralDot(s));
}

//==============================================================================
// 5. ACTUATION
//==============================================================================

double SeriesElasticActuator::computeActuation(const SimTK::State& s) const {
    if (get_Impedence()) {
        return getOptimalForce() * getControl(s);
    }

    const Coordinate* coord = getCoordinate();
    const double theta_joint = (coord != nullptr) ? coord->getValue(s) : 0.0;
    const double theta_m = getStateVariableValue(s, "motor_angle");
    return get_stiffness() * (theta_m - theta_joint);
}

// NOTE: computeForce() is intentionally NOT overridden.
// CoordinateActuator::computeForce() calls computeActuation() internally,
// registers the actuation via setActuation(), and applies the generalised
// force correctly - giving OpenSim the analytic gradient for free.

//==============================================================================
// 6. ACCESSORS / UTILITIES
//==============================================================================

double SeriesElasticActuator::getSpeed(const SimTK::State& s) const {
    const Coordinate* coord = getCoordinate();
    return (coord != nullptr) ? coord->getSpeedValue(s) : 0.0;
}

double SeriesElasticActuator::getStress(const SimTK::State& s) const {
    const double optForce = getOptimalForce();
    if (optForce < 1e-10) return 0.0;
    return std::abs(getActuation(s)) / optForce;
}

//==============================================================================
// 7. POWER
//==============================================================================

double SeriesElasticActuator::getPower(const SimTK::State& s) const {
    double omega_m = 0.0;
    try {
        omega_m = getStateVariableValue(s, "motor_speed");
    } catch (...) {
        return 0.0;
    }

    return getMotorTorque(s) * omega_m;
}

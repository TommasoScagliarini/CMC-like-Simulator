#ifndef ONLINEGRFSPHEREHALFSPACEFORCE_H
#define ONLINEGRFSPHEREHALFSPACEFORCE_H

#include <OpenSim/OpenSim.h>

class OnlineGRFSphereHalfSpaceForce : public OpenSim::Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(
        OnlineGRFSphereHalfSpaceForce, OpenSim::Force);

public:
    OpenSim_DECLARE_SOCKET(
        sphere_frame, OpenSim::PhysicalFrame,
        "Frame carrying the contact sphere.");

    OpenSim_DECLARE_PROPERTY(
        sphere_location, SimTK::Vec3, "Sphere center in sphere_frame [m].");
    OpenSim_DECLARE_PROPERTY(
        sphere_radius, double, "Contact sphere radius [m].");
    OpenSim_DECLARE_PROPERTY(
        plane_origin, SimTK::Vec3, "Point on the ground plane in ground [m].");
    OpenSim_DECLARE_PROPERTY(
        plane_normal, SimTK::Vec3, "Outward plane normal in ground.");
    OpenSim_DECLARE_PROPERTY(
        surface_velocity, SimTK::Vec3,
        "Ground/treadmill surface velocity in ground [m/s].");
    OpenSim_DECLARE_PROPERTY(
        stiffness, double, "Hertz contact stiffness.");
    OpenSim_DECLARE_PROPERTY(
        exponent, double, "Positive contact force-penetration exponent.");
    OpenSim_DECLARE_PROPERTY(
        dissipation, double, "Hunt-Crossley dissipation.");
    OpenSim_DECLARE_PROPERTY(
        static_friction, double, "Static friction coefficient.");
    OpenSim_DECLARE_PROPERTY(
        dynamic_friction, double, "Dynamic friction coefficient.");
    OpenSim_DECLARE_PROPERTY(
        viscous_friction, double, "Viscous friction coefficient.");
    OpenSim_DECLARE_PROPERTY(
        transition_velocity, double, "Friction transition velocity [m/s].");
    OpenSim_DECLARE_PROPERTY(
        smoothing, double, "Positive-part smoothing length.");

    OnlineGRFSphereHalfSpaceForce();

    OpenSim::Array<std::string> getRecordLabels() const override;
    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override;

protected:
    void computeForce(
        const SimTK::State& state,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& generalizedForces) const override;

private:
    struct ContactResult {
        SimTK::Vec3 force{0};
        SimTK::Vec3 momentAboutGround{0};
        SimTK::Vec3 contactPoint{0};
        SimTK::Vec3 sphereCenter{0};
        SimTK::Vec3 normal{0, 1, 0};
        double normalForce{0};
        double penetration{0};
        double slipSpeed{0};
    };

    void constructProperties();
    ContactResult calcContact(const SimTK::State& state) const;
    static double smoothPositive(double value, double epsilon);
};

#endif

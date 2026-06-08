#include "OnlineGRFSphereHalfSpaceForce.h"

#include <algorithm>
#include <cmath>

using namespace OpenSim;
using namespace SimTK;

OnlineGRFSphereHalfSpaceForce::OnlineGRFSphereHalfSpaceForce() {
    constructProperties();
}

void OnlineGRFSphereHalfSpaceForce::constructProperties() {
    constructProperty_sphere_location(Vec3(0));
    constructProperty_sphere_radius(0.035);
    constructProperty_plane_origin(Vec3(0));
    constructProperty_plane_normal(Vec3(0, 1, 0));
    constructProperty_surface_velocity(Vec3(0));
    constructProperty_stiffness(1.0e6);
    constructProperty_exponent(1.5);
    constructProperty_dissipation(2.0);
    constructProperty_static_friction(0.8);
    constructProperty_dynamic_friction(0.8);
    constructProperty_viscous_friction(0.0);
    constructProperty_transition_velocity(0.2);
    constructProperty_smoothing(1.0e-5);
}

double OnlineGRFSphereHalfSpaceForce::smoothPositive(
        double value, double epsilon) {
    return 0.5 * (value + std::sqrt(value * value + epsilon * epsilon));
}

OnlineGRFSphereHalfSpaceForce::ContactResult
OnlineGRFSphereHalfSpaceForce::calcContact(const State& state) const {
    ContactResult result;
    const auto& frame = getConnectee<PhysicalFrame>("sphere_frame");
    const Vec3 center = frame.findStationLocationInGround(
        state, get_sphere_location());
    const Vec3 velocity = frame.findStationVelocityInGround(
        state, get_sphere_location());

    Vec3 normal = get_plane_normal();
    const double normalNorm = normal.norm();
    if (normalNorm <= SignificantReal) {
        normal = Vec3(0, 1, 0);
    } else {
        normal /= normalNorm;
    }

    const double distance = dot(center - get_plane_origin(), normal);
    const double penetrationRaw = get_sphere_radius() - distance;
    const double epsilon = std::max(get_smoothing(), SignificantReal);
    const double penetration = smoothPositive(penetrationRaw, epsilon);

    const Vec3 relativeVelocity = velocity - get_surface_velocity();
    const double normalVelocity = dot(relativeVelocity, normal);
    const double penetrationRate = -normalVelocity;
    const double dampingFactor = smoothPositive(
        1.0 + get_dissipation() * penetrationRate, epsilon);
    const double normalForce =
        get_stiffness()
        * std::pow(penetration, std::max(get_exponent(), SignificantReal))
        * dampingFactor;

    const Vec3 tangentVelocity =
        relativeVelocity - normalVelocity * normal;
    const double slipSpeed = tangentVelocity.norm();
    const double transition =
        std::max(get_transition_velocity(), SignificantReal);
    const double ratio = slipSpeed / transition;
    const double frictionCoefficient =
        get_dynamic_friction()
        + (get_static_friction() - get_dynamic_friction())
            * std::exp(-ratio * ratio);
    const Vec3 frictionForce =
        -frictionCoefficient * normalForce
            * tangentVelocity / std::sqrt(slipSpeed * slipSpeed
                                          + transition * transition)
        - get_viscous_friction() * tangentVelocity;

    result.force = normalForce * normal + frictionForce;
    result.contactPoint = center - get_sphere_radius() * normal;
    result.momentAboutGround = result.contactPoint % result.force;
    result.sphereCenter = center;
    result.normal = normal;
    result.normalForce = normalForce;
    result.penetration = std::max(0.0, penetrationRaw);
    result.slipSpeed = slipSpeed;
    return result;
}

void OnlineGRFSphereHalfSpaceForce::computeForce(
        const State& state,
        Vector_<SpatialVec>& bodyForces,
        Vector&) const {
    const auto result = calcContact(state);
    const auto& frame = getConnectee<PhysicalFrame>("sphere_frame");
    applyForceToPoint(
        state, frame, get_sphere_location(), result.force, bodyForces);
    applyTorque(
        state, frame,
        (-get_sphere_radius() * result.normal) % result.force,
        bodyForces);
}

Array<std::string> OnlineGRFSphereHalfSpaceForce::getRecordLabels() const {
    Array<std::string> labels;
    const std::string prefix = getName();
    for (const char* axis : {"x", "y", "z"}) {
        labels.append(prefix + ".force." + axis);
    }
    for (const char* axis : {"x", "y", "z"}) {
        labels.append(prefix + ".moment." + axis);
    }
    for (const char* axis : {"x", "y", "z"}) {
        labels.append(prefix + ".point." + axis);
    }
    labels.append(prefix + ".normal_force");
    labels.append(prefix + ".penetration");
    labels.append(prefix + ".slip_speed");
    for (const char* axis : {"x", "y", "z"}) {
        labels.append(prefix + ".sphere_center." + axis);
    }
    for (const char* axis : {"x", "y", "z"}) {
        labels.append(prefix + ".normal." + axis);
    }
    return labels;
}

Array<double> OnlineGRFSphereHalfSpaceForce::getRecordValues(
        const State& state) const {
    const auto result = calcContact(state);
    Array<double> values;
    for (int i = 0; i < 3; ++i) values.append(result.force[i]);
    for (int i = 0; i < 3; ++i) values.append(result.momentAboutGround[i]);
    for (int i = 0; i < 3; ++i) values.append(result.contactPoint[i]);
    values.append(result.normalForce);
    values.append(result.penetration);
    values.append(result.slipSpeed);
    for (int i = 0; i < 3; ++i) values.append(result.sphereCenter[i]);
    for (int i = 0; i < 3; ++i) values.append(result.normal[i]);
    return values;
}

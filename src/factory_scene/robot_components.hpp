#pragma once
#include <string>
#include <entt/entt.hpp>
#include "components.hpp"
#include "pose_component.hpp"

namespace factory {

// A robot's *data* — the renderer (render::robot::Registry) loads the URDF,
// owns the threepp::Robot, and drives IK from this. Keeping threepp out of
// the ECS layer means tests stay headless and the sim doesn't pull in OpenGL.
//
// Behaviour: the robot's base is rendered at `base_position`. Each frame, the
// renderer computes its IK target from `tracks` (the world position of that
// entity becomes the end-effector position target). For a static robot pose
// (no tracking), leave `tracks = entt::null`; the arm holds whatever joint
// values it was last set to.
struct RobotComponent {
    const std::string& urdf_path()     const { return urdf_path_; }
    Vec3               base_position() const { return base_position_; }
    entt::entity       tracks()        const { return tracks_; }

    void set_urdf_path(std::string p)        { urdf_path_     = std::move(p); }
    void set_base_position(Vec3 v)           { base_position_ = v; }
    void set_tracks(entt::entity e)          { tracks_        = e; }

private:
    std::string  urdf_path_;
    Vec3         base_position_ = Vec3{0.f};
    entt::entity tracks_        = entt::null;
};

}  // namespace factory

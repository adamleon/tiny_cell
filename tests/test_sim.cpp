// Unit tests for the simulation: components, sensors, transports, lifecycle.
// No threepp dependency.
//
// Coverage:
//   - Component setter validation (happy path) for new and kept components
//   - spawn_debt accumulation and consume_spawn
//   - port_is_clear: empty list, blocked/unblocked sensors, shared sensors
//   - sensor::scan: physical sensors detect items in volume; virtual untouched
//   - transport::step (belt): advance, freeze on running=false, freeze on
//     blocked gate-port sensor, belt-to-belt handover, source backpressure
//   - transport::step (picker): state machine transitions and side-effects
//   - lifecycle::step: source spawn (gated), sink despawn

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "factory_scene/components.hpp"
#include "factory_scene/factory_scene.hpp"
#include "factory_scene/sensor_systems.hpp"
#include "factory_scene/transport_systems.hpp"
#include "factory_scene/lifecycle_systems.hpp"
#include "factory_scene/station_systems.hpp"
#include "factory_scene/placement_pattern.hpp"
#include "factory_scene/depot_components.hpp"
#include "factory_scene/throughput.hpp"
#include "factory_scene/robot_arm_catalog.hpp"
#include "factory_scene/cost_model.hpp"
#include "factory_scene/cost_options.hpp"
#include "factory_scene/station_solver_types.hpp"

// ── Test harness ──────────────────────────────────────────────────────────────
static int g_run = 0, g_fail = 0;

#define REQUIRE(cond) \
    do { if (!(cond)) throw std::runtime_error("REQUIRE failed: " #cond); } while(0)

#define REQUIRE_EQ(a, b) \
    do { if (!((a) == (b))) throw std::runtime_error( \
        std::string("REQUIRE_EQ: ") + std::to_string(a) + " != " + std::to_string(b)); } while(0)

#define REQUIRE_NEAR(a, b, eps) \
    do { if (std::abs(float(a) - float(b)) > float(eps)) throw std::runtime_error( \
        std::string("REQUIRE_NEAR: ") + std::to_string(a) + " vs " + std::to_string(b)); } while(0)

#define TEST(name) static void test_##name()
#define RUN(name) \
    do { ++g_run; try { test_##name(); printf("  PASS  " #name "\n"); } \
         catch (const std::exception& e) { ++g_fail; printf("  FAIL  " #name " — %s\n", e.what()); } \
    } while(0)

// ── Component setter / getter round-trips ─────────────────────────────────────

TEST(conveyor_belt_setters_store_values) {
    factory::ConveyorBeltComponent bc;
    bc.set_width_mm(500);
    bc.set_length_mm(3000);
    bc.set_surface_height_mm(400);
    bc.set_belt_speed_mm_s(150.f);
    bc.set_dir({0.f, 1.f, 0.f});
    REQUIRE_EQ(bc.width_mm(),          500);
    REQUIRE_EQ(bc.length_mm(),         3000);
    REQUIRE_EQ(bc.surface_height_mm(), 400);
    REQUIRE_NEAR(bc.belt_speed_mm_s(), 150.f, 1e-5f);
    REQUIRE(bc.dir().y == 1.f);
}

TEST(conveyor_belt_zero_speed_is_valid) {
    factory::ConveyorBeltComponent bc;
    bc.set_belt_speed_mm_s(0.f);
    REQUIRE_NEAR(bc.belt_speed_mm_s(), 0.f, 1e-5f);
}

TEST(conveyor_belt_default_gate_ports_empty) {
    // Tap and gate ports default to empty lists; gate_ports gets {exit_port}
    // populated by connect_belt(), not by the component itself.
    factory::ConveyorBeltComponent bc;
    REQUIRE(bc.tap_ports().empty());
    REQUIRE(bc.gate_ports().empty());
}

TEST(conveyor_belt_add_tap_port_appends) {
    factory::ConveyorBeltComponent bc;
    entt::registry reg;
    auto e1 = reg.create();
    auto e2 = reg.create();
    bc.add_tap_port(e1);
    bc.add_tap_port(e2);
    REQUIRE_EQ(static_cast<int>(bc.tap_ports().size()), 2);
    REQUIRE(bc.tap_ports()[0] == e1);
    REQUIRE(bc.tap_ports()[1] == e2);
}

TEST(equipment_cost_defaults_zero) {
    factory::EquipmentCostComponent c;
    REQUIRE_NEAR(c.capex_eur(), 0.f, 1e-5f);
    REQUIRE_NEAR(c.power_w(),   0.f, 1e-5f);
}

TEST(equipment_cost_setters_round_trip) {
    factory::EquipmentCostComponent c;
    c.set_capex_eur(12000.f);
    c.set_power_w(400.f);
    REQUIRE_NEAR(c.capex_eur(), 12000.f, 1e-3f);
    REQUIRE_NEAR(c.power_w(),     400.f, 1e-3f);
}

TEST(transport_running_default_true) {
    factory::TransportComponent tc;
    REQUIRE(tc.running());
}

TEST(transport_set_running_round_trip) {
    factory::TransportComponent tc;
    tc.set_running(false);
    REQUIRE(!tc.running());
    tc.set_running(true);
    REQUIRE(tc.running());
}

TEST(sensor_default_not_blocked) {
    factory::SensorComponent s;
    REQUIRE(!s.blocked());
}

TEST(sensor_set_blocked_round_trip) {
    factory::SensorComponent s;
    s.set_blocked(true);
    REQUIRE(s.blocked());
    s.set_blocked(false);
    REQUIRE(!s.blocked());
}

TEST(port_no_sensors_default) {
    factory::PortComponent pc;
    REQUIRE(pc.sensors().empty());
}

TEST(port_add_sensor_appends) {
    factory::PortComponent pc;
    entt::registry reg;
    auto e1 = reg.create();
    auto e2 = reg.create();
    pc.add_sensor(e1);
    pc.add_sensor(e2);
    REQUIRE_EQ(static_cast<int>(pc.sensors().size()), 2);
    REQUIRE(pc.sensors()[0] == e1);
    REQUIRE(pc.sensors()[1] == e2);
}

TEST(picker_default_state_idle) {
    factory::PickerTransportComponent pt;
    REQUIRE(pt.state() == factory::PickerState::Idle);
    REQUIRE(pt.current_box() == entt::null);
}

TEST(picker_setters_round_trip) {
    factory::PickerTransportComponent pt;
    pt.set_home_pose({100.f, 200.f, 300.f});
    pt.set_pickup_target({1000.f, 0.f, 400.f});
    pt.set_drop_target({2000.f, 0.f, 400.f});
    pt.set_speed_mm_s(500.f);
    pt.set_state(factory::PickerState::MovingToBox);
    REQUIRE_NEAR(pt.home_pose().x, 100.f, 1e-5f);
    REQUIRE_NEAR(pt.pickup_target().x, 1000.f, 1e-5f);
    REQUIRE_NEAR(pt.drop_target().x, 2000.f, 1e-5f);
    REQUIRE_NEAR(pt.speed_mm_s(), 500.f, 1e-5f);
    REQUIRE(pt.state() == factory::PickerState::MovingToBox);
}

TEST(source_rate_round_trip) {
    factory::SourceComponent sc;
    sc.set_rate_per_hour(360.f);
    REQUIRE_NEAR(sc.rate_per_hour(), 360.f, 1e-5f);
}

TEST(source_zero_rate_is_valid) {
    factory::SourceComponent sc;
    sc.set_rate_per_hour(0.f);
    REQUIRE_NEAR(sc.rate_per_hour(), 0.f, 1e-5f);
}

TEST(declared_opening_width_round_trip) {
    factory::DeclaredOpeningComponent oc;
    oc.set_width_mm(900);
    REQUIRE_EQ(oc.width_mm(), 900);
}

TEST(declared_opening_mobility_round_trip) {
    factory::DeclaredOpeningComponent oc;
    oc.set_mobility(0.5f);
    REQUIRE_NEAR(oc.mobility(), 0.5f, 1e-5f);
}

TEST(item_prototype_setters) {
    factory::ItemPrototypeComponent p;
    p.set_length_mm(600);
    p.set_width_mm(400);
    p.set_height_mm(200);
    p.set_color_hex(0xAABBCCu);
    REQUIRE_EQ(p.length_mm(), 600);
    REQUIRE_EQ(p.width_mm(),  400);
    REQUIRE_EQ(p.height_mm(), 200);
    REQUIRE_EQ(p.color_hex(), 0xAABBCCu);
}

// ── spawn_debt mechanics ──────────────────────────────────────────────────────

TEST(spawn_debt_accumulates) {
    factory::SourceComponent sc;
    sc.add_spawn_debt(0.4f);
    sc.add_spawn_debt(0.7f);
    REQUIRE_NEAR(sc.spawn_debt(), 1.1f, 1e-5f);
}

TEST(consume_spawn_decrements_by_one) {
    factory::SourceComponent sc;
    sc.add_spawn_debt(2.3f);
    sc.consume_spawn();
    REQUIRE_NEAR(sc.spawn_debt(), 1.3f, 1e-4f);
}

TEST(consume_spawn_twice) {
    factory::SourceComponent sc;
    sc.add_spawn_debt(2.5f);
    sc.consume_spawn();
    sc.consume_spawn();
    REQUIRE_NEAR(sc.spawn_debt(), 0.5f, 1e-4f);
}

// ── port_is_clear ────────────────────────────────────────────────────────────

TEST(port_clear_with_no_sensors) {
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    REQUIRE(factory::port_is_clear(scene.registry(), port_e));
}

TEST(port_clear_with_unblocked_virtual_sensor) {
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    scene.add_virtual_sensor(port_e);
    REQUIRE(factory::port_is_clear(scene.registry(), port_e));
}

TEST(port_blocked_with_blocked_virtual_sensor) {
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e = scene.add_virtual_sensor(port_e);
    scene.registry().get<factory::SensorComponent>(s_e).set_blocked(true);
    REQUIRE(!factory::port_is_clear(scene.registry(), port_e));
}

TEST(port_blocked_when_any_sensor_blocked) {
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    scene.add_virtual_sensor(port_e);                               // clear
    auto s2 = scene.add_virtual_sensor(port_e);                     // blocked
    scene.registry().get<factory::SensorComponent>(s2).set_blocked(true);
    REQUIRE(!factory::port_is_clear(scene.registry(), port_e));
}

TEST(shared_sensor_gates_multiple_ports) {
    // S2 case: one virtual sensor referenced by two port lists.
    factory::FactoryScene scene;
    auto& reg = scene.registry();
    auto p1 = scene.add_port("p1", {0.f,    0.f, 0.f});
    auto p2 = scene.add_port("p2", {1000.f, 0.f, 0.f});
    auto shared = scene.add_virtual_sensor(p1);
    reg.get<factory::PortComponent>(p2).add_sensor(shared);

    REQUIRE(factory::port_is_clear(reg, p1));
    REQUIRE(factory::port_is_clear(reg, p2));

    reg.get<factory::SensorComponent>(shared).set_blocked(true);
    REQUIRE(!factory::port_is_clear(reg, p1));
    REQUIRE(!factory::port_is_clear(reg, p2));
}

// ── sensor::scan ─────────────────────────────────────────────────────────────

TEST(sensor_scan_no_items_keeps_clear) {
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e    = scene.add_laser_sensor(port_e);

    factory::sensor::scan(scene);

    REQUIRE(!scene.registry().get<factory::SensorComponent>(s_e).blocked());
}

TEST(sensor_scan_point_item_at_laser_blocks) {
    // An item without a prototype has zero extent: it triggers the laser
    // only when its centre exactly coincides with the laser's position.
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e    = scene.add_laser_sensor(port_e);

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {0.f, 0.f, 0.f};
    pose.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(item);

    factory::sensor::scan(scene);

    REQUIRE(reg.get<factory::SensorComponent>(s_e).blocked());
}

TEST(sensor_scan_point_item_far_from_laser_clear) {
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e    = scene.add_laser_sensor(port_e);

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {500.f, 0.f, 0.f};
    pose.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(item);

    factory::sensor::scan(scene);

    REQUIRE(!reg.get<factory::SensorComponent>(s_e).blocked());
}

TEST(virtual_sensor_unaffected_by_scan) {
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e    = scene.add_virtual_sensor(port_e);
    auto& reg   = scene.registry();
    reg.get<factory::SensorComponent>(s_e).set_blocked(true);

    auto item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {0.f, 0.f, 0.f};
    pose.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(item);

    factory::sensor::scan(scene);

    // Should remain blocked — scan only writes to laser sensors.
    REQUIRE(reg.get<factory::SensorComponent>(s_e).blocked());
}

TEST(sensor_scan_triggers_when_body_crosses_laser) {
    // An item whose body crosses the laser line — even with its centre
    // already past the line — keeps the laser triggered. The sensor has
    // no extents; the item's bounding box decides.
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e    = scene.add_laser_sensor(port_e);

    auto& reg  = scene.registry();
    auto proto = scene.add_prototype(400, 400, 400, 0u);
    auto item  = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {150.f, 0.f, 0.f};   // body in x: [-50, 350], crosses 0
    pose.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(item).set_prototype(proto);

    factory::sensor::scan(scene);
    REQUIRE(reg.get<factory::SensorComponent>(s_e).blocked());
}

TEST(sensor_scan_clear_when_body_past_laser) {
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e    = scene.add_laser_sensor(port_e);

    auto& reg  = scene.registry();
    auto proto = scene.add_prototype(400, 400, 400, 0u);
    auto item  = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {300.f, 0.f, 0.f};   // body in x: [100, 500] — laser at 0
    pose.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(item).set_prototype(proto);

    factory::sensor::scan(scene);
    REQUIRE(!reg.get<factory::SensorComponent>(s_e).blocked());
}

TEST(sensor_scan_uses_offset) {
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e    = scene.add_laser_sensor(port_e, {500.f, 0.f, 0.f});

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {500.f, 0.f, 0.f};   // exactly at laser position
    pose.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(item);

    factory::sensor::scan(scene);
    REQUIRE(reg.get<factory::SensorComponent>(s_e).blocked());
}

// ── transport::step (belt motion) ────────────────────────────────────────────

TEST(belt_advances_running_clear_item) {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 2000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry", {0.f,    0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_e  = scene.add_port("exit",  {2000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    scene.connect_belt(belt_e, entry_e, exit_e);

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {0.f, 0.f, 0.f};
    pose.parent   = scene.root_entity();
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_e);
    reg.emplace<factory::SpawnedItemComponent>(item);

    factory::transport::step(scene, 1.0f);  // 200 mm/s × 1s = 200 mm

    REQUIRE_NEAR(reg.get<factory::PoseComponent>(item).position.x, 200.f, 1.f);
}

TEST(belt_frozen_when_not_running) {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 2000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry", {0.f,    0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_e  = scene.add_port("exit",  {2000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    scene.connect_belt(belt_e, entry_e, exit_e);
    scene.registry().get<factory::TransportComponent>(belt_e).set_running(false);

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {0.f, 0.f, 0.f};
    pose.parent   = scene.root_entity();
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_e);
    reg.emplace<factory::SpawnedItemComponent>(item);

    factory::transport::step(scene, 1.0f);

    REQUIRE_NEAR(reg.get<factory::PoseComponent>(item).position.x, 0.f, 1e-5f);
}

TEST(belt_frozen_when_exit_port_blocked) {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 2000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry", {0.f,    0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_e  = scene.add_port("exit",  {2000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    scene.connect_belt(belt_e, entry_e, exit_e);

    // Exit port is automatically a gate port. Block it via a virtual sensor.
    auto sensor_e = scene.add_virtual_sensor(exit_e);
    scene.registry().get<factory::SensorComponent>(sensor_e).set_blocked(true);

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {0.f, 0.f, 0.f};
    pose.parent   = scene.root_entity();
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_e);
    reg.emplace<factory::SpawnedItemComponent>(item);

    factory::transport::step(scene, 1.0f);

    REQUIRE_NEAR(reg.get<factory::PoseComponent>(item).position.x, 0.f, 1e-5f);
}

TEST(tap_in_gate_ports_freezes_belt) {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 2000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry", {0.f,    0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_e  = scene.add_port("exit",  {2000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    scene.connect_belt(belt_e, entry_e, exit_e);

    // Add a tap port mid-belt and add it to gate_ports.
    auto tap_e = scene.add_tap_port(belt_e, "tap", 1000);
    scene.make_gate_port(belt_e, tap_e);

    auto sensor_e = scene.add_virtual_sensor(tap_e);
    scene.registry().get<factory::SensorComponent>(sensor_e).set_blocked(true);

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {0.f, 0.f, 0.f};
    pose.parent   = scene.root_entity();
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_e);
    reg.emplace<factory::SpawnedItemComponent>(item);

    factory::transport::step(scene, 1.0f);

    REQUIRE_NEAR(reg.get<factory::PoseComponent>(item).position.x, 0.f, 1e-5f);
}

TEST(tap_not_in_gate_ports_does_not_freeze_belt) {
    // Tap port present but not added to gate_ports — belt advances normally.
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 2000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry", {0.f,    0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_e  = scene.add_port("exit",  {2000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    scene.connect_belt(belt_e, entry_e, exit_e);

    auto tap_e    = scene.add_tap_port(belt_e, "tap", 1000);
    auto sensor_e = scene.add_virtual_sensor(tap_e);
    scene.registry().get<factory::SensorComponent>(sensor_e).set_blocked(true);

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {0.f, 0.f, 0.f};
    pose.parent   = scene.root_entity();
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_e);
    reg.emplace<factory::SpawnedItemComponent>(item);

    factory::transport::step(scene, 1.0f);

    REQUIRE_NEAR(reg.get<factory::PoseComponent>(item).position.x, 200.f, 1.f);
}

TEST(belt_handover_to_next_belt) {
    factory::FactoryScene scene;
    auto belt_a  = scene.add_belt(300, 1000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_a = scene.add_port("entry_a", {0.f,    0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_a  = scene.add_port("exit_a",  {1000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});

    auto belt_b  = scene.add_belt(300, 2000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_b = scene.add_port("entry_b", {1000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_b  = scene.add_port("exit_b",  {3000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});

    scene.connect_belt(belt_a, entry_a, exit_a);
    scene.connect_belt(belt_b, entry_b, exit_b);
    scene.set_port_transport(exit_a, belt_b);   // A's exit hands to B

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {0.f, 0.f, 0.f};
    pose.parent   = scene.root_entity();
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_a);
    reg.emplace<factory::SpawnedItemComponent>(item);

    // 5 × 1s × 200 mm/s = 1000 mm → handover at end of A
    for (int i = 0; i < 5; ++i)
        factory::transport::step(scene, 1.0f);

    REQUIRE_EQ(scene.items_on(belt_a), 0);
    REQUIRE_EQ(scene.items_on(belt_b), 1);
}

// ── transport::step (picker state machine) ───────────────────────────────────

TEST(picker_idle_does_not_move) {
    factory::FactoryScene scene;
    auto picker_e = scene.add_picker({100.f, 200.f, 300.f}, 500.f);

    factory::transport::step(scene, 1.0f);

    const auto& pose = scene.registry().get<factory::PoseComponent>(picker_e);
    REQUIRE_NEAR(pose.position.x, 100.f, 1e-5f);
    REQUIRE_NEAR(pose.position.y, 200.f, 1e-5f);
    REQUIRE_NEAR(pose.position.z, 300.f, 1e-5f);
}

TEST(picker_advances_toward_pickup_target) {
    factory::FactoryScene scene;
    auto picker_e = scene.add_picker({0.f, 0.f, 0.f}, 100.f);  // 100 mm/s
    auto& reg     = scene.registry();
    auto& pt      = reg.get<factory::PickerTransportComponent>(picker_e);
    pt.set_pickup_target({1000.f, 0.f, 0.f});
    pt.set_state(factory::PickerState::MovingToBox);

    factory::transport::step(scene, 1.0f);   // 100 mm advance

    const auto& pose = reg.get<factory::PoseComponent>(picker_e);
    REQUIRE_NEAR(pose.position.x, 100.f, 1.f);
    REQUIRE(pt.state() == factory::PickerState::MovingToBox);  // not yet arrived
}

TEST(picker_clamps_to_pickup_target_and_transitions) {
    factory::FactoryScene scene;
    auto& reg     = scene.registry();
    auto picker_e = scene.add_picker({0.f, 0.f, 0.f}, 1000.f);

    // Place a box at the pickup target. Box must have ItemOnTransportComponent
    // that the picker will reassign onto itself.
    auto box_e = reg.create();
    auto& bpose = reg.emplace<factory::PoseComponent>(box_e);
    bpose.position = {500.f, 0.f, 0.f};
    bpose.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(box_e);
    auto dummy_belt = reg.create();
    reg.emplace<factory::TransportComponent>(dummy_belt);
    reg.emplace<factory::ItemOnTransportComponent>(box_e).set_transport(dummy_belt);

    auto& pt = reg.get<factory::PickerTransportComponent>(picker_e);
    pt.set_pickup_target({500.f, 0.f, 0.f});
    pt.set_drop_target({2000.f, 0.f, 0.f});
    pt.set_current_box(box_e);
    pt.set_state(factory::PickerState::MovingToBox);

    factory::transport::step(scene, 1.0f);  // 1000 mm/s × 1s — overshoots

    const auto& pose = reg.get<factory::PoseComponent>(picker_e);
    REQUIRE_NEAR(pose.position.x, 500.f, 1e-3f);
    REQUIRE(pt.state() == factory::PickerState::Carrying);
    // Box is now riding the picker.
    REQUIRE(reg.get<factory::ItemOnTransportComponent>(box_e).transport() == picker_e);
}

TEST(picker_carrying_advances_toward_drop_target) {
    factory::FactoryScene scene;
    auto& reg     = scene.registry();
    auto picker_e = scene.add_picker({500.f, 0.f, 0.f}, 100.f);
    auto& pt      = reg.get<factory::PickerTransportComponent>(picker_e);
    pt.set_drop_target({2000.f, 0.f, 0.f});
    pt.set_state(factory::PickerState::Carrying);

    factory::transport::step(scene, 1.0f);

    const auto& pose = reg.get<factory::PoseComponent>(picker_e);
    REQUIRE_NEAR(pose.position.x, 600.f, 1.f);  // 100 mm advance toward drop
    REQUIRE(pt.state() == factory::PickerState::Carrying);
}

TEST(picker_at_drop_target_releases_to_container) {
    factory::FactoryScene scene;
    auto& reg     = scene.registry();
    auto picker_e = scene.add_picker({0.f, 0.f, 0.f}, 10000.f);

    // Container at world (1000, 0, 0), parented to scene root.
    auto container_e = reg.create();
    auto& cpose = reg.emplace<factory::PoseComponent>(container_e);
    cpose.position = {1000.f, 0.f, 0.f};
    cpose.parent   = scene.root_entity();

    // Box on picker.
    auto box_e = reg.create();
    auto& bpose = reg.emplace<factory::PoseComponent>(box_e);
    bpose.position = {0.f, 0.f, 0.f};
    bpose.parent   = picker_e;
    reg.emplace<factory::SpawnedItemComponent>(box_e);
    reg.emplace<factory::ItemOnTransportComponent>(box_e).set_transport(picker_e);

    auto& pt = reg.get<factory::PickerTransportComponent>(picker_e);
    pt.set_drop_target({1000.f, 0.f, 0.f});
    pt.set_drop_container(container_e);
    pt.set_current_box(box_e);
    pt.set_state(factory::PickerState::Carrying);

    factory::transport::step(scene, 1.0f);

    REQUIRE(pt.state() == factory::PickerState::Returning);
    // Box is no longer on a transport.
    REQUIRE(!reg.any_of<factory::ItemOnTransportComponent>(box_e));
    // Box is parented to container.
    REQUIRE(reg.get<factory::PoseComponent>(box_e).parent == container_e);
}

TEST(picker_returning_then_idle_at_home) {
    factory::FactoryScene scene;
    auto& reg     = scene.registry();
    auto picker_e = scene.add_picker({0.f, 0.f, 0.f}, 10000.f);
    // Move picker away from home, then set Returning.
    reg.get<factory::PoseComponent>(picker_e).position = {1000.f, 0.f, 0.f};
    auto& pt = reg.get<factory::PickerTransportComponent>(picker_e);
    pt.set_state(factory::PickerState::Returning);
    pt.set_current_box(picker_e);  // dummy non-null to verify clearing

    factory::transport::step(scene, 1.0f);  // overshoots

    REQUIRE(pt.state() == factory::PickerState::Idle);
    REQUIRE(pt.current_box() == entt::null);
    const auto& pose = reg.get<factory::PoseComponent>(picker_e);
    REQUIRE_NEAR(pose.position.x, 0.f, 1e-3f);
}

// ── lifecycle::step (source / sink) ──────────────────────────────────────────

static factory::FactoryScene make_source_belt_scene(float rate_per_hour = 3600.f) {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 2000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry", {0.f,    0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_e  = scene.add_port("exit",  {2000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    scene.connect_belt(belt_e, entry_e, exit_e);
    scene.set_port_transport(entry_e, belt_e);

    auto proto_e = scene.add_prototype(300, 300, 300, 0x8B4513u);
    scene.add_source(rate_per_hour, proto_e, entry_e);
    return scene;
}

TEST(source_spawns_when_port_clear) {
    auto   scene  = make_source_belt_scene();
    auto   events = factory::lifecycle::step(scene, 1.0f);  // debt = 1.0
    REQUIRE_EQ(static_cast<int>(events.spawned.size()), 1);
}

TEST(source_does_not_spawn_when_port_blocked) {
    auto   scene  = make_source_belt_scene();
    auto&  reg    = scene.registry();

    // Block the source's out_port (= belt entry).
    factory::SourceComponent* src = nullptr;
    entt::entity              entry_port_e = entt::null;
    reg.view<factory::SourceComponent>().each(
        [&](factory::SourceComponent& s) { src = &s; entry_port_e = s.out_port(); });
    REQUIRE(entry_port_e != entt::null);

    auto sensor_e = scene.add_virtual_sensor(entry_port_e);
    reg.get<factory::SensorComponent>(sensor_e).set_blocked(true);

    auto events = factory::lifecycle::step(scene, 1.0f);
    REQUIRE(events.spawned.empty());
}

TEST(spawned_item_has_correct_prototype) {
    auto   scene  = make_source_belt_scene();
    auto&  reg    = scene.registry();
    auto   events = factory::lifecycle::step(scene, 1.0f);
    REQUIRE(!events.spawned.empty());
    auto item    = events.spawned[0].entity;
    auto proto   = events.spawned[0].prototype;
    REQUIRE(reg.valid(item));
    REQUIRE(reg.get<factory::SpawnedItemComponent>(item).prototype() == proto);
    // Spawned items must ride a transport.
    REQUIRE(reg.any_of<factory::ItemOnTransportComponent>(item));
}

TEST(no_spawn_when_debt_below_one) {
    auto  scene  = make_source_belt_scene(360.f);            // 0.1 / s
    auto  events = factory::lifecycle::step(scene, 1.0f);    // debt = 0.1
    REQUIRE(events.spawned.empty());
}

TEST(source_shifts_spawn_for_leading_edge_at_port) {
    // A spawned item is placed with its leading edge at the port, not its
    // centre — i.e. its centre is shifted back along the belt direction by
    // half its motion-axis extent. Without this shift, the freshly spawned
    // item would clip into the previous one (whose trailing edge has only
    // just cleared the source's laser).
    auto  scene  = make_source_belt_scene();          // 300×300×300 proto, +x belt
    auto& reg    = scene.registry();
    auto  events = factory::lifecycle::step(scene, 1.0f);
    REQUIRE(!events.spawned.empty());
    auto item = events.spawned[0].entity;
    // Port at (0, 0, 0), belt direction +x. half_length = 150 mm.
    REQUIRE_NEAR(reg.get<factory::PoseComponent>(item).position.x, -150.f, 1e-3f);
}

TEST(source_caps_at_one_spawn_per_tick) {
    // Even when accumulated debt is > 1 (from a long dt or a long
    // belt-frozen interval), a source spawns at most one item per tick.
    // Spawning more than one at the same world position would stack items
    // on top of each other.
    auto scene  = make_source_belt_scene();                  // 1 / s
    auto events = factory::lifecycle::step(scene, 5.0f);     // debt = 5
    REQUIRE_EQ(static_cast<int>(events.spawned.size()), 1);
}

TEST(sink_despawns_item_at_in_port) {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 1000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry", {0.f,    0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_e  = scene.add_port("exit",  {1000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    scene.connect_belt(belt_e, entry_e, exit_e);
    scene.add_sink(exit_e);
    // Sink must be able to detect arriving items via a laser on its in_port.
    scene.add_laser_sensor(exit_e);

    auto& reg = scene.registry();
    auto  proto = scene.add_prototype(300, 300, 300, 0xFF0000u);
    auto  item  = reg.create();
    auto& pose  = reg.emplace<factory::PoseComponent>(item);
    pose.position = {1000.f, 0.f, 0.f};
    pose.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(item).set_prototype(proto);
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_e);

    factory::sensor::scan(scene);
    auto events = factory::lifecycle::step(scene, 1.0f);

    REQUIRE_EQ(static_cast<int>(events.despawned.size()), 1);
    int total = 0;
    reg.view<factory::SinkComponent>().each(
        [&](const factory::SinkComponent& sk) { total += sk.received(); });
    REQUIRE_EQ(total, 1);
}

// ── transport::step (multi-item-on-one-belt safety) ─────────────────────────

TEST(belt_does_not_double_clamp_at_terminal_exit) {
    // Two items on the same belt would both overshoot length_mm in one tick.
    // The leader (higher start-of-tick t) should clamp to the exit position
    // and the trailer should NOT advance — its iteration sees the belt
    // already "frozen" for this tick.
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 1000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry", {0.f,    0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_e  = scene.add_port("exit",  {1000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    scene.connect_belt(belt_e, entry_e, exit_e);

    auto& reg = scene.registry();

    auto leader = reg.create();
    auto& lp    = reg.emplace<factory::PoseComponent>(leader);
    lp.position = {950.f, 0.f, 0.f};
    lp.parent   = scene.root_entity();
    reg.emplace<factory::ItemOnTransportComponent>(leader).set_transport(belt_e);
    reg.emplace<factory::SpawnedItemComponent>(leader);

    auto trailer = reg.create();
    auto& tp     = reg.emplace<factory::PoseComponent>(trailer);
    tp.position  = {800.f, 0.f, 0.f};
    tp.parent    = scene.root_entity();
    reg.emplace<factory::ItemOnTransportComponent>(trailer).set_transport(belt_e);
    reg.emplace<factory::SpawnedItemComponent>(trailer);

    // dt = 2s × 200 mm/s = 400 mm advance — both would overshoot.
    factory::transport::step(scene, 2.0f);

    REQUIRE_NEAR(reg.get<factory::PoseComponent>(leader).position.x,  1000.f, 1.f);
    REQUIRE_NEAR(reg.get<factory::PoseComponent>(trailer).position.x,  800.f, 1.f);
}

TEST(belt_does_not_double_handover) {
    // Same shape but with a downstream belt — only the leader transfers,
    // the trailer waits for the next tick.
    factory::FactoryScene scene;

    auto belt_a  = scene.add_belt(300, 1000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_a = scene.add_port("entry_a", {0.f,    0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_a  = scene.add_port("exit_a",  {1000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    auto belt_b  = scene.add_belt(300, 2000, 0, 200.f, {1.f, 0.f, 0.f});
    auto entry_b = scene.add_port("entry_b", {1000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    auto exit_b  = scene.add_port("exit_b",  {3000.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    scene.connect_belt(belt_a, entry_a, exit_a);
    scene.connect_belt(belt_b, entry_b, exit_b);
    scene.set_port_transport(exit_a, belt_b);

    auto& reg = scene.registry();

    auto leader = reg.create();
    auto& lp    = reg.emplace<factory::PoseComponent>(leader);
    lp.position = {950.f, 0.f, 0.f};
    lp.parent   = scene.root_entity();
    reg.emplace<factory::ItemOnTransportComponent>(leader).set_transport(belt_a);
    reg.emplace<factory::SpawnedItemComponent>(leader);

    auto trailer = reg.create();
    auto& tp     = reg.emplace<factory::PoseComponent>(trailer);
    tp.position  = {800.f, 0.f, 0.f};
    tp.parent    = scene.root_entity();
    reg.emplace<factory::ItemOnTransportComponent>(trailer).set_transport(belt_a);
    reg.emplace<factory::SpawnedItemComponent>(trailer);

    factory::transport::step(scene, 2.0f);

    REQUIRE_EQ(scene.items_on(belt_a), 1);   // trailer still on A
    REQUIRE_EQ(scene.items_on(belt_b), 1);   // leader handed off
    REQUIRE_NEAR(reg.get<factory::PoseComponent>(trailer).position.x, 800.f, 1.f);
}

// ── MagicTransportComponent ─────────────────────────────────────────────────

TEST(magic_default_state_idle) {
    factory::MagicTransportComponent mt;
    REQUIRE(mt.state()       == factory::PickerState::Idle);
    REQUIRE(mt.current_box() == entt::null);
    REQUIRE_NEAR(mt.elapsed_s(), 0.f, 1e-5f);
}

TEST(magic_setters_round_trip) {
    factory::MagicTransportComponent mt;
    mt.set_home_pose({100.f, 200.f, 300.f});
    mt.set_pickup_target({400.f, 0.f, 800.f});
    mt.set_drop_target({0.f, 0.f, 145.f});
    mt.set_leg_duration_s(2.5f);
    mt.set_elapsed_s(0.3f);
    mt.set_leg_origin({100.f, 200.f, 300.f});
    mt.set_state(factory::PickerState::Carrying);
    REQUIRE_NEAR(mt.home_pose().x,     100.f, 1e-5f);
    REQUIRE_NEAR(mt.pickup_target().z, 800.f, 1e-5f);
    REQUIRE_NEAR(mt.drop_target().z,   145.f, 1e-5f);
    REQUIRE_NEAR(mt.leg_duration_s(),  2.5f,  1e-5f);
    REQUIRE_NEAR(mt.elapsed_s(),       0.3f,  1e-5f);
    REQUIRE(mt.state() == factory::PickerState::Carrying);
}

TEST(magic_position_lands_on_endpoints) {
    // Whatever the swirl does, the envelope must be 0 at t=0 and t=1 so the
    // path lands exactly on origin and target.
    factory::Vec3 origin{0.f, 0.f, 0.f};
    factory::Vec3 target{1000.f, 500.f, 200.f};
    auto p0 = factory::transport::magic::position(origin, target, 0.f);
    auto p1 = factory::transport::magic::position(origin, target, 1.f);
    REQUIRE_NEAR(p0.x, origin.x, 1e-3f);
    REQUIRE_NEAR(p0.y, origin.y, 1e-3f);
    REQUIRE_NEAR(p0.z, origin.z, 1e-3f);
    REQUIRE_NEAR(p1.x, target.x, 1e-3f);
    REQUIRE_NEAR(p1.y, target.y, 1e-3f);
    REQUIRE_NEAR(p1.z, target.z, 1e-3f);
}

TEST(magic_position_handles_zero_length_leg) {
    // Degenerate case — origin == target. Should not divide by zero.
    factory::Vec3 p{42.f, -7.f, 13.f};
    auto out = factory::transport::magic::position(p, p, 0.5f);
    REQUIRE_NEAR(out.x, p.x, 1e-5f);
    REQUIRE_NEAR(out.y, p.y, 1e-5f);
    REQUIRE_NEAR(out.z, p.z, 1e-5f);
}

TEST(magic_idle_does_not_advance) {
    factory::FactoryScene scene;
    auto magic_e = scene.add_magic_transport({100.f, 200.f, 300.f}, 1.5f);
    factory::transport::step(scene, 0.5f);
    const auto& pose = scene.registry().get<factory::PoseComponent>(magic_e);
    REQUIRE_NEAR(pose.position.x, 100.f, 1e-5f);
    REQUIRE_NEAR(pose.position.y, 200.f, 1e-5f);
    REQUIRE_NEAR(pose.position.z, 300.f, 1e-5f);
}

TEST(magic_completes_leg_and_grabs_box) {
    factory::FactoryScene scene;
    auto& reg = scene.registry();

    auto magic_e = scene.add_magic_transport({0.f, 0.f, 0.f}, 1.0f);

    // Box at the pickup target, riding a dummy belt.
    auto box_e = reg.create();
    auto& bp   = reg.emplace<factory::PoseComponent>(box_e);
    bp.position = {500.f, 0.f, 0.f};
    bp.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(box_e);
    auto dummy = reg.create();
    reg.emplace<factory::TransportComponent>(dummy);
    reg.emplace<factory::ItemOnTransportComponent>(box_e).set_transport(dummy);

    auto& mt = reg.get<factory::MagicTransportComponent>(magic_e);
    mt.set_pickup_target({500.f, 0.f, 0.f});
    mt.set_drop_target({1000.f, 0.f, 0.f});
    mt.set_current_box(box_e);
    mt.set_state(factory::PickerState::MovingToBox);
    mt.set_leg_origin({0.f, 0.f, 0.f});
    mt.set_elapsed_s(0.f);

    // Step past the leg duration — magic should clamp, transition to
    // Carrying, and grab the box.
    factory::transport::step(scene, 1.5f);

    REQUIRE(mt.state() == factory::PickerState::Carrying);
    REQUIRE(reg.get<factory::ItemOnTransportComponent>(box_e).transport() == magic_e);
    REQUIRE(reg.get<factory::PoseComponent>(box_e).parent == magic_e);
    // After Clamp, leg_origin reset to the pickup target for the next leg.
    REQUIRE_NEAR(mt.leg_origin().x, 500.f, 1e-3f);
}

TEST(magic_drop_releases_box_to_container) {
    factory::FactoryScene scene;
    auto& reg = scene.registry();

    auto magic_e = scene.add_magic_transport({0.f, 0.f, 0.f}, 1.0f);

    // Container at world (1000, 0, 0).
    auto container = reg.create();
    auto& cp       = reg.emplace<factory::PoseComponent>(container);
    cp.position    = {1000.f, 0.f, 0.f};
    cp.parent      = scene.root_entity();

    // Box riding the magic transport.
    auto box_e = reg.create();
    auto& bp   = reg.emplace<factory::PoseComponent>(box_e);
    bp.position = {0.f, 0.f, 0.f};
    bp.parent   = magic_e;
    reg.emplace<factory::SpawnedItemComponent>(box_e);
    reg.emplace<factory::ItemOnTransportComponent>(box_e).set_transport(magic_e);

    auto& mt = reg.get<factory::MagicTransportComponent>(magic_e);
    mt.set_drop_target({1000.f, 0.f, 0.f});
    mt.set_drop_container(container);
    mt.set_current_box(box_e);
    mt.set_state(factory::PickerState::Carrying);
    mt.set_leg_origin({0.f, 0.f, 0.f});
    mt.set_elapsed_s(0.f);

    factory::transport::step(scene, 1.5f);

    REQUIRE(mt.state() == factory::PickerState::Returning);
    REQUIRE(!reg.any_of<factory::ItemOnTransportComponent>(box_e));
    REQUIRE(reg.get<factory::PoseComponent>(box_e).parent == container);
}

TEST(magic_returns_to_idle_at_home) {
    factory::FactoryScene scene;
    auto& reg = scene.registry();

    auto magic_e = scene.add_magic_transport({0.f, 0.f, 0.f}, 1.0f);
    auto& mpose  = reg.get<factory::PoseComponent>(magic_e);
    mpose.position = {1000.f, 0.f, 0.f};

    auto& mt = reg.get<factory::MagicTransportComponent>(magic_e);
    mt.set_state(factory::PickerState::Returning);
    mt.set_leg_origin({1000.f, 0.f, 0.f});
    mt.set_current_box(magic_e);     // dummy non-null to verify clearing
    mt.set_elapsed_s(0.f);

    factory::transport::step(scene, 1.5f);

    REQUIRE(mt.state()       == factory::PickerState::Idle);
    REQUIRE(mt.current_box() == entt::null);
    REQUIRE_NEAR(mpose.position.x, 0.f, 1e-3f);
}

// ── station::step ───────────────────────────────────────────────────────────

// Helper: build a minimal palletizer station with one picker, a pallet
// arrival port (with both physical and virtual sensors), and a box arrival
// port (also with both sensor flavours). Mirrors the demo's wiring without
// the threepp pieces.
struct PalletizerFixture {
    factory::FactoryScene scene;
    entt::entity          station_e;
    entt::entity          pallet_arrival_port;
    entt::entity          pallet_tap_virt;
    entt::entity          box_arrival_port;
    entt::entity          box_arrival_virt;
    entt::entity          picker;
    entt::entity          box_proto;
    entt::entity          pallet_proto;
};

static PalletizerFixture make_palletizer_fixture() {
    PalletizerFixture pf;
    auto& scene = pf.scene;
    auto& reg   = scene.registry();

    pf.pallet_arrival_port = scene.add_port("pal_arr", {0.f, 0.f, 0.f});
    scene.add_laser_sensor(pf.pallet_arrival_port);
    pf.pallet_tap_virt = scene.add_virtual_sensor(pf.pallet_arrival_port);

    // Box arrival port is at belt height, separate from the pallet port's
    // ground plane — otherwise a 1200 × 800 × 145 pallet's bounding box
    // would also enclose the box port's laser and trigger spurious
    // dispatches. (This mirrors the demo's geometry.)
    pf.box_arrival_port  = scene.add_port("box_arr", {500.f, 0.f, 800.f});
    scene.add_laser_sensor(pf.box_arrival_port);
    pf.box_arrival_virt  = scene.add_virtual_sensor(pf.box_arrival_port);

    pf.box_proto    = scene.add_prototype(250, 250, 200, 0u);
    pf.pallet_proto = scene.add_prototype(1200, 800, 145, 0u);

    pf.picker = scene.add_picker(factory::Vec3{-1000.f, 0.f, 1500.f}, 1000.f);

    pf.station_e = reg.create();
    auto& sc = reg.emplace<factory::StationComponent>(pf.station_e);
    sc.set_arrival_port(pf.box_arrival_port);
    sc.set_arrival_virtual_sensor(pf.box_arrival_virt);
    sc.add_picker(pf.picker);

    auto& palc = reg.emplace<factory::PalletizeComponent>(pf.station_e);
    palc.set_pallet_arrival_port(pf.pallet_arrival_port);
    palc.set_pallet_tap_virtual_sensor(pf.pallet_tap_virt);
    palc.set_pattern(std::make_shared<factory::GridPattern>());
    palc.set_pallet_dimensions(1200, 800, 145, 1500);

    return pf;
}

// Place an item at a port's world position. Returns the item entity.
static entt::entity place_item_at_port(factory::FactoryScene& scene,
                                       entt::entity port,
                                       entt::entity proto)
{
    auto& reg = scene.registry();
    auto e    = reg.create();
    auto& p   = reg.emplace<factory::PoseComponent>(e);
    auto pw   = factory::world_transform(port, reg);
    p.position = factory::Vec3(pw[3]);
    p.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(e).set_prototype(proto);
    return e;
}

TEST(station_claims_pallet_at_arrival_port) {
    auto pf = make_palletizer_fixture();
    auto& reg = pf.scene.registry();
    auto pallet = place_item_at_port(pf.scene, pf.pallet_arrival_port, pf.pallet_proto);

    factory::sensor::scan(pf.scene);
    factory::station::step(pf.scene, 0.016f);

    auto& palc = reg.get<factory::PalletizeComponent>(pf.station_e);
    REQUIRE(palc.current_pallet() == pallet);
    REQUIRE(reg.any_of<factory::PalletComponent>(pallet));
    // The station should also have written the pallet-tap virtual sensor.
    REQUIRE(reg.get<factory::SensorComponent>(pf.pallet_tap_virt).blocked());
}

TEST(station_skips_already_claimed_pallet) {
    // A pallet that already has a PalletComponent (i.e. previously claimed
    // and released) must NOT be re-claimed when it loiters in the detect
    // sensor's volume on its way out.
    auto pf = make_palletizer_fixture();
    auto& reg = pf.scene.registry();
    auto pallet = place_item_at_port(pf.scene, pf.pallet_arrival_port, pf.pallet_proto);
    reg.emplace<factory::PalletComponent>(pallet);   // mark as already-seen

    factory::sensor::scan(pf.scene);
    factory::station::step(pf.scene, 0.016f);

    auto& palc = reg.get<factory::PalletizeComponent>(pf.station_e);
    REQUIRE(palc.current_pallet() == entt::null);
    REQUIRE(!reg.get<factory::SensorComponent>(pf.pallet_tap_virt).blocked());
}

TEST(station_dispatches_idle_picker_when_box_arrives) {
    auto pf = make_palletizer_fixture();
    auto& reg = pf.scene.registry();

    // Pre-claim a pallet to skip the acquisition path.
    auto pallet = place_item_at_port(pf.scene, pf.pallet_arrival_port, pf.pallet_proto);
    auto& palc = reg.get<factory::PalletizeComponent>(pf.station_e);
    palc.set_current_pallet(pallet);
    auto& palletc = reg.emplace<factory::PalletComponent>(pallet);
    palletc.set_length_mm(1200);
    palletc.set_width_mm(800);
    palletc.set_height_mm(145);
    palletc.set_max_stack_height_mm(1500);

    auto box = place_item_at_port(pf.scene, pf.box_arrival_port, pf.box_proto);

    factory::sensor::scan(pf.scene);
    factory::station::step(pf.scene, 0.016f);

    const auto& pt = reg.get<factory::PickerTransportComponent>(pf.picker);
    REQUIRE(pt.state()           == factory::PickerState::MovingToBox);
    REQUIRE(pt.current_box()     == box);
    REQUIRE(pt.drop_container()  == pallet);
}

TEST(station_does_not_dispatch_without_pallet) {
    auto pf = make_palletizer_fixture();
    auto& reg = pf.scene.registry();

    place_item_at_port(pf.scene, pf.box_arrival_port, pf.box_proto);

    factory::sensor::scan(pf.scene);
    factory::station::step(pf.scene, 0.016f);

    const auto& pt = reg.get<factory::PickerTransportComponent>(pf.picker);
    REQUIRE(pt.state() == factory::PickerState::Idle);
}

TEST(station_does_not_double_claim_box) {
    // With two pickers and one box: the first station::step assigns the
    // box to picker1; the next step must NOT also assign it to picker2.
    auto pf = make_palletizer_fixture();
    auto& reg = pf.scene.registry();

    auto pallet = place_item_at_port(pf.scene, pf.pallet_arrival_port, pf.pallet_proto);
    auto& palc = reg.get<factory::PalletizeComponent>(pf.station_e);
    palc.set_current_pallet(pallet);
    auto& palletc = reg.emplace<factory::PalletComponent>(pallet);
    palletc.set_length_mm(1200);
    palletc.set_width_mm(800);
    palletc.set_height_mm(145);
    palletc.set_max_stack_height_mm(1500);

    auto picker2 = pf.scene.add_picker(factory::Vec3{1000.f, 0.f, 1500.f}, 1000.f);
    reg.get<factory::StationComponent>(pf.station_e).add_picker(picker2);

    place_item_at_port(pf.scene, pf.box_arrival_port, pf.box_proto);

    factory::sensor::scan(pf.scene);
    factory::station::step(pf.scene, 0.016f);
    factory::sensor::scan(pf.scene);
    factory::station::step(pf.scene, 0.016f);

    const auto& pt2 = reg.get<factory::PickerTransportComponent>(picker2);
    REQUIRE(pt2.state() == factory::PickerState::Idle);
}

TEST(station_arrival_virtual_blocked_when_no_pallet) {
    auto pf = make_palletizer_fixture();
    auto& reg = pf.scene.registry();

    factory::station::step(pf.scene, 0.016f);

    REQUIRE(reg.get<factory::SensorComponent>(pf.box_arrival_virt).blocked());
}

TEST(station_arrival_virtual_clear_with_pallet_and_idle_picker) {
    auto pf = make_palletizer_fixture();
    auto& reg = pf.scene.registry();
    auto pallet = place_item_at_port(pf.scene, pf.pallet_arrival_port, pf.pallet_proto);
    auto& palc = reg.get<factory::PalletizeComponent>(pf.station_e);
    palc.set_current_pallet(pallet);
    reg.emplace<factory::PalletComponent>(pallet);

    factory::station::step(pf.scene, 0.016f);

    REQUIRE(!reg.get<factory::SensorComponent>(pf.box_arrival_virt).blocked());
}

TEST(station_releases_full_pallet_when_all_pickers_idle) {
    auto pf = make_palletizer_fixture();
    auto& reg = pf.scene.registry();

    auto pallet = place_item_at_port(pf.scene, pf.pallet_arrival_port, pf.pallet_proto);
    auto& palc = reg.get<factory::PalletizeComponent>(pf.station_e);
    palc.set_current_pallet(pallet);
    auto& palletc = reg.emplace<factory::PalletComponent>(pallet);
    palletc.set_length_mm(1200);
    palletc.set_width_mm(800);
    palletc.set_height_mm(145);
    palletc.set_max_stack_height_mm(1500);

    // Fill the pallet with 12 boxes so GridPattern::next_pose returns nullopt.
    for (int i = 0; i < 12; ++i) {
        auto child = reg.create();
        reg.emplace<factory::PoseComponent>(child);
        reg.emplace<factory::SpawnedItemComponent>(child).set_prototype(pf.box_proto);
        palletc.add_item(child);
    }

    // Pretend the station already drove the gate sensor.
    reg.get<factory::SensorComponent>(pf.pallet_tap_virt).set_blocked(true);

    factory::station::step(pf.scene, 0.016f);

    REQUIRE(palc.current_pallet() == entt::null);
    REQUIRE(!reg.get<factory::SensorComponent>(pf.pallet_tap_virt).blocked());
}

TEST(station_does_not_release_while_picker_busy) {
    auto pf = make_palletizer_fixture();
    auto& reg = pf.scene.registry();

    auto pallet = place_item_at_port(pf.scene, pf.pallet_arrival_port, pf.pallet_proto);
    auto& palc = reg.get<factory::PalletizeComponent>(pf.station_e);
    palc.set_current_pallet(pallet);
    auto& palletc = reg.emplace<factory::PalletComponent>(pallet);
    palletc.set_length_mm(1200);
    palletc.set_width_mm(800);
    palletc.set_height_mm(145);
    palletc.set_max_stack_height_mm(1500);
    for (int i = 0; i < 12; ++i) {
        auto child = reg.create();
        reg.emplace<factory::PoseComponent>(child);
        reg.emplace<factory::SpawnedItemComponent>(child).set_prototype(pf.box_proto);
        palletc.add_item(child);
    }

    // Picker mid-cycle.
    auto& pt = reg.get<factory::PickerTransportComponent>(pf.picker);
    pt.set_state(factory::PickerState::Carrying);
    reg.get<factory::SensorComponent>(pf.pallet_tap_virt).set_blocked(true);

    factory::station::step(pf.scene, 0.016f);

    REQUIRE(palc.current_pallet() == pallet);
    REQUIRE(reg.get<factory::SensorComponent>(pf.pallet_tap_virt).blocked());
}

// ── Workflow solver ─────────────────────────────────────────────────────────

TEST(workflow_solve_palletizer_wires_everything) {
    // Declarative API: define entities and their relationships, let the
    // solver pick belt geometry, ports, sensors, and a transport.
    factory::FactoryScene scene;
    auto& reg = scene.registry();

    auto pallet_proto = scene.add_prototype(1200, 800, 145, 0u);
    auto box_proto    = scene.add_prototype(250,  250, 200, 0u);

    auto pallet_source = scene.declare_source( 360.f, pallet_proto);
    auto box_source    = scene.declare_source(1800.f, box_proto);
    auto pallet_sink   = scene.declare_sink();
    auto palletizer    = scene.declare_palletizer_station(pallet_proto, box_proto);

    scene.declare_flow(pallet_source, palletizer,  6.f);
    scene.declare_flow(box_source,    palletizer, 30.f);
    scene.declare_flow(palletizer,    pallet_sink, 6.f);

    scene.solve_workflow();

    // Each declared source / sink / station should now have its ports wired.
    REQUIRE(reg.get<factory::SourceComponent>(pallet_source).out_port() != entt::null);
    REQUIRE(reg.get<factory::SourceComponent>(box_source).out_port()    != entt::null);
    REQUIRE(reg.get<factory::SinkComponent>(pallet_sink).in_port()      != entt::null);

    const auto& sc = reg.get<factory::StationComponent>(palletizer);
    REQUIRE(sc.arrival_port()           != entt::null);
    REQUIRE(sc.arrival_virtual_sensor() != entt::null);
    REQUIRE_EQ(static_cast<int>(sc.pickers().size()), 1);

    const auto& palc = reg.get<factory::PalletizeComponent>(palletizer);
    REQUIRE(palc.pallet_arrival_port()       != entt::null);
    REQUIRE(palc.pallet_tap_virtual_sensor() != entt::null);
    REQUIRE(palc.pattern() != nullptr);

    // Two belts: pallet (z=0) and box (z=800).
    int belts = 0;
    reg.view<factory::ConveyorBeltComponent>().each(
        [&](auto, const factory::ConveyorBeltComponent&) { ++belts; });
    REQUIRE_EQ(belts, 2);
}

TEST(declare_flow_stores_items_per_minute) {
    factory::FactoryScene scene;
    auto& reg = scene.registry();
    auto proto = scene.add_prototype(100, 100, 100, 0u);
    auto src   = scene.declare_source(60.f, proto);
    auto sink  = scene.declare_sink();
    scene.declare_flow(src, sink, 0.5f);

    const auto& fs = scene.flows();
    REQUIRE_EQ(static_cast<int>(fs.size()), 1);
    REQUIRE(fs[0].from == src);
    REQUIRE(fs[0].to   == sink);
    REQUIRE_NEAR(fs[0].items_per_minute, 0.5f, 1e-6f);
    // Silence unused warnings for the registry handle.
    (void)reg;
}

TEST(workflow_solve_attaches_cost_to_every_equipment_entity) {
    // The solver must emplace EquipmentCostComponent on every piece of
    // physical equipment it creates: both belts, the picker (magic stand-in),
    // and the palletizer station. Defaults are zero — callers overwrite.
    factory::FactoryScene scene;
    auto& reg = scene.registry();

    auto pallet_proto = scene.add_prototype(1200, 800, 145, 0u);
    auto box_proto    = scene.add_prototype(250,  250, 200, 0u);

    auto pallet_source = scene.declare_source( 360.f, pallet_proto);
    auto box_source    = scene.declare_source(1800.f, box_proto);
    auto pallet_sink   = scene.declare_sink();
    auto palletizer    = scene.declare_palletizer_station(pallet_proto, box_proto);

    scene.declare_flow(pallet_source, palletizer,  6.f);
    scene.declare_flow(box_source,    palletizer, 30.f);
    scene.declare_flow(palletizer,    pallet_sink, 6.f);

    scene.solve_workflow();

    int belt_costs = 0;
    reg.view<factory::ConveyorBeltComponent, factory::EquipmentCostComponent>().each(
        [&](auto, const auto&, const auto&) { ++belt_costs; });
    REQUIRE_EQ(belt_costs, 2);

    int picker_costs = 0;
    reg.view<factory::MagicTransportComponent, factory::EquipmentCostComponent>().each(
        [&](auto, const auto&, const auto&) { ++picker_costs; });
    REQUIRE_EQ(picker_costs, 1);

    REQUIRE(reg.any_of<factory::EquipmentCostComponent>(palletizer));

    // Defaults are zero until overwritten.
    REQUIRE_NEAR(reg.get<factory::EquipmentCostComponent>(palletizer).capex_eur(), 0.f, 1e-5f);
    REQUIRE_NEAR(reg.get<factory::EquipmentCostComponent>(palletizer).power_w(),   0.f, 1e-5f);
}

// ── Throughput model: picker ────────────────────────────────────────────────
//
// Pure-function tests over the analytic formula, then one sim-vs-analytic
// integration test that confirms our cycle-time model matches the sim's
// straight-line motion under idealised constants (no accel-decel, no
// collision pessimism, no grip/release time).

TEST(picker_throughput_zero_distance) {
    // With zero pickup→drop distance, the cycle is purely grip + release,
    // scaled by collision_pessimism. 60 / (1.3 * (0.3 + 0.2)) ≈ 92.3/min.
    factory::throughput::PickerParams p;
    p.max_tcp_speed_mm_s = 1000.f;
    p.max_payload_kg     = 5.f;
    p.grip_time_s        = 0.3f;
    p.release_time_s     = 0.2f;
    // defaults: effective_tcp_factor=0.6, collision_pessimism=1.3
    factory::throughput::PickerGeometry g;
    g.pickup_to_drop_mm  = 0.f;

    const float tpm = factory::throughput::picker(p, g);
    const float expected = 60.f / (1.3f * 0.5f);
    REQUIRE_NEAR(tpm, expected, 1e-3f);
}

TEST(picker_throughput_realistic_kuka_kr6) {
    // KUKA KR6 R700-class: ~1000 mm/s effective max TCP (manufacturer's
    // 25/305/25 pick-place cycle ≈ 0.42 s implies ~1014 mm/s on short moves),
    // 6 kg payload, 600 mm pickup-to-drop (typical belt-to-pallet reach),
    // 0.3 s grip, 0.2 s release, default factors.
    // cycle = 1.3 * (2 * 600 / (1000 * 0.6) + 0.3 + 0.2)
    //       = 1.3 * (2.0 + 0.5) = 3.25 s  →  ~18.5/min
    factory::throughput::PickerParams p;
    p.max_tcp_speed_mm_s = 1000.f;
    p.max_payload_kg     = 5.f;
    p.grip_time_s        = 0.3f;
    p.release_time_s     = 0.2f;
    factory::throughput::PickerGeometry g;
    g.pickup_to_drop_mm  = 600.f;

    const float tpm = factory::throughput::picker(p, g);
    REQUIRE(tpm > 15.f);
    REQUIRE(tpm < 22.f);
}

TEST(picker_throughput_monotonic_in_speed) {
    factory::throughput::PickerParams p;
    p.max_tcp_speed_mm_s = 500.f;
    p.grip_time_s        = 0.3f;
    p.release_time_s     = 0.2f;
    factory::throughput::PickerGeometry g;
    g.pickup_to_drop_mm  = 800.f;

    const float slow = factory::throughput::picker(p, g);
    p.max_tcp_speed_mm_s = 2000.f;
    const float fast = factory::throughput::picker(p, g);
    REQUIRE(fast > slow);
}

TEST(picker_throughput_monotonic_in_distance) {
    factory::throughput::PickerParams p;
    p.max_tcp_speed_mm_s = 1000.f;
    p.grip_time_s        = 0.3f;
    p.release_time_s     = 0.2f;
    factory::throughput::PickerGeometry g;

    g.pickup_to_drop_mm = 300.f;
    const float near = factory::throughput::picker(p, g);
    g.pickup_to_drop_mm = 1500.f;
    const float far  = factory::throughput::picker(p, g);
    REQUIRE(near > far);
}

TEST(picker_throughput_idealised_matches_sim_cycle_time) {
    // Run a single full home→pickup→drop→home cycle in the sim with home at
    // pickup (so home→pickup leg is zero). The sim does not model accel/decel,
    // grip/release wait, or collision-avoiding paths — so we plug in idealised
    // constants (factor=1.0, pessimism=1.0, grip=release=0) and expect the
    // analytic prediction to match the simulated cycle to within a few ticks.

    factory::FactoryScene scene;
    auto& reg = scene.registry();

    const float speed_mm_s = 1000.f;
    const float distance_mm = 1000.f;

    // Picker with home at origin = pickup target.
    auto picker = scene.add_picker({0.f, 0.f, 0.f}, speed_mm_s);
    auto& pt    = reg.get<factory::PickerTransportComponent>(picker);
    pt.set_pickup_target({0.f,         0.f, 0.f});
    pt.set_drop_target  ({distance_mm, 0.f, 0.f});

    // Drop container at the drop position (parented to scene root).
    auto container = reg.create();
    auto& cpose    = reg.emplace<factory::PoseComponent>(container);
    cpose.position = {distance_mm, 0.f, 0.f};
    cpose.parent   = scene.root_entity();
    pt.set_drop_container(container);

    // Box at the pickup, on a dummy belt (so MovingToBox→Carrying reparents).
    auto box   = reg.create();
    auto& bpose = reg.emplace<factory::PoseComponent>(box);
    bpose.position = {0.f, 0.f, 0.f};
    bpose.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(box);
    auto dummy_belt = reg.create();
    reg.emplace<factory::TransportComponent>(dummy_belt);
    reg.emplace<factory::ItemOnTransportComponent>(box).set_transport(dummy_belt);

    pt.set_current_box(box);
    pt.set_state(factory::PickerState::MovingToBox);

    // Drive the picker through the full cycle.
    const float dt     = 0.01f;
    const int   max_it = 1000;
    int         ticks  = 0;
    while (pt.state() != factory::PickerState::Idle && ticks < max_it) {
        factory::transport::step(scene, dt);
        ++ticks;
    }
    REQUIRE(pt.state() == factory::PickerState::Idle);
    const float simulated_cycle_s = ticks * dt;

    // Analytic prediction with idealised constants.
    factory::throughput::PickerParams p;
    p.max_tcp_speed_mm_s    = speed_mm_s;
    p.grip_time_s           = 0.f;
    p.release_time_s        = 0.f;
    p.effective_tcp_factor  = 1.f;
    p.collision_pessimism   = 1.f;
    factory::throughput::PickerGeometry g;
    g.pickup_to_drop_mm     = distance_mm;

    const float analytic_tpm = factory::throughput::picker(p, g);
    const float analytic_cycle_s = 60.f / analytic_tpm;

    // Tolerance: 2 ticks. The sim takes 1 extra tick to recognise overshoot
    // at each transition, and home==pickup adds 1 tick at start.
    REQUIRE_NEAR(simulated_cycle_s, analytic_cycle_s, 3.f * dt);
}

// ── Throughput model: belt ──────────────────────────────────────────────────

TEST(belt_throughput_realistic_packing) {
    // 1000 mm/s belt, 300 mm boxes, 100 mm safety gap. Expected:
    // 60 * 1000 / (300 + 100) = 150 items/min.
    factory::throughput::BeltParams p;
    p.max_speed_mm_s = 1000.f;
    p.width_mm       = 400;
    factory::throughput::BeltGeometry g;
    g.item_length_mm = 300;
    g.item_width_mm  = 300;
    g.safety_gap_mm  = 100;

    REQUIRE_NEAR(factory::throughput::belt(p, g), 150.f, 1e-3f);
}

TEST(belt_throughput_zero_gap) {
    // Back-to-back packing matches what the current sim achieves.
    // 60 * 1000 / 300 = 200 items/min.
    factory::throughput::BeltParams p;
    p.max_speed_mm_s = 1000.f;
    p.width_mm       = 400;
    factory::throughput::BeltGeometry g;
    g.item_length_mm = 300;
    g.item_width_mm  = 300;
    g.safety_gap_mm  = 0;

    REQUIRE_NEAR(factory::throughput::belt(p, g), 200.f, 1e-3f);
}

TEST(belt_throughput_monotonic_in_speed) {
    factory::throughput::BeltParams p;
    p.width_mm = 400;
    factory::throughput::BeltGeometry g;
    g.item_length_mm = 300;
    g.item_width_mm  = 300;
    g.safety_gap_mm  = 100;

    p.max_speed_mm_s = 500.f;
    const float slow = factory::throughput::belt(p, g);
    p.max_speed_mm_s = 2000.f;
    const float fast = factory::throughput::belt(p, g);
    REQUIRE(fast > slow);
}

TEST(belt_throughput_monotonic_in_item_length) {
    factory::throughput::BeltParams p;
    p.max_speed_mm_s = 1000.f;
    p.width_mm       = 600;
    factory::throughput::BeltGeometry g;
    g.item_width_mm  = 300;
    g.safety_gap_mm  = 100;

    g.item_length_mm = 200;
    const float small = factory::throughput::belt(p, g);
    g.item_length_mm = 500;
    const float big   = factory::throughput::belt(p, g);
    REQUIRE(small > big);
}

TEST(belt_throughput_monotonic_in_gap) {
    factory::throughput::BeltParams p;
    p.max_speed_mm_s = 1000.f;
    p.width_mm       = 400;
    factory::throughput::BeltGeometry g;
    g.item_length_mm = 300;
    g.item_width_mm  = 300;

    g.safety_gap_mm = 0;
    const float tight = factory::throughput::belt(p, g);
    g.safety_gap_mm = 300;
    const float loose = factory::throughput::belt(p, g);
    REQUIRE(tight > loose);
}

TEST(belt_throughput_idealised_matches_sim_steady_state) {
    // Saturate a belt source-and-sink with items, measure exit rate after the
    // startup transient (first item traverses the belt before steady state).
    // The sim's only spacing constraint is "trailing edge clears entry laser
    // before next spawn," so the sim's packing equals item_length back-to-
    // back. Analytic with safety_gap = 0 should match.

    factory::FactoryScene scene;
    auto& reg = scene.registry();

    const int   belt_length_mm  = 4000;
    const int   item_length_mm  = 300;
    const int   item_width_mm   = 300;
    const float belt_speed      = 1000.f;
    const float saturated_rate  = 36000.f;   // 10/s — way over any belt's capacity

    auto belt_e  = scene.add_belt(item_width_mm + 100, belt_length_mm, 0, belt_speed,
                                  {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry", {0.f,                       0.f, 0.f},
                                  {1.f, 0.f, 0.f});
    auto exit_e  = scene.add_port("exit",  {float(belt_length_mm),     0.f, 0.f},
                                  {1.f, 0.f, 0.f});
    scene.connect_belt(belt_e, entry_e, exit_e);
    scene.set_port_transport(entry_e, belt_e);
    scene.add_laser_sensor(entry_e);   // for source backpressure
    scene.add_laser_sensor(exit_e);    // for sink detection

    auto proto_e = scene.add_prototype(item_length_mm, item_width_mm, 200, 0u);
    scene.add_source(saturated_rate, proto_e, entry_e);
    scene.add_sink(exit_e);

    // Run long enough for startup + a meaningful steady-state window.
    const float dt          = 0.01f;
    const int   total_ticks = 2000;     // 20 simulated seconds
    const float startup_s   = float(belt_length_mm) / belt_speed;  // = 4 s

    int   despawned_after_startup = 0;
    float t_first_post_startup    = -1.f;
    float t_end                   = 0.f;

    for (int tk = 0; tk < total_ticks; ++tk) {
        const float now = (tk + 1) * dt;
        factory::sensor::scan(scene);
        factory::transport::step(scene, dt);
        auto events = factory::lifecycle::step(scene, dt);
        for (auto e : events.despawned) {
            if (now >= startup_s) {
                if (t_first_post_startup < 0.f) t_first_post_startup = now;
                ++despawned_after_startup;
                t_end = now;
            }
            if (reg.valid(e)) reg.destroy(e);
        }
    }

    REQUIRE(despawned_after_startup >= 10);
    const float window_s            = t_end - t_first_post_startup;
    const float simulated_per_min   = float(despawned_after_startup - 1) * 60.f / window_s;

    factory::throughput::BeltParams p;
    p.max_speed_mm_s = belt_speed;
    p.width_mm       = item_width_mm + 100;
    factory::throughput::BeltGeometry g;
    g.item_length_mm = item_length_mm;
    g.item_width_mm  = item_width_mm;
    g.safety_gap_mm  = 0;
    const float analytic_per_min = factory::throughput::belt(p, g);

    // The sim is strictly *more conservative* than the continuous analytic:
    // discrete-time spawn gating adds 2 * dt * belt_speed of dead space per
    // item (the laser-clear check uses pre-move sensor state, and spawn
    // happens at end of tick). At dt=0.01 and 1000 mm/s, that's 20 mm of
    // overhead per 300 mm item → ~6% slower than the analytic prediction.
    // This is the expected O(dt) discretization error — bounded, one-sided,
    // and shrinks as dt → 0. We allow 10% tolerance here; a tighter sim
    // (smaller dt or sub-tick laser-crossing detection) would close the gap.
    REQUIRE(simulated_per_min <= analytic_per_min);
    REQUIRE_NEAR(simulated_per_min, analytic_per_min, analytic_per_min * 0.10f);
}

// ── Throughput model: palletizer station ────────────────────────────────────

TEST(boxes_per_pallet_demo_geometry) {
    // Demo dimensions: 1200x800x1500 mm pallet, 250x250x200 mm boxes.
    // Theoretical max packing: 4 wide, 3 deep, 7 high = 84 boxes.
    factory::throughput::PalletizerGeometry g;
    g.pallet_length_mm    = 1200;
    g.pallet_width_mm     = 800;
    g.pallet_max_stack_mm = 1500;
    g.box_length_mm       = 250;
    g.box_width_mm        = 250;
    g.box_height_mm       = 200;
    REQUIRE_EQ(factory::throughput::boxes_per_pallet(g), 84);
}

TEST(boxes_per_pallet_single_layer_when_stack_small) {
    // If the max stack height is smaller than one box, the result is zero —
    // the geometry is infeasible.
    factory::throughput::PalletizerGeometry g;
    g.pallet_length_mm    = 1200;
    g.pallet_width_mm     = 800;
    g.pallet_max_stack_mm = 100;     // below one box height
    g.box_length_mm       = 250;
    g.box_width_mm        = 250;
    g.box_height_mm       = 200;
    REQUIRE_EQ(factory::throughput::boxes_per_pallet(g), 0);
}

TEST(palletizer_throughput_one_picker) {
    // 12-box pattern (matches demo's GridPattern), one picker doing 18 box/min.
    const float result = factory::throughput::palletizer(12, 18.f, 1);
    REQUIRE_NEAR(result, 1.5f, 1e-3f);
}

TEST(palletizer_throughput_two_pickers_doubles) {
    const float one  = factory::throughput::palletizer(12, 18.f, 1);
    const float two  = factory::throughput::palletizer(12, 18.f, 2);
    REQUIRE_NEAR(two, 2.f * one, 1e-3f);
}

TEST(palletizer_throughput_monotonic_in_picker_throughput) {
    const float slow = factory::throughput::palletizer(12, 10.f, 1);
    const float fast = factory::throughput::palletizer(12, 30.f, 1);
    REQUIRE(fast > slow);
}

TEST(palletizer_throughput_monotonic_in_pallet_capacity) {
    // More boxes per pallet → fewer pallets per minute (same picker rate).
    // Slightly counterintuitive but correct: the picker has to fill bigger
    // pallets, so they complete more slowly.
    const float small_pallet = factory::throughput::palletizer(12, 18.f, 1);
    const float big_pallet   = factory::throughput::palletizer(60, 18.f, 1);
    REQUIRE(small_pallet > big_pallet);
}

// ── Depot transport ─────────────────────────────────────────────────────────
//
// A depot is a stationary transport that holds items at pattern-determined
// positions. For capacity-1 depots, source backpressure is handled by a
// laser sensor at the port — the source can't spawn while the laser is
// blocked by the existing item.

TEST(depot_setters_round_trip) {
    factory::DepotTransportComponent dc;
    dc.set_length_mm(250);
    dc.set_width_mm(250);
    dc.set_height_mm(0);
    dc.set_forward_axis({0.f, 1.f, 0.f});
    auto pattern = std::make_shared<factory::GridPattern>();
    dc.set_pattern(pattern);

    REQUIRE_EQ(dc.length_mm(), 250);
    REQUIRE_EQ(dc.width_mm(),  250);
    REQUIRE_EQ(dc.height_mm(), 0);
    REQUIRE(dc.forward_axis().y == 1.f);
    REQUIRE(dc.pattern() != nullptr);
}

TEST(add_depot_creates_pose_and_transport) {
    factory::FactoryScene scene;
    auto& reg = scene.registry();
    auto depot = scene.add_depot({500.f, 0.f, 0.f},
                                 250, 250, 0,
                                 std::make_shared<factory::GridPattern>());
    REQUIRE(reg.any_of<factory::DepotTransportComponent>(depot));
    REQUIRE(reg.any_of<factory::TransportComponent>(depot));
    REQUIRE(reg.any_of<factory::PoseComponent>(depot));
    REQUIRE_NEAR(reg.get<factory::PoseComponent>(depot).position.x, 500.f, 1e-3f);
}

TEST(depot_source_spawn_places_item_at_depot_centre) {
    // Item should appear at the depot's world position (slot 0 of a 1×1 grid),
    // *not* shifted back like belt spawns. Depot footprint = item dimensions,
    // so GridPattern yields exactly one slot at the surface centre.
    factory::FactoryScene scene;
    auto& reg = scene.registry();

    auto proto = scene.add_prototype(250, 250, 200, 0u);
    auto depot = scene.add_depot({1000.f, 0.f, 800.f},
                                 250, 250, 0,
                                 std::make_shared<factory::GridPattern>());
    auto port  = scene.add_port("depot_port", {1000.f, 0.f, 800.f}, {1.f, 0.f, 0.f});
    scene.set_port_transport(port, depot);

    scene.add_source(3600.f, proto, port);    // 1/s; one spawn per second
    auto events = factory::lifecycle::step(scene, 1.0f);

    REQUIRE_EQ(static_cast<int>(events.spawned.size()), 1);
    const auto item = events.spawned[0].entity;
    const auto& ipose = reg.get<factory::PoseComponent>(item);
    // Depot at (1000, 0, 800); GridPattern slot 0 with depot dims = item dims
    // yields local (0, 0, depot.height_mm=0). World = (1000, 0, 800).
    REQUIRE_NEAR(ipose.position.x, 1000.f, 1e-3f);
    REQUIRE_NEAR(ipose.position.y,    0.f, 1e-3f);
    REQUIRE_NEAR(ipose.position.z,  800.f, 1e-3f);
}

TEST(depot_does_not_apply_belt_spawn_shift) {
    // Regression: a depot with a +x forward axis must *not* shift the spawn
    // back by half item length (that's belt-only behaviour). The item should
    // land at the depot centre, not at -125 along x.
    factory::FactoryScene scene;
    auto& reg = scene.registry();

    auto proto = scene.add_prototype(250, 250, 200, 0u);
    auto depot = scene.add_depot({0.f, 0.f, 0.f},
                                 250, 250, 0,
                                 std::make_shared<factory::GridPattern>(),
                                 {1.f, 0.f, 0.f});
    auto port  = scene.add_port("p", {0.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    scene.set_port_transport(port, depot);

    scene.add_source(3600.f, proto, port);
    auto events = factory::lifecycle::step(scene, 1.0f);

    const auto item = events.spawned[0].entity;
    REQUIRE_NEAR(reg.get<factory::PoseComponent>(item).position.x, 0.f, 1e-3f);
}

TEST(depot_backpressure_via_laser_blocks_second_spawn) {
    // Capacity-1 depot with a laser sensor at the port. First spawn places
    // item at depot centre, laser detects it → port blocked → second spawn
    // does not happen on the next tick.
    factory::FactoryScene scene;
    auto& reg = scene.registry();

    auto proto = scene.add_prototype(250, 250, 200, 0u);
    auto depot = scene.add_depot({0.f, 0.f, 0.f},
                                 250, 250, 0,
                                 std::make_shared<factory::GridPattern>());
    auto port  = scene.add_port("p", {0.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    scene.set_port_transport(port, depot);
    scene.add_laser_sensor(port);
    scene.add_source(3600.f, proto, port);

    auto first  = factory::lifecycle::step(scene, 1.0f);
    REQUIRE_EQ(static_cast<int>(first.spawned.size()), 1);

    // Run sensor::scan so the laser registers the just-spawned item.
    factory::sensor::scan(scene);
    auto second = factory::lifecycle::step(scene, 1.0f);
    REQUIRE_EQ(static_cast<int>(second.spawned.size()), 0);
    (void)reg;
}

TEST(depot_full_grid_returns_nullopt_through_pattern) {
    // Sanity check that PlacementSurface refactor preserves "full" semantics.
    // GridPattern with 1×1 capacity and placed_count=1 must return nullopt.
    factory::PlacementSurface surface;
    surface.length_mm    = 250;
    surface.width_mm     = 250;
    surface.height_mm    = 0;
    surface.placed_count = 1;

    factory::ItemPrototypeComponent proto;
    proto.set_length_mm(250);
    proto.set_width_mm(250);
    proto.set_height_mm(200);

    factory::GridPattern pattern;
    REQUIRE(!pattern.next_pose(surface, proto).has_value());
}

// ── Robot-arm catalog ───────────────────────────────────────────────────────

static factory::robot_arm_catalog::RobotArmSpec make_test_arm(float price_eur = 30000.f,
                                                              float mass_kg   = 50.f,
                                                              float payload_kg = 10.f,
                                                              float reach_max_m = 1.0f,
                                                              float speed_max_m_s = 2.f,
                                                              float power_idle_w = 250.f,
                                                              float accel_max = 10.f)
{
    factory::robot_arm_catalog::RobotArmSpec s{};
    s.name                       = "test_arm";
    s.price_purchase_eur         = price_eur;
    s.mass_robot_kg              = mass_kg;
    s.payload_max_kg             = payload_kg;
    s.reach_max_m                = reach_max_m;
    s.reach_min_m                = 0.f;
    s.speed_max_m_s              = speed_max_m_s;
    s.power_idle_w               = power_idle_w;
    s.power_peak_w               = 1500.f;
    s.acceleration_max_m_s2      = accel_max;
    s.maintenance_cost_annual_eur = price_eur * 0.04f;
    s.lifetime_years             = 14.f;
    s.regen_capable              = false;
    return s;
}

TEST(catalog_defaults_resolve_nulls) {
    nlohmann::json j = nlohmann::json::parse(R"({
        "name": "test",
        "urdf": "x.urdf",
        "price_purchase_eur": 50000.0,
        "mass_robot_kg": 60.0,
        "payload_max_kg": 10.0,
        "reach_max_m": 1.0,
        "speed_max_m_s": 2.0,
        "maintenance_cost_annual_eur": null,
        "lifetime_years": null,
        "power_idle_w": null,
        "controller_class": "large"
    })");
    auto s = factory::robot_arm_catalog::parse_entry(j);
    REQUIRE_NEAR(s.lifetime_years, 14.f, 1e-3f);
    REQUIRE_NEAR(s.maintenance_cost_annual_eur, 50000.f * 0.04f, 1e-3f);
    REQUIRE_NEAR(s.power_idle_w, 400.f, 1e-3f);    // large → 400 W
    REQUIRE_NEAR(s.reach_min_m, 0.f, 1e-6f);
}

TEST(catalog_load_kuka_brand_file) {
    auto arms = factory::robot_arm_catalog::load("assets/robots/kuka/catalog.json");
    REQUIRE_EQ(static_cast<int>(arms.size()), 11);  // 5 Agilus + 3 Cybertech + 3 Quantec PA
    // First entry is KR4_R600 (smallest Agilus).
    REQUIRE(arms[0].name == "KR4_R600");
    REQUIRE_NEAR(arms[0].reach_max_m, 0.601f, 1e-3f);
    REQUIRE_NEAR(arms[0].payload_max_kg, 4.f,  1e-3f);
    // Last entry is the heaviest Quantec PA palletizer.
    REQUIRE(arms.back().name == "KR240_R3200_PA");
    REQUIRE_NEAR(arms.back().reach_max_m, 3.195f, 1e-3f);
    REQUIRE_NEAR(arms.back().payload_max_kg, 240.f, 1e-3f);
}

TEST(catalog_arm_without_urdf_loads_with_empty_path) {
    // Cybertech and Quantec PA entries have urdf: null in JSON — the loader
    // should resolve that to an empty string (not throw or skip the entry).
    auto arms = factory::robot_arm_catalog::load("assets/robots/kuka/catalog.json");
    bool found_kr16 = false;
    for (const auto& a : arms) {
        if (a.name == "KR16_R1610") {
            found_kr16 = true;
            REQUIRE(a.urdf_path.empty());
            REQUIRE(a.payload_max_kg > 0.f);   // catalog data still loaded
        }
    }
    REQUIRE(found_kr16);
}

TEST(cost_quantec_pa_feasible_for_full_pallet) {
    // A real palletizing scenario the Agilus family can't handle: 1200x800mm
    // pallet, ~30 kg loaded pallet weight, lifted to a 1.2 m drop height,
    // 2.5 m operating distance from robot base. KR240 R3200 PA should be
    // feasible; KR10 R1100 should not.
    auto arms = factory::robot_arm_catalog::load("assets/robots/kuka/catalog.json");
    const factory::robot_arm_catalog::RobotArmSpec* kr10  = nullptr;
    const factory::robot_arm_catalog::RobotArmSpec* kr240 = nullptr;
    for (const auto& a : arms) {
        if (a.name == "KR10_R1100_2") kr10  = &a;
        if (a.name == "KR240_R3200_PA") kr240 = &a;
    }
    REQUIRE(kr10 != nullptr);
    REQUIRE(kr240 != nullptr);

    factory::cost::TaskParams t{};
    t.distance_m           = 1.5f;
    t.payload_mass_kg      = 30.f;
    t.cycle_time_s         = 4.f;
    t.vertical_lift_m      = 1.2f;
    t.operating_distance_m = 2.5f;
    factory::cost::ExternalParams ext{0.30f, 4000.f};

    auto r_kr10  = factory::cost::estimate(*kr10,  t, ext);
    auto r_kr240 = factory::cost::estimate(*kr240, t, ext);
    REQUIRE(!r_kr10.feasible);          // payload AND reach exceed the small arm
    REQUIRE(r_kr240.feasible);
}

// ── Cost model: feasibility ─────────────────────────────────────────────────

TEST(cost_payload_too_heavy_infeasible) {
    auto spec = make_test_arm(/*price*/30000.f, /*mass*/50.f, /*payload_max*/5.f);
    factory::cost::TaskParams t{};
    t.distance_m       = 0.5f;
    t.payload_mass_kg  = 10.f;       // exceeds 5 kg limit
    t.cycle_time_s     = 2.f;
    factory::cost::ExternalParams ext{0.30f, 2000.f};
    auto r = factory::cost::estimate(spec, t, ext);
    REQUIRE(!r.feasible);
    REQUIRE(r.infeasible_reason != nullptr);
    REQUIRE(std::isinf(r.annual_cost_eur));
}

TEST(cost_reach_too_far_infeasible) {
    auto spec = make_test_arm(30000.f, 50.f, 10.f, /*reach_max*/0.7f);
    factory::cost::TaskParams t{};
    t.distance_m       = 0.3f;
    t.payload_mass_kg  = 2.f;
    t.cycle_time_s     = 2.f;
    t.operating_distance_m = 1.5f;   // exceeds 0.7 reach
    factory::cost::ExternalParams ext{0.30f, 2000.f};
    auto r = factory::cost::estimate(spec, t, ext);
    REQUIRE(!r.feasible);
}

TEST(cost_speed_too_fast_infeasible) {
    auto spec = make_test_arm(30000.f, 50.f, 10.f, 1.0f, /*speed_max*/0.5f);
    factory::cost::TaskParams t{};
    t.distance_m       = 1.0f;
    t.payload_mass_kg  = 2.f;
    t.cycle_time_s     = 1.0f;       // v_peak = 1.5 * 1.0 / 1.0 = 1.5 m/s > 0.5
    factory::cost::ExternalParams ext{0.30f, 2000.f};
    auto r = factory::cost::estimate(spec, t, ext);
    REQUIRE(!r.feasible);
}

TEST(cost_acceleration_too_high_infeasible) {
    auto spec = make_test_arm(30000.f, 50.f, 10.f, 1.0f, 5.f, 250.f, /*accel_max*/2.f);
    factory::cost::TaskParams t{};
    t.distance_m       = 1.0f;
    t.payload_mass_kg  = 2.f;
    t.cycle_time_s     = 1.0f;       // required_accel = 4.5 * 1 / 1 = 4.5 m/s² > 2
    factory::cost::ExternalParams ext{0.30f, 2000.f};
    auto r = factory::cost::estimate(spec, t, ext);
    REQUIRE(!r.feasible);
}

TEST(cost_accel_zero_means_unconstrained) {
    // acceleration_max_m_s2 == 0 should mean "no constraint", not "fails everything".
    auto spec = make_test_arm(30000.f, 50.f, 10.f, 1.0f, 5.f, 250.f, /*accel_max*/0.f);
    factory::cost::TaskParams t{};
    t.distance_m       = 1.0f;
    t.payload_mass_kg  = 2.f;
    t.cycle_time_s     = 1.0f;
    factory::cost::ExternalParams ext{0.30f, 2000.f};
    auto r = factory::cost::estimate(spec, t, ext);
    REQUIRE(r.feasible);
}

// ── Cost model: numerical behaviour ─────────────────────────────────────────

TEST(cost_breakdown_components_non_negative) {
    auto spec = make_test_arm();
    factory::cost::TaskParams t{};
    t.distance_m         = 0.5f;
    t.payload_mass_kg    = 3.f;
    t.cycle_time_s       = 2.f;
    t.vertical_lift_m    = 0.2f;
    factory::cost::ExternalParams ext{0.30f, 2000.f};
    auto r = factory::cost::estimate(spec, t, ext);
    REQUIRE(r.feasible);
    REQUIRE(r.energy_per_cycle_j  >= 0.f);
    REQUIRE(r.annual_kwh          >= 0.f);
    REQUIRE(r.annual_energy_eur   >= 0.f);
    REQUIRE(r.annual_maintenance_eur >= 0.f);
    REQUIRE(r.annualised_capex_eur > 0.f);
    REQUIRE(r.annual_cost_eur     > 0.f);
}

TEST(cost_cheaper_arm_wins_when_both_feasible) {
    // Two specs differing only in price. Same task → cheaper arm gives lower cost.
    auto cheap     = make_test_arm(/*price*/25000.f);
    auto expensive = make_test_arm(/*price*/45000.f);
    factory::cost::TaskParams t{};
    t.distance_m       = 0.4f;
    t.payload_mass_kg  = 2.f;
    t.cycle_time_s     = 3.f;
    factory::cost::ExternalParams ext{0.30f, 2000.f};
    auto rc = factory::cost::estimate(cheap, t, ext);
    auto re = factory::cost::estimate(expensive, t, ext);
    REQUIRE(rc.feasible);
    REQUIRE(re.feasible);
    REQUIRE(rc.annual_cost_eur < re.annual_cost_eur);
}

TEST(cost_reach_forces_bigger_arm) {
    // Small-reach arm infeasible; bigger-reach arm feasible.
    auto small_reach = make_test_arm(25000.f, 30.f, 4.f, /*reach_max*/0.6f);
    auto big_reach   = make_test_arm(48000.f, 58.f, 10.f, /*reach_max*/1.1f);
    factory::cost::TaskParams t{};
    t.distance_m       = 0.4f;
    t.payload_mass_kg  = 2.f;
    t.cycle_time_s     = 2.f;
    t.operating_distance_m = 0.9f;   // beyond 0.6, within 1.1
    factory::cost::ExternalParams ext{0.30f, 2000.f};
    auto r_small = factory::cost::estimate(small_reach, t, ext);
    auto r_big   = factory::cost::estimate(big_reach,   t, ext);
    REQUIRE(!r_small.feasible);
    REQUIRE(r_big.feasible);
}

TEST(cost_operating_distance_defaults_to_midrange) {
    // When operating_distance == 0, function should fall back to
    // (reach_max + reach_min) / 2. Verify by computing both ways.
    auto spec = make_test_arm();
    spec.reach_max_m = 1.0f;
    spec.reach_min_m = 0.2f;

    factory::cost::TaskParams t_default{};
    t_default.distance_m       = 0.3f;
    t_default.payload_mass_kg  = 2.f;
    t_default.cycle_time_s     = 2.f;
    // operating_distance_m left at 0 → resolved to (1.0 + 0.2) / 2 = 0.6

    factory::cost::TaskParams t_explicit = t_default;
    t_explicit.operating_distance_m = 0.6f;

    factory::cost::ExternalParams ext{0.30f, 2000.f};
    auto r_def = factory::cost::estimate(spec, t_default, ext);
    auto r_exp = factory::cost::estimate(spec, t_explicit, ext);
    REQUIRE(r_def.feasible);
    REQUIRE(r_exp.feasible);
    REQUIRE_NEAR(r_def.annual_cost_eur, r_exp.annual_cost_eur, 1e-2f);
}

TEST(cost_higher_electricity_price_increases_energy_eur) {
    auto spec = make_test_arm();
    factory::cost::TaskParams t{};
    t.distance_m       = 0.4f;
    t.payload_mass_kg  = 3.f;
    t.cycle_time_s     = 2.f;
    factory::cost::ExternalParams cheap_e{0.10f, 2000.f};
    factory::cost::ExternalParams pricey_e{0.50f, 2000.f};
    auto r_cheap  = factory::cost::estimate(spec, t, cheap_e);
    auto r_pricey = factory::cost::estimate(spec, t, pricey_e);
    REQUIRE(r_pricey.annual_energy_eur > r_cheap.annual_energy_eur);
    // kWh same, EUR scales linearly
    REQUIRE_NEAR(r_cheap.annual_kwh, r_pricey.annual_kwh, 1e-3f);
    REQUIRE_NEAR(r_pricey.annual_energy_eur / r_cheap.annual_energy_eur, 5.0f, 1e-2f);
}

TEST(cost_regen_capable_uses_higher_efficiency) {
    // A regen-capable arm should have lower kinetic-energy contribution,
    // hence lower annual energy at the same task.
    auto no_regen = make_test_arm();
    auto regen    = make_test_arm();
    regen.regen_capable = true;
    factory::cost::TaskParams t{};
    t.distance_m       = 0.8f;        // longer distance amplifies kinetic share
    t.payload_mass_kg  = 5.f;
    t.cycle_time_s     = 1.5f;
    factory::cost::ExternalParams ext{0.30f, 2000.f};
    auto r_no = factory::cost::estimate(no_regen, t, ext);
    auto r_re = factory::cost::estimate(regen,    t, ext);
    REQUIRE(r_no.feasible);
    REQUIRE(r_re.feasible);
    REQUIRE(r_re.annual_kwh < r_no.annual_kwh);
}

// ── Cost-option enumerators ─────────────────────────────────────────────────

TEST(robot_arm_options_returns_only_feasible_arms_sorted) {
    auto catalog = factory::robot_arm_catalog::load("assets/robots/kuka/catalog.json");
    factory::cost::TaskParams base{};
    base.distance_m       = 0.4f;
    base.payload_mass_kg  = 3.f;
    base.cycle_time_s     = 0.f;        // overwritten by enumerator
    factory::cost::ExternalParams ext{0.30f, 2000.f};

    // Low demand → most arms are feasible.
    auto opts = factory::cost::robot_arm_options(15.f, catalog, base, ext);

    REQUIRE(!opts.empty());
    // Sorted ascending by annual cost.
    for (size_t i = 1; i < opts.size(); ++i) {
        REQUIRE(opts[i - 1].breakdown.annual_cost_eur
                <= opts[i].breakdown.annual_cost_eur);
    }
    // All returned options must be feasible.
    for (const auto& o : opts) REQUIRE(o.breakdown.feasible);
    // The cheapest feasible should be one of the small Agilus arms — they
    // outweigh the bigger ones for a 3 kg / 0.4 m task by capex.
    REQUIRE(opts.front().spec.name.find("KR4") != std::string::npos ||
            opts.front().spec.name.find("KR6") != std::string::npos);
}

TEST(robot_arm_options_filters_infeasible_payload) {
    auto catalog = factory::robot_arm_catalog::load("assets/robots/kuka/catalog.json");
    factory::cost::TaskParams base{};
    base.distance_m       = 0.4f;
    base.payload_mass_kg  = 50.f;       // exceeds every Agilus and Cybertech-low
    factory::cost::ExternalParams ext{0.30f, 2000.f};

    auto opts = factory::cost::robot_arm_options(5.f, catalog, base, ext);
    // No Agilus arm has payload >= 50 kg; only Quantec PA arms qualify.
    for (const auto& o : opts) {
        REQUIRE(o.spec.payload_max_kg >= 50.f);
        REQUIRE(o.spec.name.find("PA") != std::string::npos);
    }
}

TEST(robot_arm_options_high_demand_eliminates_slow_arms) {
    // Very short cycle requires high v_peak; arms with lower speed_max fail.
    auto catalog = factory::robot_arm_catalog::load("assets/robots/kuka/catalog.json");
    factory::cost::TaskParams base{};
    base.distance_m       = 1.0f;
    base.payload_mass_kg  = 2.f;
    base.operating_distance_m = 0.5f;   // within all reach envelopes
    factory::cost::ExternalParams ext{0.30f, 2000.f};

    auto few   = factory::cost::robot_arm_options(20.f,  catalog, base, ext);  // cycle 3.0 s
    auto many  = factory::cost::robot_arm_options(100.f, catalog, base, ext);  // cycle 0.6 s

    REQUIRE(many.size() < few.size());   // some arms drop out at high demand
}

// ── Palletizer options ──────────────────────────────────────────────────────

TEST(palletizer_options_one_big_vs_many_small_visible) {
    auto catalog = factory::robot_arm_catalog::load("assets/robots/kuka/catalog.json");

    factory::cost::PalletizerContext ctx{};
    ctx.geometry.pallet_length_mm    = 1200;
    ctx.geometry.pallet_width_mm     = 800;
    ctx.geometry.pallet_max_stack_mm = 200;        // single layer
    ctx.geometry.box_length_mm       = 300;
    ctx.geometry.box_width_mm        = 200;
    ctx.geometry.box_height_mm       = 200;
    ctx.per_arm_task.distance_m      = 0.6f;
    ctx.per_arm_task.payload_mass_kg = 5.f;
    ctx.per_arm_task.operating_distance_m = 0.7f;
    ctx.station_mech_capex_eur                = 15000.f;
    ctx.station_mech_power_w                  = 150.f;
    ctx.station_mech_maintenance_annual_eur   = 600.f;
    ctx.max_pickers                           = 3;

    factory::cost::ExternalParams ext{0.30f, 2000.f};

    // Moderate pallet demand → multiple (N, arm) combinations feasible.
    auto opts = factory::cost::palletizer_options(0.5f, catalog, ctx, ext);
    REQUIRE(!opts.empty());

    // Sort order ascending by total cost.
    for (size_t i = 1; i < opts.size(); ++i) {
        REQUIRE(opts[i - 1].annual_total_cost_eur
                <= opts[i].annual_total_cost_eur);
    }

    // The (N, arm) trade-off must be solver-visible: at least 2 distinct
    // num_pickers values should appear among the feasible options.
    int n1 = 0, n_gt1 = 0;
    for (const auto& o : opts) {
        if (o.num_pickers == 1) ++n1;
        else if (o.num_pickers > 1) ++n_gt1;
    }
    REQUIRE(n1 > 0);
    REQUIRE(n_gt1 > 0);
}

TEST(palletizer_options_includes_station_mech_cost) {
    // The composite total must include the station mechanism contributions,
    // not just N × per_arm.
    auto catalog = factory::robot_arm_catalog::load("assets/robots/kuka/catalog.json");

    factory::cost::PalletizerContext ctx{};
    ctx.geometry.pallet_length_mm    = 1200;
    ctx.geometry.pallet_width_mm     = 800;
    ctx.geometry.pallet_max_stack_mm = 200;
    ctx.geometry.box_length_mm       = 300;
    ctx.geometry.box_width_mm        = 200;
    ctx.geometry.box_height_mm       = 200;
    ctx.per_arm_task.distance_m      = 0.6f;
    ctx.per_arm_task.payload_mass_kg = 5.f;
    ctx.per_arm_task.operating_distance_m = 0.7f;
    ctx.station_mech_capex_eur                = 15000.f;
    ctx.station_mech_power_w                  = 150.f;
    ctx.station_mech_maintenance_annual_eur   = 600.f;
    ctx.max_pickers                           = 2;
    ctx.station_mech_lifetime_years           = 14.f;

    factory::cost::ExternalParams ext{0.30f, 2000.f};

    auto opts = factory::cost::palletizer_options(0.5f, catalog, ctx, ext);
    REQUIRE(!opts.empty());
    const auto& o = opts.front();

    // Independently compute expected components.
    const float expected_capex = 15000.f / 14.f;
    const float expected_kwh   = 150.f * 2000.f / 1000.f;
    const float expected_e_eur = expected_kwh * 0.30f;
    REQUIRE_NEAR(o.station_mech_annualised_capex_eur, expected_capex, 0.1f);
    REQUIRE_NEAR(o.station_mech_annual_energy_eur,    expected_e_eur, 0.1f);
    REQUIRE_NEAR(o.station_mech_maintenance_annual_eur, 600.f, 1e-3f);

    // And the total respects them.
    const float expected_total =
        float(o.num_pickers) * o.per_arm_breakdown.annual_cost_eur
      + expected_capex + 600.f + expected_e_eur;
    REQUIRE_NEAR(o.annual_total_cost_eur, expected_total, 0.5f);
}

TEST(palletizer_options_infeasible_returns_empty) {
    // Pallet too tall to fit any box: boxes_per_pallet = 0 → won't fire the
    // assert because we set max_stack >= box.height. Instead, push demand
    // beyond any arm's capability with a tight cycle and a far reach.
    auto catalog = factory::robot_arm_catalog::load("assets/robots/kuka/catalog.json");

    factory::cost::PalletizerContext ctx{};
    ctx.geometry.pallet_length_mm    = 1200;
    ctx.geometry.pallet_width_mm     = 800;
    ctx.geometry.pallet_max_stack_mm = 200;
    ctx.geometry.box_length_mm       = 300;
    ctx.geometry.box_width_mm        = 200;
    ctx.geometry.box_height_mm       = 200;
    ctx.per_arm_task.distance_m      = 5.0f;     // very far
    ctx.per_arm_task.payload_mass_kg = 500.f;    // beyond every arm in catalog
    ctx.per_arm_task.operating_distance_m = 5.0f;
    ctx.station_mech_capex_eur       = 15000.f;
    ctx.station_mech_power_w         = 150.f;
    ctx.station_mech_maintenance_annual_eur = 600.f;
    ctx.max_pickers                  = 3;

    factory::cost::ExternalParams ext{0.30f, 2000.f};
    auto opts = factory::cost::palletizer_options(10.f, catalog, ctx, ext);
    REQUIRE(opts.empty());
}

// ── Station-solver types ────────────────────────────────────────────────────
//
// Type-validation only — verifies the shared vocabulary the strategies and
// solver loop will consume. No solver logic yet.

TEST(station_solver_palletize_task_constructs) {
    factory::station_solver::Task t{};
    t.kind                        = factory::station_solver::TaskKind::Palletize;
    t.throughput_items_per_minute = 30.f;
    t.pallet_length_mm            = 1200;
    t.pallet_width_mm             = 800;
    t.pallet_max_stack_mm         = 145;
    t.box_length_mm               = 250;
    t.box_width_mm                = 250;
    t.box_height_mm               = 200;
    t.box_mass_kg                 = 5.f;
    REQUIRE(t.kind == factory::station_solver::TaskKind::Palletize);
    REQUIRE_NEAR(t.throughput_items_per_minute, 30.f, 1e-3f);
    REQUIRE_EQ(t.pallet_length_mm, 1200);
}

TEST(station_solver_terminal_arms_proposal_carries_equipment) {
    // A complete sub-tree: arms strategy realises Palletize directly with two
    // robots + a station frame; no sub-tasks remain.
    factory::station_solver::Proposal p{};
    p.solves        = factory::station_solver::TaskKind::Palletize;
    p.strategy_name = "ArmsStrategy";
    p.equipment.push_back({"robot_arm",     "KR10_R1100_2",
                           48000.f, 2500.f});
    p.equipment.push_back({"robot_arm",     "KR10_R1100_2",
                           48000.f, 2500.f});
    p.equipment.push_back({"station_frame", "generic_palletizer_frame",
                           15000.f, 150.f});
    p.annual_cost_eur    = 12500.f;
    p.annual_cost_lb_eur = 12500.f;

    REQUIRE(factory::station_solver::is_terminal(p));
    REQUIRE_EQ(static_cast<int>(p.equipment.size()), 3);
    REQUIRE(p.equipment[0].archetype_name == "robot_arm");
    REQUIRE(p.equipment[2].archetype_name == "station_frame");
}

TEST(station_solver_pusher_proposal_emits_sub_task_conditions) {
    // A push-and-index strategy realises Palletize with a pusher + frame, but
    // emits sub-tasks the solver must dispatch: detect-presence + index-pallet.
    factory::station_solver::Proposal p{};
    p.solves        = factory::station_solver::TaskKind::Palletize;
    p.strategy_name = "PushAndIndexStrategy";
    p.equipment.push_back({"pusher",        "pneumatic_500mm",
                           4000.f, 200.f});
    p.equipment.push_back({"station_frame", "generic_palletizer_frame",
                           8000.f, 100.f});

    factory::station_solver::Task detect{};
    detect.kind            = factory::station_solver::TaskKind::DetectItemPresence;
    detect.source_position = factory::Vec3{0.f, 0.f, 800.f};
    p.remaining.push_back(detect);

    factory::station_solver::Task index{};
    index.kind          = factory::station_solver::TaskKind::IndexPallet;
    index.index_step_mm = 250;
    p.remaining.push_back(index);

    p.annual_cost_eur    = 2500.f;    // own equipment so far
    p.annual_cost_lb_eur = 2700.f;    // + a cheap floor for the two unresolved sub-tasks

    REQUIRE(!factory::station_solver::is_terminal(p));
    REQUIRE_EQ(static_cast<int>(p.remaining.size()), 2);
    REQUIRE(p.remaining[0].kind == factory::station_solver::TaskKind::DetectItemPresence);
    REQUIRE(p.remaining[1].kind == factory::station_solver::TaskKind::IndexPallet);
    REQUIRE_EQ(p.remaining[1].index_step_mm, 250);
    // Lower bound must be at least the cost-so-far (sanity).
    REQUIRE(p.annual_cost_lb_eur >= p.annual_cost_eur);
}

TEST(palletizer_integrates_with_picker_throughput) {
    // End-to-end composition: take the picker's analytic throughput and feed
    // it directly into the palletizer model. Confirms the units line up
    // (boxes/min in, pallets/min out) and that the value chain is consistent.
    factory::throughput::PickerParams p;
    p.max_tcp_speed_mm_s = 1000.f;
    p.grip_time_s        = 0.3f;
    p.release_time_s     = 0.2f;
    factory::throughput::PickerGeometry pg;
    pg.pickup_to_drop_mm = 600.f;

    const float box_per_min = factory::throughput::picker(p, pg);
    const float pallets_per_min = factory::throughput::palletizer(12, box_per_min, 1);

    // box_per_min ≈ 18.5, pallets/min ≈ 18.5/12 ≈ 1.54
    REQUIRE(pallets_per_min > 1.3f);
    REQUIRE(pallets_per_min < 1.8f);
}

TEST(workflow_solve_runs_full_palletizer_cycle) {
    // Same setup as the manual-wiring integration test, but build the scene
    // through the declarative API and then drive the sim. Confirms the
    // solver produces a scene that actually works end-to-end.
    factory::FactoryScene scene;
    auto& reg = scene.registry();

    auto pallet_proto = scene.add_prototype(1200, 800, 145, 0u);
    auto box_proto    = scene.add_prototype(250,  250, 200, 0u);

    auto pallet_source = scene.declare_source( 360.f, pallet_proto);
    auto box_source    = scene.declare_source(1800.f, box_proto);
    auto pallet_sink   = scene.declare_sink();
    auto palletizer    = scene.declare_palletizer_station(pallet_proto, box_proto);

    scene.declare_flow(pallet_source, palletizer,  6.f);
    scene.declare_flow(box_source,    palletizer, 30.f);
    scene.declare_flow(palletizer,    pallet_sink, 6.f);

    scene.solve_workflow();

    // 90 s simulated at the demo's clamped tick rate.
    int pallets_completed = 0;
    const float dt          = 0.05f;
    const int   total_ticks = 1800;

    for (int tick = 0; tick < total_ticks; ++tick) {
        factory::sensor::scan(scene);
        factory::transport::step(scene, dt);
        factory::station::step(scene, dt);
        auto events = factory::lifecycle::step(scene, dt);
        for (auto e : events.despawned) {
            if (auto* palletc = reg.try_get<factory::PalletComponent>(e)) {
                REQUIRE_EQ(static_cast<int>(palletc->items().size()), 12);
                ++pallets_completed;
                for (auto child : palletc->items())
                    if (reg.valid(child)) reg.destroy(child);
            }
            if (reg.valid(e)) reg.destroy(e);
        }
    }

    REQUIRE(pallets_completed >= 1);
}

// ── End-to-end palletizer integration ───────────────────────────────────────
//
// Runs the full demo-style palletizing scene (single mid-tap pallet belt,
// box belt, source / sink, picker, station) for ~60 simulated seconds at the
// demo's clamped tick rate. No threepp dependency. Catches the class of bugs
// that only surface during sustained operation: multi-spawn pile-up, the
// post-release re-claim loop, multi-clamp at exits, item-entity leaks, and
// the station deadlocking with a partial pallet. We care less about the
// exact pallet count than that the system makes forward progress without
// accumulating state.

TEST(palletizer_integration_runs_for_90_seconds) {
    factory::FactoryScene scene;
    auto& reg = scene.registry();

    // Pallet belt — single 6000 mm belt with mid-stream gate / detect ports.
    // Belts run faster than the demo's 200 mm/s so pallet 1 makes it from
    // the tap to the sink before pallet 2 (which inevitably queues up,
    // because the source rate exceeds the station's throughput) reaches
    // the tap and freezes the belt again.
    auto pallet_belt = scene.add_belt(800, 6000, 0, 1000.f, {0.f, 1.f, 0.f});
    auto pal_entry   = scene.add_port("pal_entry", {0.f, -3000.f, 0.f}, {0.f, 1.f, 0.f});
    auto pal_exit    = scene.add_port("pal_exit",  {0.f,  3000.f, 0.f}, {0.f, 1.f, 0.f});
    scene.connect_belt(pallet_belt, pal_entry, pal_exit);
    scene.set_port_transport(pal_entry, pallet_belt);

    auto pal_tap = scene.add_claim_station_taps(pallet_belt, 3000, "pal_tap");

    scene.add_laser_sensor(pal_entry);    // spawn throttle (item body decides spacing)
    scene.add_laser_sensor(pal_exit);     // sink detection

    // Box belt — 2800 mm at height 800.
    auto box_belt  = scene.add_belt(300, 2800, 800, 1000.f, {1.f, 0.f, 0.f});
    auto box_entry = scene.add_port("box_entry", {-2400.f, 0.f, 800.f}, {1.f, 0.f, 0.f});
    auto box_exit  = scene.add_port("box_exit",  {  400.f, 0.f, 800.f}, {1.f, 0.f, 0.f});
    scene.connect_belt(box_belt, box_entry, box_exit);
    scene.set_port_transport(box_entry, box_belt);

    scene.add_laser_sensor(box_entry);
    scene.add_laser_sensor(box_exit);
    auto box_exit_virt = scene.add_virtual_sensor(box_exit);

    // Prototypes & sources.
    auto pallet_proto = scene.add_prototype(1200, 800, 145, 0xC8A060u);
    auto box_proto    = scene.add_prototype(250,  250, 200, 0x8B4513u);
    scene.add_source( 360.f, pallet_proto, pal_entry);
    scene.add_source(1800.f, box_proto,    box_entry);
    scene.add_sink(pal_exit);

    // Picker — using the deterministic linear flavour for predictable timing.
    // 3000 mm/s makes one full pickup-drop-return leg in roughly 1.5 s, so
    // 12 boxes per pallet ≈ 18 s and the box-belt source rate (one every 2 s)
    // keeps up.
    auto picker = scene.add_picker(factory::Vec3{-1000.f, 0.f, 1500.f}, 3000.f);

    // Station.
    auto station_e = reg.create();
    auto& sc = reg.emplace<factory::StationComponent>(station_e);
    sc.set_arrival_port(box_exit);
    sc.set_arrival_virtual_sensor(box_exit_virt);
    sc.add_picker(picker);

    auto& palc = reg.emplace<factory::PalletizeComponent>(station_e);
    palc.set_pallet_arrival_port(pal_tap.detect_port);
    palc.set_pallet_tap_virtual_sensor(pal_tap.virtual_sensor);
    palc.set_pattern(std::make_shared<factory::GridPattern>());
    palc.set_pallet_dimensions(1200, 800, 145, 1500);

    // ── Run loop ────────────────────────────────────────────────────────
    int pallets_completed = 0;
    int boxes_spawned     = 0;
    int pallets_spawned   = 0;
    int peak_items        = 0;

    // 90 s simulated. With 1000 mm/s belts pallet 1 should despawn well
    // before t=60; the extra headroom catches the second pallet too.
    const float dt           = 0.05f;
    const int   total_ticks  = 1800;

    for (int tick = 0; tick < total_ticks; ++tick) {
        factory::sensor::scan(scene);
        factory::transport::step(scene, dt);
        factory::station::step(scene, dt);
        auto events = factory::lifecycle::step(scene, dt);

        for (auto& [item, proto] : events.spawned) {
            if (proto == box_proto)    ++boxes_spawned;
            if (proto == pallet_proto) ++pallets_spawned;
        }

        // Cascade-destroy despawned pallets (mirrors the demo's render-side
        // cleanup, but without meshes). Every despawned pallet should be
        // fully filled — the station only releases on `pattern_full && all
        // pickers idle`, so a half-filled pallet despawning is itself a bug.
        for (auto e : events.despawned) {
            if (auto* palletc = reg.try_get<factory::PalletComponent>(e)) {
                REQUIRE_EQ(static_cast<int>(palletc->items().size()), 12);
                ++pallets_completed;
                for (auto child : palletc->items()) {
                    if (reg.valid(child)) reg.destroy(child);
                }
            }
            if (reg.valid(e)) reg.destroy(e);
        }

        // Track current item count to detect leaks / pile-ups.
        int items_now = 0;
        reg.view<factory::SpawnedItemComponent>().each(
            [&](auto, auto&) { ++items_now; });
        if (items_now > peak_items) peak_items = items_now;
    }

    // ── Assertions ──────────────────────────────────────────────────────
    // 1. The system makes forward progress: at least one pallet completes
    //    and despawns within 60 s. Catches deadlocks (post-release re-claim
    //    loop, station never declaring full, etc.).
    REQUIRE(pallets_completed >= 1);

    // 2. The box source actually produced items (sensor throttling didn't
    //    permanently lock spawn).
    REQUIRE(boxes_spawned >= 12);

    // 3. The pallet source produced at least as many pallets as completed,
    //    plus at least one in flight — otherwise the source is leaking.
    REQUIRE(pallets_spawned >= pallets_completed);

    // 4. No runaway item accumulation. With 1000 mm/s belts and the demo's
    //    source rates, observed peak in-flight is ~37 (boxes on belt + boxes
    //    placed on the queued pallets + ~5 pallets in various states). 60 is
    //    a comfortable bound; an explosion past that signals multi-spawn or
    //    a stuck picker.
    REQUIRE(peak_items < 60);

    // 5. After the run completes, in-flight count is similarly bounded.
    int items_remaining = 0;
    reg.view<factory::SpawnedItemComponent>().each(
        [&](auto, auto&) { ++items_remaining; });
    REQUIRE(items_remaining < 60);
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main() {
    printf("Running sim tests...\n");

    // Component setters
    RUN(conveyor_belt_setters_store_values);
    RUN(conveyor_belt_zero_speed_is_valid);
    RUN(conveyor_belt_default_gate_ports_empty);
    RUN(conveyor_belt_add_tap_port_appends);
    RUN(equipment_cost_defaults_zero);
    RUN(equipment_cost_setters_round_trip);
    RUN(transport_running_default_true);
    RUN(transport_set_running_round_trip);
    RUN(sensor_default_not_blocked);
    RUN(sensor_set_blocked_round_trip);
    RUN(port_no_sensors_default);
    RUN(port_add_sensor_appends);
    RUN(picker_default_state_idle);
    RUN(picker_setters_round_trip);
    RUN(source_rate_round_trip);
    RUN(source_zero_rate_is_valid);
    RUN(declared_opening_width_round_trip);
    RUN(declared_opening_mobility_round_trip);
    RUN(item_prototype_setters);

    // spawn_debt
    RUN(spawn_debt_accumulates);
    RUN(consume_spawn_decrements_by_one);
    RUN(consume_spawn_twice);

    // port_is_clear
    RUN(port_clear_with_no_sensors);
    RUN(port_clear_with_unblocked_virtual_sensor);
    RUN(port_blocked_with_blocked_virtual_sensor);
    RUN(port_blocked_when_any_sensor_blocked);
    RUN(shared_sensor_gates_multiple_ports);

    // sensor::scan
    RUN(sensor_scan_no_items_keeps_clear);
    RUN(sensor_scan_point_item_at_laser_blocks);
    RUN(sensor_scan_point_item_far_from_laser_clear);
    RUN(virtual_sensor_unaffected_by_scan);
    RUN(sensor_scan_triggers_when_body_crosses_laser);
    RUN(sensor_scan_clear_when_body_past_laser);
    RUN(sensor_scan_uses_offset);

    // transport::step (belt)
    RUN(belt_advances_running_clear_item);
    RUN(belt_frozen_when_not_running);
    RUN(belt_frozen_when_exit_port_blocked);
    RUN(tap_in_gate_ports_freezes_belt);
    RUN(tap_not_in_gate_ports_does_not_freeze_belt);
    RUN(belt_handover_to_next_belt);

    // transport::step (picker)
    RUN(picker_idle_does_not_move);
    RUN(picker_advances_toward_pickup_target);
    RUN(picker_clamps_to_pickup_target_and_transitions);
    RUN(picker_carrying_advances_toward_drop_target);
    RUN(picker_at_drop_target_releases_to_container);
    RUN(picker_returning_then_idle_at_home);

    // lifecycle::step
    RUN(source_spawns_when_port_clear);
    RUN(source_does_not_spawn_when_port_blocked);
    RUN(spawned_item_has_correct_prototype);
    RUN(no_spawn_when_debt_below_one);
    RUN(source_shifts_spawn_for_leading_edge_at_port);
    RUN(source_caps_at_one_spawn_per_tick);
    RUN(sink_despawns_item_at_in_port);

    // multi-item-on-one-belt safety
    RUN(belt_does_not_double_clamp_at_terminal_exit);
    RUN(belt_does_not_double_handover);

    // MagicTransportComponent
    RUN(magic_default_state_idle);
    RUN(magic_setters_round_trip);
    RUN(magic_position_lands_on_endpoints);
    RUN(magic_position_handles_zero_length_leg);
    RUN(magic_idle_does_not_advance);
    RUN(magic_completes_leg_and_grabs_box);
    RUN(magic_drop_releases_box_to_container);
    RUN(magic_returns_to_idle_at_home);

    // station::step
    RUN(station_claims_pallet_at_arrival_port);
    RUN(station_skips_already_claimed_pallet);
    RUN(station_dispatches_idle_picker_when_box_arrives);
    RUN(station_does_not_dispatch_without_pallet);
    RUN(station_does_not_double_claim_box);
    RUN(station_arrival_virtual_blocked_when_no_pallet);
    RUN(station_arrival_virtual_clear_with_pallet_and_idle_picker);
    RUN(station_releases_full_pallet_when_all_pickers_idle);
    RUN(station_does_not_release_while_picker_busy);

    // Workflow solver
    RUN(workflow_solve_palletizer_wires_everything);
    RUN(declare_flow_stores_items_per_minute);
    RUN(workflow_solve_attaches_cost_to_every_equipment_entity);
    RUN(workflow_solve_runs_full_palletizer_cycle);

    // Throughput model
    RUN(picker_throughput_zero_distance);
    RUN(picker_throughput_realistic_kuka_kr6);
    RUN(picker_throughput_monotonic_in_speed);
    RUN(picker_throughput_monotonic_in_distance);
    RUN(picker_throughput_idealised_matches_sim_cycle_time);
    RUN(belt_throughput_realistic_packing);
    RUN(belt_throughput_zero_gap);
    RUN(belt_throughput_monotonic_in_speed);
    RUN(belt_throughput_monotonic_in_item_length);
    RUN(belt_throughput_monotonic_in_gap);
    RUN(belt_throughput_idealised_matches_sim_steady_state);
    RUN(boxes_per_pallet_demo_geometry);
    RUN(boxes_per_pallet_single_layer_when_stack_small);
    RUN(palletizer_throughput_one_picker);
    RUN(palletizer_throughput_two_pickers_doubles);
    RUN(palletizer_throughput_monotonic_in_picker_throughput);
    RUN(palletizer_throughput_monotonic_in_pallet_capacity);

    // Depot transport
    RUN(depot_setters_round_trip);
    RUN(add_depot_creates_pose_and_transport);
    RUN(depot_source_spawn_places_item_at_depot_centre);
    RUN(depot_does_not_apply_belt_spawn_shift);
    RUN(depot_backpressure_via_laser_blocks_second_spawn);
    RUN(depot_full_grid_returns_nullopt_through_pattern);

    // Robot-arm catalog
    RUN(catalog_defaults_resolve_nulls);
    RUN(catalog_load_kuka_brand_file);
    RUN(catalog_arm_without_urdf_loads_with_empty_path);

    // Robot-arm cost model
    RUN(cost_payload_too_heavy_infeasible);
    RUN(cost_reach_too_far_infeasible);
    RUN(cost_speed_too_fast_infeasible);
    RUN(cost_acceleration_too_high_infeasible);
    RUN(cost_accel_zero_means_unconstrained);
    RUN(cost_breakdown_components_non_negative);
    RUN(cost_cheaper_arm_wins_when_both_feasible);
    RUN(cost_reach_forces_bigger_arm);
    RUN(cost_operating_distance_defaults_to_midrange);
    RUN(cost_higher_electricity_price_increases_energy_eur);
    RUN(cost_regen_capable_uses_higher_efficiency);
    RUN(cost_quantec_pa_feasible_for_full_pallet);

    // Cost-option enumerators
    RUN(robot_arm_options_returns_only_feasible_arms_sorted);
    RUN(robot_arm_options_filters_infeasible_payload);
    RUN(robot_arm_options_high_demand_eliminates_slow_arms);
    RUN(palletizer_options_one_big_vs_many_small_visible);
    RUN(palletizer_options_includes_station_mech_cost);
    RUN(palletizer_options_infeasible_returns_empty);

    // Station-solver types (foundation; no strategies / solver yet)
    RUN(station_solver_palletize_task_constructs);
    RUN(station_solver_terminal_arms_proposal_carries_equipment);
    RUN(station_solver_pusher_proposal_emits_sub_task_conditions);

    RUN(palletizer_integrates_with_picker_throughput);

    // End-to-end integration
    RUN(palletizer_integration_runs_for_90_seconds);

    printf("\n%d/%d passed", g_run - g_fail, g_run);
    if (g_fail) printf("  (%d FAILED)", g_fail);
    printf("\n");
    return g_fail ? 1 : 0;
}

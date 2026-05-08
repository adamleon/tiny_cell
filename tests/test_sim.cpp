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

    // End-to-end integration
    RUN(palletizer_integration_runs_for_90_seconds);

    printf("\n%d/%d passed", g_run - g_fail, g_run);
    if (g_fail) printf("  (%d FAILED)", g_fail);
    printf("\n");
    return g_fail ? 1 : 0;
}

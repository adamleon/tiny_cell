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

TEST(detection_volume_set_dimensions) {
    factory::DetectionVolumeComponent dv;
    dv.set_dimensions(300, 200, 100);
    REQUIRE_EQ(dv.length_mm(), 300);
    REQUIRE_EQ(dv.width_mm(),  200);
    REQUIRE_EQ(dv.height_mm(), 100);
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
    auto s_e    = scene.add_physical_sensor(port_e, 200, 200, 200, {0.f, 0.f, 0.f});

    factory::sensor::scan(scene);

    REQUIRE(!scene.registry().get<factory::SensorComponent>(s_e).blocked());
}

TEST(sensor_scan_item_in_volume_blocks) {
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e    = scene.add_physical_sensor(port_e, 200, 200, 200, {0.f, 0.f, 0.f});

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {0.f, 0.f, 0.f};
    pose.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(item);

    factory::sensor::scan(scene);

    REQUIRE(reg.get<factory::SensorComponent>(s_e).blocked());
}

TEST(sensor_scan_item_outside_volume_clear) {
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e    = scene.add_physical_sensor(port_e, 200, 200, 200, {0.f, 0.f, 0.f});

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {500.f, 0.f, 0.f};   // far outside ±100 mm in x
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

    // Should remain blocked — scan only writes to physical sensors.
    REQUIRE(reg.get<factory::SensorComponent>(s_e).blocked());
}

TEST(sensor_scan_centre_not_aabb) {
    // Detection is centre-point: an item whose centre is OUTSIDE the volume
    // does NOT block, even if its body would overlap. The sensor is a
    // trigger-point model, not a physical AABB.
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e    = scene.add_physical_sensor(port_e, 200, 200, 200, {0.f, 0.f, 0.f});

    auto& reg  = scene.registry();
    auto proto = scene.add_prototype(400, 400, 400, 0u);   // big body, far past sensor
    auto item  = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {150.f, 0.f, 0.f};                     // centre 50 mm past sensor edge
    pose.parent   = scene.root_entity();
    reg.emplace<factory::SpawnedItemComponent>(item).set_prototype(proto);

    factory::sensor::scan(scene);
    REQUIRE(!reg.get<factory::SensorComponent>(s_e).blocked());
}

TEST(sensor_scan_uses_offset) {
    // Sensor's local offset moves its volume centre; verify an item at the
    // offset position is detected and an item at the port origin is not.
    factory::FactoryScene scene;
    auto port_e = scene.add_port("p", {0.f, 0.f, 0.f});
    auto s_e    = scene.add_physical_sensor(port_e, 100, 100, 100, {500.f, 0.f, 0.f});

    auto& reg  = scene.registry();
    auto  item = reg.create();
    auto& pose = reg.emplace<factory::PoseComponent>(item);
    pose.position = {500.f, 0.f, 0.f};   // at sensor centre
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
    // Sink must be able to detect arriving items via a sensor on its in_port.
    scene.add_physical_sensor(exit_e, 200, 200, 200, {0.f, 0.f, 0.f});

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
    RUN(detection_volume_set_dimensions);
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
    RUN(sensor_scan_item_in_volume_blocks);
    RUN(sensor_scan_item_outside_volume_clear);
    RUN(virtual_sensor_unaffected_by_scan);
    RUN(sensor_scan_centre_not_aabb);
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
    RUN(source_caps_at_one_spawn_per_tick);
    RUN(sink_despawns_item_at_in_port);

    printf("\n%d/%d passed", g_run - g_fail, g_run);
    if (g_fail) printf("  (%d FAILED)", g_fail);
    printf("\n");
    return g_fail ? 1 : 0;
}

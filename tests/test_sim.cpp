// Unit tests for simulation components and sim::step().
// No threepp dependency.
//
// Coverage:
//   - Component setter validation (happy path)
//   - spawn_debt accumulation and consume_spawn
//   - FactoryScene::items_on and transport_has_capacity
//   - sim::step: spawning, item movement, stopped transport, belt-to-belt
//     transfer, terminal despawn, sink counting

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "factory_scene/components.hpp"
#include "factory_scene/factory_scene.hpp"
#include "factory_scene/sim_systems.hpp"

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

TEST(transport_component_defaults) {
    factory::TransportComponent tc;
    REQUIRE(tc.running()   == true);
    REQUIRE(tc.capacity()  == 0);
}

TEST(transport_capacity_zero_is_unlimited) {
    factory::TransportComponent tc;
    tc.set_capacity(0);
    REQUIRE_EQ(tc.capacity(), 0);
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

// ── Scene helpers: items_on / transport_has_capacity ─────────────────────────

// Build a minimal one-belt scene without ECS-level factory setup.
// Returns: {belt_entity, entry_port, exit_port}
static std::tuple<factory::FactoryScene, entt::entity, entt::entity, entt::entity>
make_belt_scene(int capacity = 0, bool running = true) {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 1000, 400, 200.f, {1.f, 0.f, 0.f}, capacity);
    auto entry_e = scene.add_port("entry", {0.f,    0.f, 400.f});
    auto exit_e  = scene.add_port("exit",  {1000.f, 0.f, 400.f});
    scene.connect_belt(belt_e, entry_e, exit_e);
    scene.set_port_transport(entry_e, belt_e);
    // exit transport stays null (terminal)

    if (!running)
        scene.registry().get<factory::TransportComponent>(belt_e).set_running(false);

    return {std::move(scene), belt_e, entry_e, exit_e};
}

TEST(items_on_returns_zero_for_empty_belt) {
    auto [scene, belt_e, entry_e, exit_e] = make_belt_scene();
    REQUIRE_EQ(scene.items_on(belt_e), 0);
}

TEST(transport_has_capacity_when_unlimited) {
    auto [scene, belt_e, entry_e, exit_e] = make_belt_scene(0);
    REQUIRE(scene.transport_has_capacity(belt_e));
}

TEST(transport_at_capacity_returns_false) {
    auto [scene, belt_e, entry_e, exit_e] = make_belt_scene(1);
    // Manually place an item
    auto& reg  = scene.registry();
    auto  item = reg.create();
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_e);
    reg.emplace<factory::PoseComponent>(item).position = {0.f, 0.f, 400.f};
    reg.emplace<factory::SpawnedItemComponent>(item);

    REQUIRE_EQ(scene.items_on(belt_e), 1);
    REQUIRE(!scene.transport_has_capacity(belt_e));
}

TEST(transport_not_running_has_no_capacity) {
    auto [scene, belt_e, entry_e, exit_e] = make_belt_scene(0, false);
    REQUIRE(!scene.transport_has_capacity(belt_e));
}

// ── sim::step: spawning ───────────────────────────────────────────────────────

// Build a scene with a source feeding a belt, then step until the first spawn.
// Rate: 3600/hour = 1/s → dt=1.0 triggers exactly one spawn.
static factory::FactoryScene make_source_scene() {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 2000, 400, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry", {0.f,    0.f, 400.f});
    auto exit_e  = scene.add_port("exit",  {2000.f, 0.f, 400.f});
    scene.connect_belt(belt_e, entry_e, exit_e);
    scene.set_port_transport(entry_e, belt_e);

    auto proto_e = scene.add_prototype(300, 300, 300, 0x8B4513u);
    scene.add_source(3600.f, proto_e, entry_e);
    return scene;
}

TEST(step_spawns_item_when_debt_reaches_one) {
    auto scene  = make_source_scene();
    auto events = factory::sim::step(scene, 1.0f);
    REQUIRE_EQ(static_cast<int>(events.spawned.size()), 1);
}

TEST(spawned_item_has_correct_prototype) {
    auto  scene    = make_source_scene();
    auto& reg      = scene.registry();
    auto  events   = factory::sim::step(scene, 1.0f);
    REQUIRE(!events.spawned.empty());
    auto item      = events.spawned[0].entity;
    auto proto_ref = events.spawned[0].prototype;
    REQUIRE(proto_ref != entt::null);
    REQUIRE(reg.valid(item));
    const auto& sp = reg.get<factory::SpawnedItemComponent>(item);
    REQUIRE(sp.prototype() == proto_ref);
}

TEST(spawned_item_starts_at_entry_port) {
    // Use a very small dt so movement after spawn is negligible (<1 mm).
    // Rate 3,600,000/hr = 1000 items/s → dt=0.001s yields debt=1.0, one spawn,
    // and only 0.2 mm of movement before the check.
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 2000, 400, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry",  {0.f,    0.f, 400.f});
    auto exit_e  = scene.add_port("exit", {2000.f, 0.f, 400.f});
    scene.connect_belt(belt_e, entry_e, exit_e);
    scene.set_port_transport(entry_e, belt_e);
    auto proto_e = scene.add_prototype(300, 300, 300, 0x8B4513u);
    scene.add_source(3'600'000.f, proto_e, entry_e);

    auto& reg    = scene.registry();
    auto  events = factory::sim::step(scene, 0.001f);
    REQUIRE(!events.spawned.empty());
    const auto& pose = reg.get<factory::PoseComponent>(events.spawned[0].entity);
    REQUIRE_NEAR(pose.position.x, 0.f, 1.f);
    REQUIRE_NEAR(pose.position.y, 0.f, 1.f);
    REQUIRE_NEAR(pose.position.z, 400.f, 1.f);
}

TEST(no_spawn_when_debt_below_one) {
    auto  scene  = make_source_scene();
    auto  events = factory::sim::step(scene, 0.2f);  // 3600/hr * 0.2s = 0.2 debt
    REQUIRE(events.spawned.empty());
}

TEST(no_spawn_when_belt_at_capacity) {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 2000, 400, 200.f, {1.f, 0.f, 0.f}, 1);
    auto entry_e = scene.add_port("entry",  {0.f,    0.f, 400.f});
    auto exit_e  = scene.add_port("exit", {2000.f, 0.f, 400.f});
    scene.connect_belt(belt_e, entry_e, exit_e);
    scene.set_port_transport(entry_e, belt_e);

    auto proto_e = scene.add_prototype(300, 300, 300, 0x8B4513u);
    scene.add_source(3600.f, proto_e, entry_e);

    // Fill the capacity slot manually
    auto& reg  = scene.registry();
    auto  item = reg.create();
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_e);
    reg.emplace<factory::PoseComponent>(item).position = {0.f, 0.f, 400.f};
    reg.emplace<factory::SpawnedItemComponent>(item);

    auto events = factory::sim::step(scene, 1.0f);
    REQUIRE(events.spawned.empty());
}

// ── sim::step: movement ───────────────────────────────────────────────────────

// Place an item manually and advance it 1s. Belt: 2000mm, speed 200 mm/s, dir +x.
TEST(item_moves_along_belt_direction) {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 2000, 400, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry",  {0.f,    0.f, 400.f});
    auto exit_e  = scene.add_port("exit", {2000.f, 0.f, 400.f});
    scene.connect_belt(belt_e, entry_e, exit_e);

    auto& reg  = scene.registry();
    auto  item = reg.create();
    reg.emplace<factory::PoseComponent>(item).position = {0.f, 0.f, 400.f};
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_e);
    reg.emplace<factory::SpawnedItemComponent>(item);

    factory::sim::step(scene, 1.0f);  // 200 mm/s × 1s = 200 mm

    const auto& pose = reg.get<factory::PoseComponent>(item);
    REQUIRE_NEAR(pose.position.x, 200.f, 1.f);
    REQUIRE_NEAR(pose.position.y, 0.f,   1e-5f);
}

TEST(stopped_transport_does_not_move_items) {
    auto  scene  = make_source_scene();
    auto& reg    = scene.registry();
    factory::sim::step(scene, 1.0f);  // spawn item

    // Stop the belt
    reg.view<factory::TransportComponent>().each(
        [](factory::TransportComponent& tc) { tc.set_running(false); });

    // Remember position
    factory::Vec3 pos_before;
    reg.view<factory::PoseComponent, factory::SpawnedItemComponent>().each(
        [&](const factory::PoseComponent& pose, const factory::SpawnedItemComponent&) {
            pos_before = pose.position;
        });

    factory::sim::step(scene, 1.0f);

    reg.view<factory::PoseComponent, factory::SpawnedItemComponent>().each(
        [&](const factory::PoseComponent& pose, const factory::SpawnedItemComponent&) {
            REQUIRE_NEAR(pose.position.x, pos_before.x, 1e-5f);
        });
}

// ── sim::step: terminal despawn ───────────────────────────────────────────────

// Belt: 1000mm long, speed 200 mm/s. Spawn then run 5s → item exits.
// No sink → item arrives at port (not despawned).
TEST(item_despawns_at_terminal_exit) {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 1000, 400, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry",  {0.f,    0.f, 400.f});
    auto exit_e  = scene.add_port("exit", {1000.f, 0.f, 400.f});
    scene.connect_belt(belt_e, entry_e, exit_e);
    scene.set_port_transport(entry_e, belt_e);
    scene.add_sink(exit_e);  // sink → item is despawned on exit

    auto proto_e = scene.add_prototype(300, 300, 300, 0xFF0000u);
    scene.add_source(3600.f, proto_e, entry_e);

    factory::sim::step(scene, 1.0f);  // spawn at x=0

    // 5 steps × 1s = 5s → 5 × 200mm = 1000mm travelled → exit
    factory::sim::StepEvents last_events;
    for (int i = 0; i < 5; ++i)
        last_events = factory::sim::step(scene, 1.0f);

    REQUIRE(!last_events.despawned.empty());
}

// Place one item manually and run until it exits. No source keeps spawning extras.
TEST(sink_increments_received_on_despawn) {
    factory::FactoryScene scene;
    auto belt_e  = scene.add_belt(300, 1000, 400, 200.f, {1.f, 0.f, 0.f});
    auto entry_e = scene.add_port("entry",  {0.f,    0.f, 400.f});
    auto exit_e  = scene.add_port("exit", {1000.f, 0.f, 400.f});
    scene.connect_belt(belt_e, entry_e, exit_e);
    scene.add_sink(exit_e);

    auto& reg   = scene.registry();
    auto  proto = scene.add_prototype(300, 300, 300, 0xFF0000u);
    auto  item  = reg.create();
    reg.emplace<factory::PoseComponent>(item).position = {0.f, 0.f, 400.f};
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_e);
    reg.emplace<factory::SpawnedItemComponent>(item).set_prototype(proto);

    // 5 steps × 1s × 200 mm/s = 1000 mm → item exits
    for (int i = 0; i < 5; ++i)
        factory::sim::step(scene, 1.0f);

    int total = 0;
    reg.view<factory::SinkComponent>().each(
        [&](const factory::SinkComponent& sk) { total += sk.received(); });
    REQUIRE_EQ(total, 1);
}

// ── sim::step: belt-to-belt transfer ─────────────────────────────────────────

// Place one item manually on belt A. After 5s it transfers to belt B.
TEST(item_transfers_to_next_belt) {
    factory::FactoryScene scene;

    auto belt_a  = scene.add_belt(300, 1000, 400, 200.f, {1.f, 0.f, 0.f});
    auto entry_a = scene.add_port("entry_a",  {0.f,    0.f, 400.f});
    auto exit_a  = scene.add_port("exit_a", {1000.f, 0.f, 400.f});

    auto belt_b  = scene.add_belt(300, 2000, 400, 200.f, {1.f, 0.f, 0.f});
    auto entry_b = scene.add_port("entry_b",  {1000.f, 0.f, 400.f});
    auto exit_b  = scene.add_port("exit_b", {3000.f, 0.f, 400.f});

    scene.connect_belt(belt_a, entry_a, exit_a);
    scene.connect_belt(belt_b, entry_b, exit_b);
    scene.set_port_transport(exit_a,  belt_b);   // A's exit hands off to B
    scene.set_port_transport(entry_b, belt_b);

    auto& reg   = scene.registry();
    auto  proto = scene.add_prototype(300, 300, 300, 0x00FF00u);
    auto  item  = reg.create();
    reg.emplace<factory::PoseComponent>(item).position = {0.f, 0.f, 400.f};
    reg.emplace<factory::ItemOnTransportComponent>(item).set_transport(belt_a);
    reg.emplace<factory::SpawnedItemComponent>(item).set_prototype(proto);

    // 5 steps × 1s × 200 mm/s = 1000 mm → exits belt A, snaps to entry_b position
    for (int i = 0; i < 5; ++i)
        factory::sim::step(scene, 1.0f);

    REQUIRE_EQ(scene.items_on(belt_a), 0);
    REQUIRE_EQ(scene.items_on(belt_b), 1);
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main() {
    printf("Running sim tests...\n");

    // Component setters
    RUN(conveyor_belt_setters_store_values);
    RUN(conveyor_belt_zero_speed_is_valid);
    RUN(transport_component_defaults);
    RUN(transport_capacity_zero_is_unlimited);
    RUN(source_rate_round_trip);
    RUN(source_zero_rate_is_valid);
    RUN(declared_opening_width_round_trip);
    RUN(declared_opening_mobility_round_trip);
    RUN(item_prototype_setters);

    // spawn_debt
    RUN(spawn_debt_accumulates);
    RUN(consume_spawn_decrements_by_one);
    RUN(consume_spawn_twice);

    // items_on / capacity
    RUN(items_on_returns_zero_for_empty_belt);
    RUN(transport_has_capacity_when_unlimited);
    RUN(transport_at_capacity_returns_false);
    RUN(transport_not_running_has_no_capacity);

    // sim::step – spawning
    RUN(step_spawns_item_when_debt_reaches_one);
    RUN(spawned_item_has_correct_prototype);
    RUN(spawned_item_starts_at_entry_port);
    RUN(no_spawn_when_debt_below_one);
    RUN(no_spawn_when_belt_at_capacity);

    // sim::step – movement
    RUN(item_moves_along_belt_direction);
    RUN(stopped_transport_does_not_move_items);

    // sim::step – despawn & sink
    RUN(item_despawns_at_terminal_exit);
    RUN(sink_increments_received_on_despawn);

    // sim::step – transfer
    RUN(item_transfers_to_next_belt);

    printf("\n%d/%d passed", g_run - g_fail, g_run);
    if (g_fail) printf("  (%d FAILED)", g_fail);
    printf("\n");
    return g_fail ? 1 : 0;
}

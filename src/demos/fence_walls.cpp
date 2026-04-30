#include "common/scene_setup.hpp"

#include <threepp/loaders/OBJLoader.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace threepp;
using json = nlohmann::json;

// ── Cell parameters ───────────────────────────────────────────────────────────
constexpr int  kNorthSouthMm = 4000;
constexpr int  kEastWestMm   = 3000;
constexpr bool kPreferOver   = true;   // true = nearest ≥ target, false = nearest ≤

// ── Lookup types ──────────────────────────────────────────────────────────────
struct LookupResult {
    int              actual_mm;
    std::vector<int> panels_mm;
};

using LookupTable = std::map<int, std::vector<int>>;

static LookupTable loadTable(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open: " + path);
    json j = json::parse(f);
    LookupTable table;
    for (const auto& e : j["entries"])
        table[e["total_mm"].get<int>()] = e["panels_mm"].get<std::vector<int>>();
    return table;
}

static LookupResult lookup(const LookupTable& table, int target_mm, bool prefer_over) {
    if (prefer_over) {
        auto it = table.lower_bound(target_mm);   // first key >= target
        if (it == table.end()) --it;              // clamp if nothing >=
        return {it->first, it->second};
    } else {
        auto it = table.upper_bound(target_mm);   // first key > target
        if (it != table.begin()) --it;            // step back to <= target
        else ++it;                                // nothing <=, use smallest
        return {it->first, it->second};
    }
}

static LookupResult solve(const LookupTable& table, int desired_mm, bool prefer_over) {
    std::vector<int> prefix;
    while (desired_mm > 2100) {
        prefix.push_back(1000);
        desired_mm -= 1050;   // 1000mm panel + 50mm post
    }
    auto tail = lookup(table, desired_mm, prefer_over);
    prefix.insert(prefix.end(), tail.panels_mm.begin(), tail.panels_mm.end());
    std::sort(prefix.begin(), prefix.end(), std::greater<int>());   // largest first

    int n = static_cast<int>(prefix.size());
    int actual = -50;
    for (int w : prefix) actual += w + 50;   // sum(widths) + (n-1)*50  [= sum(w+50) - 50]
    if (n == 0) actual = 0;
    return {actual, prefix};
}

// ── OBJ helpers ───────────────────────────────────────────────────────────────

static float catalogScale(const json& catalog) {
    return catalog["scale"][0].get<float>();
}

// Build panel prototypes using scale from catalog. Panels are Y-up, centered at origin.
static std::map<int, std::shared_ptr<Object3D>> buildPanelProtos(
    OBJLoader& loader,
    const std::string& dir,
    const json& catalog)
{
    const float s = catalogScale(catalog);
    auto mat = MeshPhongMaterial::create();
    mat->color = Color(0xa8a8a8);

    std::map<int, std::shared_ptr<Object3D>> protos;
    for (const auto& p : catalog["panels"]) {
        int w         = p["width_mm"].get<int>();
        std::string f = dir + "/" + p["filename"].get<std::string>();
        auto grp = loader.load(f);
        if (!grp) { std::cerr << "Failed to load: " << f << "\n"; continue; }
        grp->scale.set(s, s, s);
        grp->traverseType<Mesh>([&](Mesh& m) { m.setMaterial(mat); });
        protos[w] = grp;
    }
    return protos;
}

// Build post prototype from OBJ using catalog properties:
//   origin_offset     — raw world offset baked into the OBJ export (subtract to centre)
//   coordinate_system — "z_up" rotates -90° around X to convert to Y-up
//   scale             — top-level catalog scale (mm → m)
static std::shared_ptr<Object3D> buildPostProto(
    OBJLoader& loader,
    const std::string& dir,
    const json& catalog)
{
    const json& post = catalog["post"];
    const float s    = catalogScale(catalog);

    std::string f = dir + "/" + post["filename"].get<std::string>();
    auto raw = loader.load(f);
    if (!raw) throw std::runtime_error("Failed to load post: " + f);

    // Subtract the baked-in world offset so the post is centred at the local origin.
    auto off = post.value("origin_offset", json::array({0.0, 0.0, 0.0}));
    raw->position.set(
        -off[0].get<float>(),
        -off[1].get<float>(),
        -off[2].get<float>());

    auto mat = MeshPhongMaterial::create();
    mat->color = Color(0x707070);
    raw->traverseType<Mesh>([&](Mesh& m) { m.setMaterial(mat); });

    auto proto = Group::create();
    proto->add(raw);
    proto->scale.set(s, s, s);

    // Rotate to Y-up if the OBJ was exported in Z-up space.
    if (post.value("coordinate_system", "y_up") == "z_up")
        proto->rotation.x = -math::PI / 2.0f;

    return proto;
}

// Build one wall group (X-axis aligned). Caller repositions/rotates for each side.
static std::shared_ptr<Object3D> buildWall(
    const std::vector<int>& panels_mm,
    int height_mm,
    int y_offset_mm,
    const std::map<int, std::shared_ptr<Object3D>>& panelProtos,
    const std::shared_ptr<Object3D>& postProto)
{
    const int n = static_cast<int>(panels_mm.size());

    int visual_mm = -50;
    for (int w : panels_mm) visual_mm += w + 50;

    auto wall = Group::create();
    float cursor  = visual_mm * -0.0005f;
    float lift_y  = (height_mm * 0.5f + y_offset_mm) * 0.001f;

    // corner post at start
    {
        auto post = postProto->clone();
        post->position.set(cursor - 0.025f, 0.0f, 0.0f);
        wall->add(post);
    }

    for (int i = 0; i < n; ++i) {
        int w = panels_mm[i];

        auto panel = panelProtos.at(w)->clone();
        panel->position.set(cursor + w * 0.0005f, lift_y, 0.0f);
        wall->add(panel);

        cursor += w * 0.001f;

        auto post = postProto->clone();
        post->position.set(cursor + 0.025f, 0.0f, 0.0f);
        wall->add(post);
        cursor += 0.050f;
    }
    return wall;
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main() {
    const std::string assetDir = "assets/components/fences/axelent_x-guard";

    json catalog;
    {
        std::ifstream f(assetDir + "/catalog.json");
        if (!f) throw std::runtime_error("Cannot open catalog.json");
        catalog = json::parse(f);
    }
    const int panelH  = catalog["panels"][0]["height_mm"].get<int>();
    const int yOffset = catalog["panel_y_offset_mm"].get<int>();

    auto table = loadTable(assetDir + "/combinations.json");

    auto ns = solve(table, kNorthSouthMm, kPreferOver);
    auto ew = solve(table, kEastWestMm,   kPreferOver);

    std::cout << "N/S wall: " << ns.actual_mm << " mm ("
              << ns.panels_mm.size() << " panels)\n";
    std::cout << "E/W wall: " << ew.actual_mm << " mm ("
              << ew.panels_mm.size() << " panels)\n";

    // ── Scene ─────────────────────────────────────────────────────────────────
    SceneSetup ss("FenceWalls");
    ss.scene->background = Color(0x1a1f2e);
    ss.camera->position.set(7.0f, 5.0f, 9.0f);
    ss.camera->lookAt({0, 0.5f, 0});
    ss.controls->target = {0, 0.5f, 0};
    ss.controls->update();

    ss.scene->add(AmbientLight::create(0xffffff, 0.4f));
    {
        auto sun = DirectionalLight::create(0xffffff, 0.8f);
        sun->position.set(5, 10, 7);
        ss.scene->add(sun);
    }

    // Floor
    {
        float fw = ns.actual_mm * 0.001f;
        float fd = ew.actual_mm * 0.001f;
        auto geo = PlaneGeometry::create(fw, fd);
        auto mat = MeshPhongMaterial::create();
        mat->color = Color(0x2a2a2a);
        auto floor = Mesh::create(geo, mat);
        floor->rotation.x = -math::PI / 2.0f;
        ss.scene->add(floor);
    }

    // ── OBJ prototypes ────────────────────────────────────────────────────────
    OBJLoader loader;
    auto panelProtos = buildPanelProtos(loader, assetDir, catalog);
    auto postProto   = buildPostProto(loader, assetDir, catalog);

    // ── Walls ─────────────────────────────────────────────────────────────────
    // hw: half the N/S wall visual length → X position of E/W walls
    // hd: half the E/W wall visual length → Z position of N/S walls
    const float hw = ns.actual_mm * 0.0005f;
    const float hd = ew.actual_mm * 0.0005f;

    auto north = buildWall(ns.panels_mm, panelH, yOffset, panelProtos, postProto);
    north->position.set(0, 0, -hd);
    ss.scene->add(north);

    auto south = buildWall(ns.panels_mm, panelH, yOffset, panelProtos, postProto);
    south->position.set(0, 0, +hd);
    ss.scene->add(south);

    auto east = buildWall(ew.panels_mm, panelH, yOffset, panelProtos, postProto);
    east->position.set(+hw, 0, 0);
    east->rotation.y = math::PI / 2.0f;
    ss.scene->add(east);

    auto west = buildWall(ew.panels_mm, panelH, yOffset, panelProtos, postProto);
    west->position.set(-hw, 0, 0);
    west->rotation.y = math::PI / 2.0f;
    ss.scene->add(west);

    // ── Render loop ───────────────────────────────────────────────────────────
    ss.canvas.animate([&] {
        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}

#pragma once
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include <threepp/loaders/OBJLoader.hpp>
#include <threepp/materials/MeshPhongMaterial.hpp>
#include <threepp/math/MathUtils.hpp>
#include <threepp/objects/Group.hpp>
#include <threepp/objects/Mesh.hpp>

namespace cell {

// All OBJ prototypes for one fence catalog (panels keyed by width_mm, plus post).
struct CatalogProtos {
    std::map<int, std::shared_ptr<threepp::Object3D>> panels;
    std::shared_ptr<threepp::Object3D> post;
};

inline nlohmann::json loadCatalog(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open catalog: " + path);
    return nlohmann::json::parse(f);
}

// Shared transform logic for any catalog component.
// Catalog fields consumed:
//   origin_offset  [x,y,z]  additive translation of the raw mesh (mm, before scale)
//   scale          [s,s,s]  mm → metres
//   up_axis        [x,y,z]  OBJ up direction; [0,0,1] = Z-up → rotate -90° around X
inline std::shared_ptr<threepp::Object3D> buildComponentProto(
    threepp::OBJLoader& loader,
    const std::string& path,
    const nlohmann::json& entry,
    const std::shared_ptr<threepp::Material>& mat)
{
    using namespace threepp;
    auto raw = loader.load(path);
    if (!raw) throw std::runtime_error("Failed to load: " + path);

    auto off = entry.value("origin_offset", nlohmann::json::array({0.0, 0.0, 0.0}));
    raw->position.set(off[0].get<float>(), off[1].get<float>(), off[2].get<float>());
    raw->traverseType<Mesh>([&](Mesh& m) { m.setMaterial(mat); });

    float s = entry["scale"][0].get<float>();
    auto proto = Group::create();
    proto->add(raw);
    proto->scale.set(s, s, s);

    auto up = entry.value("up_axis", nlohmann::json::array({0, 1, 0}));
    if (up[2].get<float>() > 0.5f)
        proto->rotation.x = -math::PI / 2.0f;

    return proto;
}

inline CatalogProtos loadCatalogProtos(
    threepp::OBJLoader& loader,
    const std::string& dir,
    const nlohmann::json& catalog)
{
    using namespace threepp;
    CatalogProtos protos;

    auto panelMat = MeshPhongMaterial::create();
    panelMat->color = Color(0xa8a8a8);
    for (const auto& p : catalog["panels"]) {
        int w = p["width_mm"].get<int>();
        protos.panels[w] = buildComponentProto(
            loader, dir + "/" + p["filename"].get<std::string>(), p, panelMat);
    }

    auto postMat = MeshPhongMaterial::create();
    postMat->color = Color(0x707070);
    const auto& post = catalog["post"];
    protos.post = buildComponentProto(
        loader, dir + "/" + post["filename"].get<std::string>(), post, postMat);

    return protos;
}

} // namespace cell

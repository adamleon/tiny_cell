// Phase-0 Vulkan render spike (post-MVP sales-demo track — see HANDOFF.md).
//
// SUCCESS = this standalone exe builds inside tiny_cell's vcpkg + FetchContent
// tree with threepp's Vulkan deferred-hybrid backend, opens a window on the dev
// machine's RTX 3090, and renders a lit box on a floor with an orbiting camera.
// Its only job is to retire the riskiest integration unknown — threepp + Vulkan
// coexisting with tiny_cell's build — BEFORE the real render/ leg is built. It
// deliberately carries no tiny_cell types yet.
//
//   Interactive:              tinycell_render_spike
//   Headless smoke (a PNG):   tinycell_render_spike --frames 90 --out spike.png

#include "threepp/threepp.hpp"

#include "threepp/lights/AmbientLight.hpp"
#include "threepp/lights/DirectionalLight.hpp"
#include "threepp/renderers/VulkanRenderer.hpp"

#include <string>

using namespace threepp;

int main(int argc, char** argv) {
    // Optional headless smoke-test flags so the spike can be verified without a
    // human watching the window: render N frames, save one PNG, then quit.
    int         framesToRender = -1;// -1 = run interactively until the window closes
    std::string outPng         = "render_spike.png";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--frames" && i + 1 < argc) framesToRender = std::stoi(argv[++i]);
        else if (a == "--out" && i + 1 < argc) outPng = argv[++i];
    }

    Canvas canvas("tiny_cell - Vulkan render spike",
                  {{"vsync", true}, {"size", WindowSize{1280, 800}}});
    VulkanRenderer renderer(canvas);

    Scene scene;
    scene.background = Color(0.83f, 0.88f, 1.0f);// soft sky, placeholder look

    // Floor plane (threepp's PlaneGeometry faces +Z; rotate to lie in the XZ plane).
    auto floor = Mesh::create(
            PlaneGeometry::create(10.f, 10.f),
            MeshStandardMaterial::create(MeshStandardMaterial::Params{}.color(Color(0.80f, 0.80f, 0.82f)).roughness(0.95f)));
    floor->rotation.x = -math::PI / 2.f;
    scene.add(floor);

    // A single box standing on the floor — the "hello cell" stand-in.
    auto box = Mesh::create(
            BoxGeometry::create(1.f, 1.f, 1.f),
            MeshStandardMaterial::create(MeshStandardMaterial::Params{}.color(Color(0.95f, 0.45f, 0.2f)).roughness(0.5f)));
    box->position.set(0.f, 0.5f, 0.f);
    scene.add(box);

    // Warm key light + a soft ambient fill.
    auto key = DirectionalLight::create(Color(1.0f, 0.9f, 0.75f), 3.0f);
    key->position.set(4.f, 6.f, 3.f);
    scene.add(key);
    scene.add(AmbientLight::create(Color::white, 0.25f));

    PerspectiveCamera camera(50.f, canvas.aspect(), 0.1f, 100.f);
    camera.position.set(4.f, 3.f, 5.f);
    OrbitControls controls{camera, canvas};
    controls.target.set(0.f, 0.5f, 0.f);
    controls.update();

    canvas.onWindowResize([&](WindowSize s) {
        renderer.setSize(s);
        camera.aspect = canvas.aspect();
        camera.updateProjectionMatrix();
    });

    int frame = 0;
    canvas.animate([&] {
        controls.update();
        renderer.render(scene, camera);
        if (framesToRender > 0 && ++frame >= framesToRender) {
            renderer.writeFramebuffer(outPng);// save proof-of-render, then quit
            canvas.close();
        }
    });

    return 0;
}

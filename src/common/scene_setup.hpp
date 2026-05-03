#pragma once

#include <threepp/threepp.hpp>
#include <threepp/controls/OrbitControls.hpp>
#include <threepp/renderers/WgpuRenderer.hpp>
#include <memory>
#include <string>

struct SceneSetup {
    threepp::Canvas                             canvas;
    threepp::WgpuRenderer                       renderer;
    std::shared_ptr<threepp::Scene>             scene;
    std::shared_ptr<threepp::PerspectiveCamera> camera;
    std::unique_ptr<threepp::OrbitControls>     controls;

    explicit SceneSetup(const std::string& title)
        : canvas(title)
        , renderer(canvas)
        , scene(threepp::Scene::create())
        , camera(threepp::PerspectiveCamera::create(50, canvas.aspect(), 0.1f, 200))
    {
        renderer.outputColorSpace = threepp::ColorSpace::sRGB;
        renderer.toneMapping      = threepp::ToneMapping::ACESFilmic;
        controls = std::make_unique<threepp::OrbitControls>(*camera, canvas);
        canvas.onWindowResize([this](threepp::WindowSize sz) {
            camera->aspect = sz.aspect();
            camera->updateProjectionMatrix();
            renderer.setSize(sz);
        });
    }
};

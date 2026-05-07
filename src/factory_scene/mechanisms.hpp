#pragma once
#include <entt/entt.hpp>

namespace factory {

class IMechanism {
public:
    virtual ~IMechanism() = default;
    virtual void start(entt::entity item, entt::registry& reg) = 0;
    virtual void tick(float dt, entt::registry& reg) = 0;
    virtual bool done() const = 0;
    virtual void reset() = 0;
};

class ProcessMechanism : public IMechanism {
public:
    explicit ProcessMechanism(float duration_s) : duration_s_(duration_s) {}

    void start(entt::entity, entt::registry&) override {
        elapsed_ = 0.f;
        done_    = false;
    }

    void tick(float dt, entt::registry&) override {
        elapsed_ += dt;
        done_     = elapsed_ >= duration_s_;
    }

    bool done() const override { return done_; }

    void reset() override {
        elapsed_ = 0.f;
        done_    = false;
    }

private:
    float duration_s_;
    float elapsed_ = 0.f;
    bool  done_    = false;
};

}  // namespace factory

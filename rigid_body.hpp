#pragma once

#include "glm/ext/quaternion_geometric.hpp"
#include "glm/matrix.hpp"
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <array>
#include <cassert>
#include <cstdint>

using RBState = std::array<float, 13>;

RBState::iterator serialize(glm::vec3 item, RBState::iterator &begin);
RBState::iterator serialize(glm::quat item, RBState::iterator &begin);
RBState::const_iterator deserialize(glm::vec3 &item, RBState::const_iterator &begin);
RBState::const_iterator deserialize(glm::quat &item, RBState::const_iterator &begin);

struct RigidBody {
    uint64_t id;
    glm::vec3 x; // position
    glm::quat q; // rotation
    glm::vec3 p; // linear momentem
    glm::vec3 L; // angular momentem

    glm::vec3 v; // linear velocity
    glm::vec3 tau; // torque
    glm::vec3 F; // total force
    glm::vec3 omega; // angular velocity

    glm::mat3 I0; // inertia tensor
    float M; // total mass
    glm::mat3 I;

    RBState to_state() const;
    void load_state(RBState &&state);
    RBState to_dstate() const;
    void update_I();

    RigidBody(uint64_t id, RBState &&state) : id(id) {
        load_state(std::move(state));
    }
    RigidBody(uint64_t id, glm::vec3 x, glm::quat q, glm::vec3 p, glm::vec3 L, glm::mat3 I0, float M) 
        : id(id), x(x), q(q), p(p), L(L), I0(I0), M(M)
    {
        v = p / M;
        tau = glm::vec3(.0f);
        F = glm::vec3(.0f);
        update_I();
        omega = glm::inverse(I) * L;
    }
    RigidBody() = default;
};

void RigidBody::update_I() {
    glm::mat3 R = glm::toMat3(q);
    I = R * I0 * glm::transpose(R);
}

RBState::iterator serialize(glm::vec3 item, RBState::iterator &begin) {
    *begin++ = item[0];
    *begin++ = item[1];
    *begin++ = item[2];
    return begin;
}

RBState::iterator serialize(glm::quat item, RBState::iterator &begin) {
    *begin++ = item[0];
    *begin++ = item[1];
    *begin++ = item[2];
    *begin++ = item[3];
    return begin;
}

RBState::const_iterator deserialize(glm::vec3 &item, RBState::const_iterator &begin) {
    item[0] = *begin++;
    item[1] = *begin++;
    item[2] = *begin++;
    return begin;
}

RBState::const_iterator deserialize(glm::quat &item, RBState::const_iterator &begin) {
    item[0] = *begin++;
    item[1] = *begin++;
    item[2] = *begin++;
    item[3] = *begin++;
    return begin;
}

RBState RigidBody::to_state() const {
    RBState state;
    auto begin = state.begin();

    serialize(x, begin);
    serialize(q, begin);
    serialize(p, begin);
    serialize(L, begin);

    assert(begin == state.end());

    return std::move(state);
}

void RigidBody::load_state(RBState &&state) {
    RBState st = state;
    auto begin = state.cbegin();
    deserialize(x, begin);
    deserialize(q, begin);
    deserialize(p, begin);
    deserialize(L, begin);

    assert(begin == state.cend());

    q = glm::normalize(q);
    update_I();

    v = p / M;
    omega = glm::inverse(I) * L;
}

RBState RigidBody::to_dstate() const {
    RBState state;
    auto begin = state.begin();

    glm::quat dq = glm::quat(0, omega) * q * 0.5f;

    serialize(v, begin);
    serialize(dq, begin);
    serialize(F, begin);
    serialize(tau, begin);

    assert(begin == state.end());

    return std::move(state);
}
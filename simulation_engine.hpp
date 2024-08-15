#pragma once

#include "rigid_body.hpp"
#include "ode_solver.hpp"
#include "contact.hpp"
#include <glm/glm.hpp>
#include <unordered_map>
#include <vector>
#include <memory>
#include <cstdint>

struct Force {
    glm::vec3 F;
    glm::vec3 x;
    uint64_t rb_id;
};

struct IDPair {
    uint64_t a_id, b_id;
    IDPair(uint64_t a_id, uint64_t b_id) : a_id(a_id), b_id(b_id) {

    }

    bool operator==(const IDPair &oth) {
        return a_id == oth.a_id && b_id == oth.b_id;
    }
};

template <>
struct std::hash<IDPair>
{
  std::size_t operator()(const IDPair& k) const
  {
    using std::size_t;
    using std::hash;
    using std::string;

    // Compute individual hash values for first,
    // second and third and combine them using XOR
    // and bit shifting:
    return hash<uint64_t>()(k.a_id) ^ (hash<uint64_t>()(k.b_id) << 1);
  }
};


struct Engine {
    std::unordered_map<uint64_t, RigidBody> bodies;
    std::shared_ptr<ODESolver<float, 13>> solver_ref;
    std::vector<Force> forces;
    float t; // time

    std::vector<Contact> contacts;
    std::unordered_map<IDPair, Witness> witnesses;

    void step(float step = 0.01);
    void update_forces_and_torques();
    uint64_t add_body(RBState &&rbstate);
    uint64_t add_body(glm::vec3 x, glm::quat q, glm::vec3 p, glm::vec3 L, glm::mat3 I0, float M);
    void add_force(Force &&force);

    Engine(std::shared_ptr<ODESolver<float, 13>> solver_ref) : solver_ref(solver_ref) {
        
    }

private:
    uint64_t id_counter = 0;

    std::shared_ptr<Witness> find_witness(const RigidBody& a, const RigidBody &b);
    void find_all_contacts();
    void resolve_collision(RigidBody& a, RigidBody& b);
};

// uint64_t Engine::add_body(RigidBody &&body) {
//     body.id = ++id_counter;
//     bodies.emplace(id_counter, body);
//     return id_counter;
// }

uint64_t Engine::add_body(RBState &&rbstate) {
    ++id_counter;
    bodies.emplace(id_counter, RigidBody(id_counter, std::move(rbstate)));
    return id_counter;
}

uint64_t Engine::add_body(glm::vec3 x, glm::quat q, glm::vec3 p, glm::vec3 L, glm::mat3 I0, float M) {
    ++id_counter;
    bodies.emplace(id_counter, RigidBody(id_counter, x, q, p, L, I0, M));
    return id_counter;
}

void Engine::add_force(Force &&force) {
    forces.push_back(force);
}

void Engine::update_forces_and_torques() {
    // clear all forces and torques
    for (auto &item : bodies) {
        item.second.F = glm::vec3(.0f);
        item.second.tau = glm::vec3(.0f);
    }
    // exert forces to bodies
    for (const Force &force : forces) {
        if (bodies.find(force.rb_id) != bodies.end()) {
            bodies[force.rb_id].F += force.F;
            glm::vec3 torque = glm::cross((force.x - bodies[force.rb_id].x), force.F);
            bodies[force.rb_id].tau += torque;
        }
    }
}

void Engine::step(float step) {
    update_forces_and_torques();
    for (auto &item : bodies) {
        item.second.update_I();

        auto x0 = item.second.to_state();
        auto x_dot = item.second.to_dstate();
        RBState x1;

        solver_ref->solve(x0, x1, t, t + step, x_dot);

        item.second.load_state(std::move(x1));
    }
    forces.clear();
}

std::shared_ptr<Witness> Engine::find_witness(const RigidBody& a, const RigidBody& b) {
    return nullptr;
}

void Engine::find_all_contacts() {

}

void Engine::resolve_collision(RigidBody& a, RigidBody& b) {

}
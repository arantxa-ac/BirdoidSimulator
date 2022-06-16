#pragma once

#include "givr.h"

#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

#include "models.h"

using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

namespace simulation {

//
// Particle Model
//
template <typename View>
void render(ParticleModel const &model, View const &view) {
  // static: only initiallized once, the first time this function is called.
  // This isn't the most elegant method to do this but it works. Just put your
  // draw calls in here.

  //boids

  static auto point_renders = createInstancedRenderable(
      Sphere(Radius(0.25f)), // geometry
      Phong(Colour(1.f, 0.4f, 0.2f),
            LightPosition(100.f, 100.f, 100.f)) // style
  );

  auto const &particles = model.particles;

  for (auto const &particle : particles) {
    auto M = translate(mat4f{1.f}, particle.x);
    addInstance(point_renders, M);
  }

  //obstacles

  static auto obst_renders = createInstancedRenderable(
      Sphere(Radius(1.f)), // geometry
      Phong(Colour(1.f, 0.f, 1.f),
          LightPosition(100.f, 100.f, 100.f)) // style
  );

  auto const& obstacles = model.obstacles;

  for (auto const& obstacle : obstacles) {
      auto M = translate(mat4f{ 1.f }, obstacle.x);
      M = scale(M, vec3f{ obstacle.r });
      addInstance(obst_renders, M);
  }

  //// planes
  //static auto plane_renders = createInstancedRenderable(
  //    Triangle(Point1(-10.f, 20.f, -20.f), Point2(-20.f, -20.f, -20.f), Point3(-20.f, -20.f, 20.f)),
  //    Phong(Colour(0.f, 1.f, 0.f), LightPosition(100.f, 100.f, 100.f)))
  //)

  // draw the renderable
  draw(point_renders, view);
  draw(obst_renders, view);
}

//
// Helper class/functions
//
template <typename View> struct RenderableModel {

  template <typename Model>
  RenderableModel(Model const &model) : m_self(new model_t<Model>(model)) {}

  friend void render(RenderableModel const &renderable, View const &view) {
    renderable.m_self->renderSelf(view);
  }

  struct concept_t {
    virtual ~concept_t() = default;
    virtual void renderSelf(View const &view) const = 0;
  };

  template <typename Model> struct model_t : public concept_t {
    model_t(Model const &model) : data(model) {}
    void renderSelf(View const &view) const override { render(data, view); }
    Model const &data;
  };

  std::shared_ptr<concept_t const> m_self;
};

template <typename Model, typename View>
RenderableModel<View> makeModelRenderable(Model const &model,
                                          View const &view) {
  return RenderableModel<View>(model);
}

} // namespace simulation

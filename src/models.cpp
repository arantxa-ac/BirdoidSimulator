#include "models.h"

#define CELLSW 10
#define CELLSH 10
#define CELLSD 10

namespace simulation {

//
// Particle Model
//
ParticleModel::ParticleModel() { reset(); }

void ParticleModel::reset() {
  particles.clear();
  planes.clear();
  obstacles.clear();

  // setup particles
  for (int i = -15; i <= 15; ++i) {
    int vx, vy, vz;
    vec3f velocities;
    for (int j = 0; j < 3; j++) {
        int random = rand() % 100;
        if (random > 50) velocities[j] = 1;
        else velocities[j] = -1;
    }
    vx = velocities[0];
    vy = velocities[1];
    vz = velocities[2];

    particles.push_back(Particle({i, i, i}, {vx, vy, vz}));
    particles.push_back(Particle({-i, -i, -i }, { -vx, -vy, -vz }));
  }

  //setup planes
  //left
  planes.push_back(Plane({ -25.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }));
  //right
  planes.push_back(Plane({ 25.f, 0.f, 0.f }, { -1.f, 0.f, 0.f }));
  //ceiling
  planes.push_back(Plane({ 0.f, 25.f, 0.f }, { 0.f, -1.f, 0.f }));
  //floor
  planes.push_back(Plane({ 0.f, -25.f, 0.f }, { 0.f, 1.f, 0.f }));
  //front
  planes.push_back(Plane({ 0.f, 0.f, -5.f }, { 0.f, 0.f, 1.f }));
  //back
  planes.push_back(Plane({ 0.f, 0.f, 20.f }, { 0.f, 0.f, -1.f }));

  //setup obstacles
  obstacles.push_back(Obstacle({ 5.f, -2.f, 20.f }, 1.f));
  obstacles.push_back(Obstacle({ -20.f, 5.f, -20.f }, 5.f));
  obstacles.push_back(Obstacle({ -6.f, -17.f, -3.f }, 3.f));
  obstacles.push_back(Obstacle({ -12.f, 12.f, 12.f }, 2.f));
  obstacles.push_back(Obstacle({ 20.f, -20.f, -5.f }, 4.f));
  obstacles.push_back(Obstacle({ 15.f, 22.f, 5.f }, 5.f));
}

void ParticleModel::step(float dt, float ks, float kc, float ka) {
  
  // SPATIAL PARTITIONING FAIL
  // 
  // increment grid global time
  //grid.tg++;
  //for (int i = 0; i < particles.size(); i++) {
  //    vec3f index = getIndex(i);
  //    grid.insert(index, i);
  //}
  //for (int i = 0; i < particles.size(); i++) {
  //    vec3f index = getIndex(i);
  //    std::vector<Cell> Ni = grid.neighbourhood(index, i);
  //    for (auto& cell : Ni) {
  //        for (auto& j : cell.points) {
  //            if (!particles[i].equalTo(particles[j])) {
  //                // calculate forces for neighbor boids
  //                float rs = 3.f;
  //                float ra = 4.f;
  //                float rc = 5.f;
  //                float costheta = cos(3.14159f);
  //                for (auto& i : particles) {
  //                    for (auto& j : particles) {
  //                        if (!i.equalTo(j)) {
  //                            vec3f dx = j.x - i.x;
  //                            float d = length(dx);
  //                            float alpha = dot(normalize(dx), normalize(j.v - i.v));
  //                            if (d < rs && alpha > costheta)
  //                                i.F += separationForce(i, j, ks);
  //                            else if (d < ra && alpha > costheta)
  //                                i.F += alignmentForce(i, j, ka);
  //                            else if (d < rc && alpha > costheta)
  //                                i.F += cohessionForce(i, j, kc);
  //                        }
  //                    }
  //                    i.F += repulsionForce(i);
  //                }
  //            }
  //        }
  //    }
  //}

  // calculate forces for neighbor boids
  float rs = 3.f;
  float ra = 4.f;
  float rc = 5.f;
  float costheta = cos(3.14159f);
  for (auto& i : particles) {
      for (auto& j : particles) {
          if (!i.equalTo(j)) {
              vec3f dx = j.x - i.x;
              float d = length(dx);
              float alpha = dot(normalize(dx), normalize(j.v - i.v));
              if (d < rs && alpha > costheta)
                  i.F += separationForce(i, j, ks);
              else if (d < ra && alpha > costheta)
                  i.F += alignmentForce(i,j,ka);
              else if (d < rc && alpha > costheta)
                  i.F += cohessionForce(i, j, kc);
          }
      }
      i.F += repulsionForce(i);
  }

  //move particles
  for (auto& p : particles) {
      p.v += p.F * dt;
      p.x += p.v * dt;
      p.F = vec3f{ 0.f };
  }

}

vec3f ParticleModel::separationForce(Particle i, Particle j, float ks) {
    return -ks * (j.x - i.x) / (length(j.x - i.x) * length(j.x - i.x));
}

vec3f ParticleModel::cohessionForce(Particle i, Particle j, float kc) {
    return kc * (j.x - i.x);
}

vec3f ParticleModel::alignmentForce(Particle i, Particle j, float ka) {
    return ka * (j.v - i.v);
}

vec3f ParticleModel::repulsionForce(Particle i) {
    vec3f rForce = vec3f{ 0.f };
    for (auto& p : planes) {
        float r = dot((i.x - p.x), p.n);
        rForce += p.n * vec3f{ 5.f } / (r * r);
    }
    for (auto& o : obstacles) {
        float r = distance(i.x, (o.x + (normalize(i.x - o.x) * o.r)));
        rForce += normalize(i.x - o.x) * vec3f(2.f) / (r);
    }
    return rForce;
}

vec3f ParticleModel::getIndex(int i) {
    int w = particles[i].x.x * CELLSW / 50;
    int h = particles[i].x.y * CELLSH / 50;
    int d = particles[i].x.z * CELLSD / 25;
    return vec3f{ w, h, d };
}

} // namespace simulation

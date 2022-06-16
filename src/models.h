#pragma once

#include <glm/glm.hpp>

#include <vector>

#define CELLSW 10
#define CELLSH 10
#define CELLSD 10



namespace simulation {

using vec2f = glm::vec2;
using vec3f = glm::vec3;

struct Model {
  virtual ~Model() = default;
  virtual void reset() = 0;
  virtual void step(float dt, float ks, float kc, float ka) = 0;
};

struct Particle {
  explicit Particle(vec3f position) : x(position) {}
  Particle(vec3f position, vec3f velocity) : x(position), v(velocity) {}

  vec3f x;
  vec3f v = vec3f{0.f};
  vec3f F = vec3f{ 0.f };

  bool equalTo(Particle p) {
	  if (this->F == p.F && this->v == p.v && this->x == p.x)
		  return true;
	  else
		  return false;
  }
};

struct Plane {
	explicit Plane(vec3f position, vec3f normal) : x(position), n(normal) {};

	vec3f x;
	vec3f n;
};

struct Obstacle {
	explicit Obstacle(vec3f position, float radius) : x(position), r(radius) {};

	vec3f x;
	float r;
};

struct Cell {
	int tc = 0;
	std::vector<int> points;
};

class Grid {
public:
	int tg = 0;
	std::vector<Cell> cells;

	void insert(vec3f index, int p) {
		//get cell
		Cell cell = gridij(index.x, index.y, index.z);
		//insert p in the cell
		cell.points.push_back(p);
	}

	Cell gridij(int w, int h, int d) {
		int ia = w + CELLSW * h + CELLSW * CELLSH * d; //row major linearization
		Cell cell = cells[ia];
		if (cell.tc != this->tg) {
			cell.points.clear();
			cell.tc = this->tg;
		}
		return cell;
	}

	std::vector<Cell> neighbourhood(vec3f index, int p) {
		std::vector<Cell> np;
		np.push_back(gridij(index.x, index.y, index.z));
		if (index.x < CELLSW)
			np.push_back(gridij(index.x + 1, index.y, index.z));
		if (index.x > 0)
			np.push_back(gridij(index.x - 1, index.y, index.z));
		if (index.y < CELLSH)
			np.push_back(gridij(index.x, index.y + 1, index.z));
		if (index.y > 0)
			np.push_back(gridij(index.x, index.y - 1, index.z));
		if (index.z < CELLSD)
			np.push_back(gridij(index.x, index.y, index.z + 1));
		if (index.z > 0)
			np.push_back(gridij(index.x, index.y, index.z - 1));
		return np;
	}
};

//
// Particle Model
//
class ParticleModel : public Model {
public:
  ParticleModel();
  void reset() override;
  void step(float dt, float ks, float kc, float ka) override;

  vec3f separationForce(Particle i, Particle j, float ks);
  vec3f cohessionForce(Particle i, Particle j, float kc);
  vec3f alignmentForce(Particle i, Particle j, float ka);
  vec3f repulsionForce(Particle i);
  vec3f getIndex(int i);

public:
  std::vector<Particle> particles;
  std::vector<Plane> planes;
  std::vector<Obstacle> obstacles;
  Grid grid;
};

} // namespace simulation

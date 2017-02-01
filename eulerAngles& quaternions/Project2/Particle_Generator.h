// GLM Mathemtics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


// GLFW
#include <GLFW/glfw3.h>


#include "Model.h"
#include "Camera.h"
#include "Shader.h"
#include <vector>


#include <cstdlib>
#include <ctime>


#pragma once
struct Particle
{
	glm::vec3 position;
	glm::vec3 velocity;
	GLfloat mass;
	glm::vec3 force;
	vector<glm::vec3> forces;
	GLfloat life;
	glm::vec4 color;
	glm::vec3 dir;

	Particle()
		:position(0.0f),velocity(0.0f),mass(1.0f),force(0.0f),life(5.0f){

		GLfloat r1 = (((rand() % 100) + 1) / 100.0f) - .01f;
		GLfloat r2 = (((rand() % 100) + 1) / 100.0f) - .01f;
		GLfloat r3 = (((rand() % 100) + 1) / 100.0f) - .01f;

		this->color = glm::vec4(r1,r2,r3,1.0f);
	
	}
};


class CollisionObjectPlane
{
public:
	glm::vec3 point;
	glm::vec3 normal;

};

struct WorldPhysics
{
	glm::vec3 g;//gravity
	GLfloat k;//drag

	WorldPhysics(){}
	
	WorldPhysics(glm::vec3 gravity, GLfloat drag)
	{
		this->g = gravity;
		this->k = drag;
	}
	
};



class ParticleGenerator
{

public:
	Shader shader;
	GLuint amount;
	GLuint VAO;
	vector<Particle> particles;
	Model model;
	Camera camera;
	CollisionObjectPlane plane;
	WorldPhysics F;

	//constructor
	ParticleGenerator(Shader shader, GLuint amount,Model model);
	void init();
	void Update(GLfloat dt, glm::vec3 pos, GLuint newParticles, glm::vec3 offset);
	void respawnParticle(Particle &particle, glm::vec3 pos, glm::vec3 offset);
	void Draw();
	void SetWorldPhysics(glm::vec3 gravity, GLfloat drag);
	glm::vec3 Gravity(Particle p);
	glm::vec3 Drag(Particle p);

	// Stores the index of the last particle used (for quick access to next dead particle)
	GLuint lastUsedParticle = 0;
	GLuint firstUnusedParticle();

};




ParticleGenerator::ParticleGenerator(Shader shader, GLuint amount,Model model)
	: shader(shader), amount(amount),model(model)
{
	this->init();
}


void ParticleGenerator::SetWorldPhysics(glm::vec3 gravity, GLfloat drag) {

	this->F.g = gravity;
	this->F.k = drag;
}

glm::vec3 ParticleGenerator::Gravity(Particle p)
{
	return p.mass*F.g;
}

glm::vec3 ParticleGenerator::Drag(Particle p)
{
	return -F.k*p.velocity;
}


void ParticleGenerator::Update(GLfloat dt, glm::vec3 pos, GLuint newParticles, glm::vec3 offset)
{
	// Add new particles 
	for (GLuint i = 0; i < newParticles; ++i)
	{
		int unusedParticle = this->firstUnusedParticle();
		Particle &particle=this->particles[unusedParticle];
		particle.position = pos;
		particle.velocity = glm::vec3(0.0f);
		particle.life = 7.0f;
		particle.forces.push_back(particle.dir*10.0f);
	}

	// Update all particles
	for (GLuint i = 1; i < this->amount; ++i)
	{
		Particle &p = this->particles[i];
		p.life = p.life - dt; // reduce life
		if (p.life > 0.0f)
		{
		

			// particle is alive, thus update
			p.forces.push_back(Gravity(p));
			p.forces.push_back(Drag(p));
			


			//force accumulator
			p.force = glm::vec3(0.0f);
			for (int i = 0; i < p.forces.size(); i++)
			{
				p.force += p.forces[i];
			}
			if (glm::dot(glm::normalize((p.position - glm::vec3(0, -0.4, 0)) - plane.point), plane.normal) < 0.001 &&
				glm::dot(glm::normalize(p.velocity), plane.normal) < .001f)
			{
				//p.force += - glm::dot(p.force, plane.normal)*plane.normal;
				//p.velocity= -glm::dot(p.velocity, plane.normal)*plane.normal;
			}



			glm::vec3 acceleration = p.force / p.mass;
			p.velocity = p.velocity + (acceleration*dt);

			//check for collisions
			if (glm::dot(glm::normalize((p.position - glm::vec3(0, -0.4, 0)) - plane.point), plane.normal) < 0.001 &&
				glm::dot(glm::normalize(p.velocity), plane.normal) < .001f)
			{
				GLfloat mag = glm::distance(glm::vec3(0.0f), p.velocity);
				p.velocity = p.velocity + 1.5f *(plane.normal*mag);

//				p.velocity+
				//p.velocity = -glm::dot(p.velocity, plane.normal)*plane.normal;
			}
			p.position += p.velocity * dt;
			p.color.a += dt * 2.5;

			p.forces.clear();
			
		}
	}
}

void ParticleGenerator::respawnParticle(Particle &particle, glm::vec3 pos, glm::vec3 offset)
{
	particle.position = pos;
	particle.velocity = glm::vec3(0.0f);

	GLfloat r1 = (((rand() % 100) + 1) / 100.0f) - .01f;
	GLfloat r2 = (((rand() % 100) + 1) / 100.0f) - .01f;
	GLfloat r3 = (((rand() % 100) + 1) / 100.0f) - .01f;
	
	particle.color = glm::vec4(r1, r2, r3, .2f);
	particle.life = 7.0f;

	r1 = (((rand() % 200 ) + 1.0f) / 100.0f) - 1.0f;
	r2 = (((rand() % 200) + 1.0f) / 100.0f) - 1.0f;
	r3 = (((rand() % 200) + 1.0f) / 100.0f) - 1.0f;

	particle.forces.push_back(100.0f*glm::vec3(r1, r2, r3));

	printf("%f\n", 20.0f*r1);
}

// Stores the index of the last particle used (for quick access to next dead particle)
GLuint lastUsedParticle = 0;
GLuint ParticleGenerator::firstUnusedParticle()
{
	// First search from last used particle, this will usually return almost instantly
	for (GLuint i = lastUsedParticle; i < this->amount; ++i) {
		if (this->particles[i].life <= 0.0f) {
			lastUsedParticle = i;
			return i;
		}
	}
	// Otherwise, do a linear search
	for (GLuint i = 0; i < lastUsedParticle; ++i) {
		if (this->particles[i].life <= 0.0f) {
			lastUsedParticle = i;
			return i;
		}
	}
	// All particles are taken, override the first one (note that if it repeatedly hits this case, more particles should be reserved)
	lastUsedParticle = 0;
	return 0;
}


void ParticleGenerator::init()
{
	// Create this->amount default particle instances
	for (GLuint i = 0; i < this->amount; ++i)
	{
		this->particles.push_back(Particle());

		GLfloat r1 = (((rand() % 200)+1) / 100.0f)-1.0f;
		GLfloat r2 = (((rand() % 200)+1) / 100.0f)-1.0f;
		GLfloat r3 = (((rand() % 200)+1) / 100.0f)-1.0f;

		this->particles[i].forces.push_back(20.0f*glm::vec3(r1, 0.0f, r3));
		this->particles[i].dir = 10.0f*glm::vec3(r1, 0.0f, r3);
	}
}



// Render all particles
void ParticleGenerator::Draw()
{
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	this->shader.Use();
	for (Particle particle : this->particles)
	{
		if (particle.life > 0.0f)
		{
			//local view Position
			GLint viewPos = glGetUniformLocation(shader.Program, "viewPos");
			glUniform3f(viewPos, camera.Position.x, camera.Position.y, camera.Position.z);

			// Create camera transformations
			glm::mat4 view = camera.GetViewMatrix();
			glm::mat4 projection = glm::perspective(camera.Zoom, (GLfloat)800 / (GLfloat)600, 0.1f, 100.0f);

			// Get the uniform matrixes
			GLint modelLoc = glGetUniformLocation(shader.Program, "model");
			GLint viewLoc = glGetUniformLocation(shader.Program, "view");
			GLint projLoc = glGetUniformLocation(shader.Program, "projection");

			// Pass the matrices for view and projection
			glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
			glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

			glm::mat4 particleModel;
			particleModel = glm::scale(particleModel, glm::vec3(1.0f, 1.0f, 1.0f));
			glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(particleModel));


			glm::vec3 offset = glm::vec3(particle.position);
			glUniform3f(glGetUniformLocation(shader.Program, "offset"), (GLfloat)offset.x, (GLfloat)offset.y,(GLfloat)offset.z);


			GLfloat scale = .1f;
			glUniform1f(glGetUniformLocation(shader.Program, "scale"), scale);


			glm::vec4 color = particle.color;
			glUniform4f(glGetUniformLocation(shader.Program, "color"), (GLfloat)color.r, (GLfloat)color.g, (GLfloat)color.b, (GLfloat)color.a);

			glm::vec3 lightPos = glm::vec3(0.0f,20.0f,0.0f);
			glUniform3f(glGetUniformLocation(shader.Program, "lightPos"), (GLfloat)lightPos.x, (GLfloat)lightPos.y, (GLfloat)lightPos.z);


			model.Draw(shader);
		}
	}
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}





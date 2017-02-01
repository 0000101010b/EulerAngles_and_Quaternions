#include <GLFW/glfw3.h>

#include "Model.h"
#include "Camera.h"
#include "Shader.h"

// GLM Mathemtics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "glm/ext.hpp"

#include "Mesh.h"

#pragma once
struct RigidBody
{
	GLfloat mass;
	glm::vec3 position;
	glm::vec3 velocity;

	glm::mat3 orientation;


	//glm::vec3 angularVelocity;

	glm::vec3 force;
	glm::vec3 torque;
	
	glm::vec4 color;

	RigidBody()
		:position(0.0f), mass(1.0f), force(0.00000f),color(0.1f,0.5f,0.3f,1.0f) {
	}
};


class RigidBodyGenerator
{
public:
	glm::mat3 I_sphere;
	glm::mat3 w;
	RigidBody rigidbody;
	Model model;
	Shader shader;
	Camera camera;
	//constructor
	RigidBodyGenerator(Shader shader, Model model);
	void init();
	void Draw();
	void Update(GLfloat dt);
};


RigidBodyGenerator::RigidBodyGenerator(Shader shader, Model model)
	: shader(shader), model(model)
{
	this->init();
}



void RigidBodyGenerator::init()
{
	this->rigidbody = RigidBody();
	float R[9] = {
		glm::cos(.0f),-glm::sin(.0f),0,
		glm::sin(.0f),glm::cos(.0f),0,
		0,0,1
	};
	this->rigidbody.orientation = glm::make_mat3(R);


	float m = this->rigidbody.mass;
	float r = 1.0f;

	float Isphere[9] = {
		2.0f / 5.0f * m * r *r,0			  ,0,
		0				,2.0f / 5.0f *m*r*r,0,
		0				,0			  ,2.0f / 5.0f * m*r*r
	};
	this->I_sphere = glm::make_mat3(Isphere);



}

void RigidBodyGenerator::Update(GLfloat dt)
{

	//define axis of rotation

	

	glm::vec3 acceleration = this->rigidbody.force / this->rigidbody.mass;
	this->rigidbody.velocity += (acceleration*dt);
	//this->rigidbody.position += this->rigidbody.velocity;


	//get torque
	this->rigidbody.torque = glm::vec3(0.0f,0.0f,0.0f);
	for (int i = 0; i < model.meshes.size(); i++)
	{
		for (int j = 0; j < model.meshes[i].vertices.size(); j++)
		{
			Vertex &vertex = model.meshes[i].vertices[j];
			//float dist =glm::distance(vertex.Position ,this->rigidbody.position);
			if (j == 10) {
				cout << "pos" << endl;
				cout << glm::to_string(this->rigidbody.position) << endl;
				cout << "vertex" << endl;
				cout << glm::to_string(vertex.Position)<< endl;
				this->rigidbody.torque += (vertex.Position - this->rigidbody.position) * glm::vec3(5.0f, 0.0f, 0.0f);
				cout << "torque" << endl;
				cout << glm::to_string((vertex.Position - this->rigidbody.position) * glm::vec3(5.0f, 0.0f, 0.0f)) << endl;
			}
			this->rigidbody.torque += (vertex.Position - this->rigidbody.position) * glm::vec3(5.0f, 0.0f, 0.0f);
		}
	}

	cout << glm::to_string(this->rigidbody.torque) << endl;
	
	//glm::vec3 w = glm::inverse(I_sphere) * this->rigidbody.torque;
	/*
	std::cout << glm::to_string(this->I_sphere) << std::endl;
	
	glm::vec3 w = this->rigidbody.torque;
	
	float wf_matrix[9] = {
		0, -w.z,w.y,
		w.z,0,-w.x,
		-w.y,w.x,0
	};

	glm::mat3 wMat = glm::make_mat3(wf_matrix);
	this->rigidbody.orientation +=this->rigidbody.orientation*wMat;
	*/

	//re-normalize
	
	
	glm::mat3 R = this->rigidbody.orientation;
	glm::mat3 I = R * I_sphere * glm::transpose(R);
	glm::vec3 w = glm::inverse(I)*this->rigidbody.torque;


	float wf_matrix[9] = {
		0, -w.z,w.y,
		w.z,0,-w.x,
		-w.y,w.x,0
	};

	glm::mat3 wMat = glm::make_mat3(wf_matrix);

	this->rigidbody.orientation += this->rigidbody.orientation*wMat;
	
	//apply rotation
	for (int i = 0; i < model.meshes.size(); i++)
	{
		for (int j = 0; j < model.meshes[i].vertices.size(); j++)
		{
			Vertex &vertex = model.meshes[i].vertices[j];
			vertex.Position =this->rigidbody.position +(this->rigidbody.orientation*vertex.Position);
		}

		

		model.meshes[i].setupMesh();
	}
	
	//this->rigidbody.position += this->rigidbody.velocity;
}

void RigidBodyGenerator::Draw() {

	this->shader.Use();

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


	glm::vec3 offset = glm::vec3(0.0f);
		//glm::vec3(this->rigidbody.position);
	glUniform3f(glGetUniformLocation(shader.Program, "offset"), (GLfloat)offset.x, (GLfloat)offset.y, (GLfloat)offset.z);


	GLfloat scale = 1.0f;
	glUniform1f(glGetUniformLocation(shader.Program, "scale"), scale);


	glm::vec4 color = this->rigidbody.color;
	glUniform4f(glGetUniformLocation(shader.Program, "color"), (GLfloat)color.r, (GLfloat)color.g, (GLfloat)color.b, (GLfloat)color.a);

	glm::vec3 lightPos = glm::vec3(0.0f, 20.0f, 0.0f);
	glUniform3f(glGetUniformLocation(shader.Program, "lightPos"), (GLfloat)lightPos.x, (GLfloat)lightPos.y, (GLfloat)lightPos.z);

	model.Draw(shader);
}




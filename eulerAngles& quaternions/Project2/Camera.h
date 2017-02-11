#pragma once

/*
Camera Class from: 
	http://learnopengl.com/#!Getting-started/Camera
*/
// Std. Includes
#include <vector>

// GL Includes
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "maths_funcs.h"



// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement {
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT
};

// Default camera values
const GLfloat YAW = -90.0f;
const GLfloat PITCH = 0.0f;
const GLfloat SPEED = 3.0f;
const GLfloat SENSITIVTY = 0.25f;
const GLfloat ZOOM = 45.0f;


// An abstract camera class that processes input and calculates the corresponding Eular Angles, Vectors and Matrices for use in OpenGL
class Camera
{
public:
	bool isEuler;
	glm::vec3 Eangles;
	// Camera Attributes
	glm::vec3 Position;
	glm::vec3 Front;
	glm::vec3 Up;
	glm::vec3 Right;
	glm::vec3 WorldUp;
	// Eular Angles
	GLfloat Yaw;
	GLfloat Pitch;
	// Camera options
	GLfloat MovementSpeed;
	GLfloat MouseSensitivity;
	GLfloat Zoom;

	// Constructor with vectors
	Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), GLfloat yaw = YAW, GLfloat pitch = PITCH) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVTY), Zoom(ZOOM)
	{
		this->Position = position;
		this->WorldUp = up;
		this->Yaw = yaw;
		this->Pitch = pitch;
		this->updateCameraVectors2();
		//this->updateCameraVectors();
	}
	// Constructor with scalar values
	Camera(GLfloat posX, GLfloat posY, GLfloat posZ, GLfloat upX, GLfloat upY, GLfloat upZ, GLfloat yaw, GLfloat pitch) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVTY), Zoom(ZOOM)
	{
		this->Position = glm::vec3(posX, posY, posZ);
		this->WorldUp = glm::vec3(upX, upY, upZ);
		this->Yaw = yaw;
		this->Pitch = pitch;
		this->updateCameraVectors2();
		//this->updateCameraVectors();
	}

	// Returns the view matrix calculated using Eular Angles and the LookAt Matrix
	glm::mat4 GetViewMatrix()
	{
		return glm::lookAt(this->Position, this->Position + this->Front, this->Up);
	}

	// Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
	void ProcessKeyboard(Camera_Movement direction, GLfloat deltaTime)
	{
		GLfloat velocity = this->MovementSpeed * deltaTime;
		if (direction == FORWARD)
			this->Position += this->Front * velocity;
		if (direction == BACKWARD)
			this->Position -= this->Front * velocity;
		if (direction == LEFT)
			this->Position -= this->Right * velocity;
		if (direction == RIGHT)
			this->Position += this->Right * velocity;
	}

	// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
	void ProcessMouseMovement(GLfloat xoffset, GLfloat yoffset, GLboolean constrainPitch = true)
	{
		xoffset *= this->MouseSensitivity;
		yoffset *= this->MouseSensitivity;

		this->Yaw += xoffset;
		this->Pitch += yoffset;

		// Make sure that when pitch is out of bounds, screen doesn't get flipped
		if (constrainPitch)
		{
			if (this->Pitch > 89.0f)
				this->Pitch = 89.0f;
			if (this->Pitch < -89.0f)
				this->Pitch = -89.0f;
		}

		// Update Front, Right and Up Vectors using the updated Eular angles
		//this->updateCameraVectors();
		this->updateCameraVectors2();
	}

	// Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
	void ProcessMouseScroll(GLfloat yoffset)
	{
		if (this->Zoom >= 1.0f && this->Zoom <= 45.0f)
			this->Zoom -= yoffset;
		if (this->Zoom <= 1.0f)
			this->Zoom = 1.0f;
		if (this->Zoom >= 45.0f)
			this->Zoom = 45.0f;
	}

	// Calculates the front vector from the Camera's (updated) Eular Angles
	void updateCameraVectors()
	{
		// Calculate the new Front vector
		glm::vec3 front;
		front.x = cos(glm::radians(this->Yaw)) * cos(glm::radians(this->Pitch));
		front.y = sin(glm::radians(this->Pitch));
		front.z = sin(glm::radians(this->Yaw)) * cos(glm::radians(this->Pitch));

		this->Front = glm::normalize(front);
		// Also re-calculate the Right and Up vector
		this->Right = glm::normalize(glm::cross(this->Front, this->WorldUp));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
		this->Up = glm::normalize(glm::cross(this->Right, this->Front));
	}
	
	void updateCameraVectors2() {
		
		glm::mat4 R;

		if (isEuler)
		{
			//euler angles
			float xR[16] = {
				1.0f,0,0,0,
				0,cos(-Eangles.x),-sin(-Eangles.x),0,
				0,sin(-Eangles.x),cos(-Eangles.x),0,
				0,0,0,1.0f
			};
			glm::mat4 xRot = glm::make_mat4(xR);
			float yR[16] = {
				cos(-Eangles.y),0,sin(-Eangles.y),0,
				0,1.0f,0,0,
				-sin(-Eangles.y),0,cos(-Eangles.y),0,
				0,0,0,1.0f
			};
			glm::mat4 yRot = glm::make_mat4(yR);

			float zR[16] = {
				cos(-Eangles.z),-sin(-Eangles.z),0,0,
				sin(-Eangles.z),cos(-Eangles.z),0,0,
				0,0,1.0f,0,
				0,0,0,1.0f
			};
			glm::mat4 zRot = glm::make_mat4(zR);
			
			R= xRot*yRot*zRot;

		}
		else {

			//quaternions

			versor qX = quat_from_axis_rad(Eangles.x, 0.0f, 1.0f, 0.0f);
			versor qY = quat_from_axis_rad(Eangles.y, 1.0f, 0.0f, 0.0f);
			versor qZ = quat_from_axis_rad(Eangles.z, 0.0f, 0.0f, 1.0f);


			versor q = qX*qY*qZ;

			float w = q.q[0];
			float x = q.q[1];
			float y = q.q[2];
			float z = q.q[3];

			float quat[16]
			{
				1.0f - 2.0f * y * y - 2.0f * z * z,
				2.0f * x * y - 2.0f * w * z,
				2.0f * x * z + 2.0f * w * y,
				0.0f,

				2.0f * x * y + 2.0f * w * z,
				1.0f - 2.0f * x * x - 2.0f * z * z,
				2.0f * y * z - 2.0f * w * x,
				0.0f,

				2.0f * x * z - 2.0f * w * y,
				2.0f * y * z + 2.0f * w * x,
				1.0f - 2.0f * x * x - 2.0f * y * y,
				0.0f,

				0.0f,
				0.0f,
				0.0f,
				1.0f
			};

			glm::mat4 qMatrix = glm::make_mat4(quat);
			R = qMatrix;
		}
	
		glm::vec4 forward = glm::vec4(0.0f, 0.0f, -1.0f, 1.0f);
		forward = R*forward;
		glm::vec4 up= glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
		up = R*up;
		glm::vec4 right = glm::vec4(-1.0f, 0.0f, 0.0f, 1.0f);
		right = R*right;

		// Calculate the new Front vector
		glm::vec3 front;
		front.x = forward.x;
		front.y = forward.y;
		front.z = forward.z;
		
		glm::vec3 upV3;
		upV3.x = up.x;
		upV3.y = up.y;
		upV3.z = up.z;

		glm::vec3 rightV3;
		rightV3.x = right.x;
		rightV3.y = right.y;
		rightV3.z = right.z;

		this->Front = glm::normalize(front);
		this->Up = glm::normalize(upV3);
		this->Right =glm::normalize(rightV3);
		//this->Right= glm::normalize(glm::cross(this->Front, this->WorldUp));

	}
};
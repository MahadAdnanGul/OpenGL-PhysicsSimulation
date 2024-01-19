#pragma once
#include "Collider.h"
#include "Rigidbody.h"
#include "Transform.h"
#include "Utils.h"
#include "SOIL/SOIL.h"

class GameObject
{
public:
    Transform* transform;
    Rigidbody* rigidbody;
    Collider* collider;
    GLUquadric* quad;

    GameObject()
    {
        transform = new Transform;
        rigidbody = new Rigidbody;
        collider = new Collider(transform->scale->Magnitude()/2);
        quad = gluNewQuadric();
    }
    
    void Render()
    {
        glPushMatrix();
        //glTranslatef (transform->position->x, transform->position->y, transform->position->z);
        glMultMatrixf(quaternionAndTranslationToMatrix(transform->rotation,transform->position->x, transform->position->y, transform->position->z));
        gluQuadricTexture(quad,1);
        gluSphere(quad,transform->scale->x,20,20);
        glPopMatrix();
    }

    void CheckForCollisions(GameObject* collision)
    {
        if(Distance(transform->position,collision->transform->position) < collision->collider->radius + collider->radius)
        {
            ResolveCollision(collision);
        }
    }
    float MomentOfInertia()
    {
        return rigidbody->mass * pow(collider->radius, 2); 
    }

    void ResolveCollision(GameObject* collision)
    {
        // Calculate the direction from this object to the colliding object
        Vector3* collisionDirection = *collision->transform->position - transform->position;

        // Calculate the normal of our direction
        Vector3* collisionNormal = collisionDirection->Normalized();

        // Calculate the overlap distance
        float overlap = (collider->radius + collision->collider->radius) - collisionDirection->Magnitude();

        // calculate the separation vector
        Vector3* separation = *collisionDirection*(0.25f*overlap);

        // Adjust the positions of the colliding objects using the separation vector so that they are no longer overlapping
        transform->position = *transform->position - (separation);
        collision->transform->position = *collision->transform->position + (separation);


        //Handling Translation
        //Calculate relative velocity
        Vector3* relativeVelocity = *rigidbody->velocity-collision->rigidbody->velocity;

        // Calculate impulse magnitude based of conservation of momentum and kinetic energy. We take dot product with our normal to account for the angle of collision
        float impulseMagnitude = -(1.0f + rigidbody->bounceReduction) * relativeVelocity->Dot(collisionNormal);
        impulseMagnitude /= 1.0f / rigidbody->mass + 1.0f / collision->rigidbody->mass;

        //adjust resultant velocity addition based on mass
        Vector3* resultant = *collisionNormal*(impulseMagnitude/rigidbody->mass);
        Vector3* resultantB = *collisionNormal*(impulseMagnitude/collision->rigidbody->mass);
        
        rigidbody->velocity = *rigidbody->velocity + resultant;
        collision->rigidbody->velocity = *collision->rigidbody->velocity - resultantB;

        Vector3* relativeAngularVelocity = *rigidbody->angularVelocity - collision->rigidbody->angularVelocity;
        
        float momentOfInertiaA = MomentOfInertia();
        float momentOfInertiaB = collision->MomentOfInertia();

        // Calculate the angular impulse similarly using conservation of angular momentum
        float angularImpulseMagnitude = -(1.0f + rigidbody->bounceReduction) * relativeAngularVelocity->Dot(collisionNormal);
        angularImpulseMagnitude /= 1.0f / momentOfInertiaA + 1.0f / momentOfInertiaB;

        // Apply angular impulse to adjust angular velocities
        Vector3* angularResultA = *collisionNormal*(angularImpulseMagnitude / momentOfInertiaA) ;
        Vector3* angularResultB = *collisionNormal*(angularImpulseMagnitude / momentOfInertiaB) ;
        rigidbody->angularVelocity = *rigidbody->angularVelocity + angularResultA;
        collision->rigidbody->angularVelocity = *collision->rigidbody->angularVelocity - angularResultB;
    }

    void CheckForCollisionsWithBounds(Vector3* negBounds, Vector3* posBounds)
    {
        // Simply reflect velocity with bounds and reduce angular momentum based on bounce reduction
        if(transform->position->y - collider->radius <= negBounds->y)
        {
            transform->position = new Vector3(transform->position->x,negBounds->y + collider->radius,transform->position->z);
            rigidbody->velocity = new Vector3(rigidbody->velocity->x*(rigidbody->bounceReduction),-rigidbody->velocity->y*(rigidbody->bounceReduction),rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }
        if(transform->position->y + collider->radius >= posBounds->y)
        {
            transform->position = new Vector3(transform->position->x,posBounds->y - collider->radius,transform->position->z);
            rigidbody->velocity = new Vector3(rigidbody->velocity->x*(rigidbody->bounceReduction),-rigidbody->velocity->y*(rigidbody->bounceReduction),rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }

        if(transform->position->x - collider->radius <= negBounds->x)
        {
            transform->position = new Vector3(negBounds->x + collider->radius,transform->position->y,transform->position->z);
            rigidbody->velocity = new Vector3(-rigidbody->velocity->x*(rigidbody->bounceReduction),rigidbody->velocity->y*(rigidbody->bounceReduction),rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }
        if(transform->position->x + collider->radius >= posBounds->x)
        {
            transform->position = new Vector3(posBounds->x - collider->radius,transform->position->y,transform->position->z);
            rigidbody->velocity = new Vector3(-rigidbody->velocity->x*(rigidbody->bounceReduction),rigidbody->velocity->y*(rigidbody->bounceReduction),rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }

        if(transform->position->z - collider->radius <= negBounds->z)
        {
            transform->position = new Vector3(transform->position->x,transform->position->y,negBounds->z + collider->radius);
            rigidbody->velocity = new Vector3(rigidbody->velocity->x*(rigidbody->bounceReduction),rigidbody->velocity->y*(rigidbody->bounceReduction),-rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }
        if(transform->position->z + collider->radius >= posBounds->z)
        {
            transform->position = new Vector3(transform->position->x,transform->position->y,posBounds->z - collider->radius);
            rigidbody->velocity = new Vector3(rigidbody->velocity->x*(rigidbody->bounceReduction),rigidbody->velocity->y*(rigidbody->bounceReduction),-rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }
    }
    void ResolveForces(float dt)
    {
        //Record changes in velocity
        float x = rigidbody->acceleration->x * dt - (rigidbody->velocity->x * rigidbody->drag * dt);
        float y = rigidbody->acceleration->y * dt - (rigidbody->velocity->y * rigidbody->drag * dt);
        float z = rigidbody->acceleration->z * dt - (rigidbody->velocity->z * rigidbody->drag * dt);

        //Add gravitational force if gravity enabled
        if(rigidbody->useGravity)
        {
            y += Rigidbody::GRAVITY*dt;
        }

        //Convert angular velocity change into a quaternion
        Quaternion* q = eulerToQuaternion(rigidbody->angularVelocity->x*dt,rigidbody->angularVelocity->y*dt,rigidbody->angularVelocity->z*dt);

        //Calculate new rotation
        Quaternion* newRotation = *q * transform->rotation;
        

        //Normalize
        double magnitude = std::sqrt(newRotation->x * newRotation->x +
                              newRotation->y * newRotation->y +
                              newRotation->z * newRotation->z +
                              newRotation->w * newRotation->w);
        
        newRotation->x /= magnitude;
        newRotation->y /= magnitude;
        newRotation->z /= magnitude;
        newRotation->w /= magnitude;

        //Update rotation
        transform->rotation = newRotation;
        
        // update velocity
        rigidbody->velocity = new Vector3(rigidbody->velocity->x+x,rigidbody->velocity->y+y,rigidbody->velocity->z+z);

        //Apply angular drag
        float xA = (rigidbody->angularVelocity->x * rigidbody->angularDrag * dt);
        float yA = (rigidbody->angularVelocity->y * rigidbody->angularDrag * dt);
        float zA = (rigidbody->angularVelocity->z * rigidbody->angularDrag * dt);

        rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x - xA,rigidbody->angularVelocity->y-yA,rigidbody->angularVelocity->z-zA);
        
        //update position based on velocity
        transform->position = new Vector3(transform->position->x+(rigidbody->velocity->x*dt),transform->position->y+(rigidbody->velocity->y*dt),transform->position->z+(rigidbody->velocity->z*dt));
    }
    
};

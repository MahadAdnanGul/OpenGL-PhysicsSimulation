class Rigidbody
{
public:
    bool useGravity;
    static constexpr float GRAVITY = -9.81f;
    
    Vector3* velocity;
    Vector3* angularVelocity; // in radians per second
    Vector3* acceleration;
    float drag;
    float angularDrag;
    float mass;
    float bounceReduction;
    
    Rigidbody()
    {
        useGravity = true;
        velocity = Vector3::zero();
        acceleration = Vector3::zero();
        angularVelocity = Vector3::zero();
        drag = 0.01f;
        angularDrag = 0.01f;
        mass = 1;
        bounceReduction = 0.85f;
    }
    
    void AddImpulseForce(Vector3* force)
    {
        velocity = new Vector3(velocity->x+force->x,velocity->y+force->y,velocity->z+force->z);
    }

    void AddAngularImpulse(Vector3* force)
    {
        angularVelocity = new Vector3(angularVelocity->x+force->x,angularVelocity->y+force->y,angularVelocity->z+force->z);
    }
};

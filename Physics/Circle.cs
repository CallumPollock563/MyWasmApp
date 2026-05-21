using System;

namespace PhysicsEngine
{
    
    /// Represents a physics entity with physical properties and collision resolution capabilities.
    
    internal class Circle
    {
        public float X;
        public float Y;
        public float Radius;
        public float VelocityX;
        public float VelocityY;

        public float Restitution = 0.85f; 
        public float Mass = 1f;

        public Circle(float x, float y, float radius, float velocityX, float velocityY, float mass)
        {
            this.X = x;
            this.Y = y;
            this.Radius = radius;
            this.VelocityX = velocityX;
            this.VelocityY = velocityY;
            this.Mass = mass;
        }

        
        /// Updates the circle's position and velocity based on elapsed time, gravity, and bounds.
        
        public void UpdatePhysics(float floor, float left, float right, float ceiling, float deltaTime, float gravityX, float gravityY)
        {
            VelocityY += gravityY * deltaTime;
            VelocityX += gravityX * deltaTime;

            X += VelocityX * deltaTime;
            Y += VelocityY * deltaTime;

            VelocityX *= 0.999f;
            VelocityY *= 0.999f;

            ResolveWallCollisions(floor, left, right, ceiling);
        }

        
        /// Detects and resolves collisions against the defined spatial boundaries.
        
        private void ResolveWallCollisions(float floor, float left, float right, float ceiling)
        {
            if (Y + Radius > floor)
            {
                Y = floor - Radius;
                if (VelocityY > 0) VelocityY = -VelocityY * Restitution;
            }

            if (Y - Radius < ceiling)
            {
                Y = ceiling + Radius;
                if (VelocityY < 0) VelocityY = -VelocityY * Restitution;
            }

            if (X + Radius > right)
            {
                X = right - Radius;
                if (VelocityX > 0) VelocityX = -VelocityX * Restitution;
            }

            if (X - Radius < left)
            {
                X = left + Radius;
                if (VelocityX < 0) VelocityX = -VelocityX * Restitution;
            }
        }

        
        /// Resolves overlapping and dynamic collisions with another Circle instance.
        /// Includes validation to prevent divide-by-zero errors.
        
        public void ResolveCollision(Circle other)
        {
            float deltaX = other.X - X;
            float deltaY = other.Y - Y;

            float distanceSquared = deltaX * deltaX + deltaY * deltaY;
            float minimumDistance = Radius + other.Radius;

            if (distanceSquared >= minimumDistance * minimumDistance)
                return;

            float distance = (float)Math.Sqrt(distanceSquared);

            float normalX, normalY;
            
            // Error Prevention: Avoid divide-by-zero if objects occupy the exact same space
            if (distance == 0f)
            {
                normalX = 1f;
                normalY = 0f;
                distance = 0.001f;
            }
            else
            {
                normalX = deltaX / distance;
                normalY = deltaY / distance;
            }

            float overlap = minimumDistance - distance;
            float combinedMass = Mass + other.Mass;

            X -= normalX * overlap * (other.Mass / combinedMass);
            Y -= normalY * overlap * (other.Mass / combinedMass);

            other.X += normalX * overlap * (Mass / combinedMass);
            other.Y += normalY * overlap * (Mass / combinedMass);

            float relativeVelocityX = other.VelocityX - VelocityX;
            float relativeVelocityY = other.VelocityY - VelocityY;

            float velocityAlongNormal = relativeVelocityX * normalX + relativeVelocityY * normalY;

            if (velocityAlongNormal > 0)
                return;

            float impulseMagnitude = -(1 + Restitution) * velocityAlongNormal;
            impulseMagnitude /= (1 / Mass + 1 / other.Mass);

            float impulseX = impulseMagnitude * normalX;
            float impulseY = impulseMagnitude * normalY;

            VelocityX -= impulseX / Mass;
            VelocityY -= impulseY / Mass;

            other.VelocityX += impulseX / other.Mass;
            other.VelocityY += impulseY / other.Mass;
        }
    }
}
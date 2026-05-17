using System;

namespace PhysicsEngine
{
    internal class Circle
    {
        // Position coordinates
        public float X;
        public float Y;
        public float Radius;

        // Movement vectors
        public float VelocityX;
        public float VelocityY;

        // Physical properties
        public float Restitution = 0.85f; // Bounciness
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

        // Main simulation step
        public void UpdatePhysics(float floor, float left, float right, float ceiling, float deltaTime, float gravityX, float gravityY)
        {
            // Apply gravity influence
            VelocityY += gravityY * deltaTime;
            VelocityX += gravityX * deltaTime;

            // Apply velocity to position
            X += VelocityX * deltaTime;
            Y += VelocityY * deltaTime;

            // Global air friction / damping
            VelocityX *= 0.999f;
            VelocityY *= 0.999f;

            ResolveWallCollisions(floor, left, right, ceiling);
        }

        private void ResolveWallCollisions(float floor, float left, float right, float ceiling)
        {
            // Bottom wall
            if (Y + Radius > floor)
            {
                Y = floor - Radius;
                if (VelocityY > 0) VelocityY = -VelocityY * Restitution;
            }

            // Top wall
            if (Y - Radius < ceiling)
            {
                Y = ceiling + Radius;
                if (VelocityY < 0) VelocityY = -VelocityY * Restitution;
            }

            // Right wall
            if (X + Radius > right)
            {
                X = right - Radius;
                if (VelocityX > 0) VelocityX = -VelocityX * Restitution;
            }

            // Left wall
            if (X - Radius < left)
            {
                X = left + Radius;
                if (VelocityX < 0) VelocityX = -VelocityX * Restitution;
            }
        }

        public void ResolveCollision(Circle other)
        {
            float deltaX = other.X - X;
            float deltaY = other.Y - Y;

            float distanceSquared = deltaX * deltaX + deltaY * deltaY;
            float minimumDistance = Radius + other.Radius;

            // Check if circles are overlapping
            if (distanceSquared >= minimumDistance * minimumDistance)
                return;

            float distance = (float)Math.Sqrt(distanceSquared);

            // Calculate collision normal
            float normalX, normalY;
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

            // Static resolution: push circles apart so they don't overlap
            float overlap = minimumDistance - distance;
            float combinedMass = Mass + other.Mass;

            X -= normalX * overlap * (other.Mass / combinedMass);
            Y -= normalY * overlap * (other.Mass / combinedMass);

            other.X += normalX * overlap * (Mass / combinedMass);
            other.Y += normalY * overlap * (Mass / combinedMass);

            // Dynamic resolution: calculate impulse based on relative velocity
            float relativeVelocityX = other.VelocityX - VelocityX;
            float relativeVelocityY = other.VelocityY - VelocityY;

            float velocityAlongNormal = relativeVelocityX * normalX + relativeVelocityY * normalY;

            // Do not resolve if objects are already moving apart
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
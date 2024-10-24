package org.firstinspires.ftc.teamcode;

public class Localization {
    // Position and orientation
    private double x, y, theta;

    // Velocity variables
    private double linearVelocity; // in meters per second
    private double angularVelocity; // in radians per second

    // Time tracking
    private long lastUpdateTime;

    // Constructor to initialize the position and orientation
    public Localization(double initialX, double initialY, double initialTheta) {
        this.x = initialX;
        this.y = initialY;
        this.theta = initialTheta;
        this.linearVelocity = 0.0;
        this.angularVelocity = 0.0;
        this.lastUpdateTime = System.nanoTime();
    }

    // Update velocities (from encoders and gyroscope)
    public void setVelocities(double linearVelocity, double angularVelocity) {
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;
    }

    // Update the robot's position based on time elapsed
    public void updatePosition() {
        // Calculate the time elapsed since the last update in seconds
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastUpdateTime) / 1e9; // Convert to seconds
        lastUpdateTime = currentTime;

        // Update the position based on the motion model
        double deltaX = linearVelocity * deltaTime * Math.cos(theta);
        double deltaY = linearVelocity * deltaTime * Math.sin(theta);
        double deltaTheta = angularVelocity * deltaTime;

        // Update position and orientation
        this.x += deltaX;
        this.y += deltaY;
        this.theta += deltaTheta;

        // Normalize theta to stay within 0 and 2*PI
        this.theta = normalizeAngle(this.theta);
    }

    // Normalize the angle to stay between 0 and 2*PI
    private double normalizeAngle(double angle) {
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        while (angle >= 2 * Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    // Getters for position and orientation
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    // Method to reset the position (for resetting during matches)
    public void resetPosition(double newX, double newY, double newTheta) {
        this.x = newX;
        this.y = newY;
        this.theta = newTheta;
    }
}


package org.firstinspires.ftc.teamcode.purepursuit;

public class Waypoint {
    private double x;
    private double y;
    private double velocity;

    public Waypoint(double x, double y, double velocity) {
        this.x = x;
        this.y = y;
        this.velocity = velocity;
    }

    // Getters for position and velocity
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getVelocity() {
        return velocity;
    }
}

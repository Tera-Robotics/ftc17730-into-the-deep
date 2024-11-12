package org.firstinspires.ftc.teamcode.purepursuit;

public class Waypoint {
    private double x;
    private double y;
    private double heading;

    public Waypoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    // Getters for position and velocity
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}

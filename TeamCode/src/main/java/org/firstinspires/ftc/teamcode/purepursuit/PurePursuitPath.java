package org.firstinspires.ftc.teamcode.purepursuit;

import java.util.ArrayList;
import java.util.List;

public class PurePursuitPath {
    private List<Waypoint> waypoints;

    public PurePursuitPath() {
        waypoints = new ArrayList<>();
    }

    // Adds a waypoint to the path
    public void addWaypoint(double x, double y, double velocity) {
        waypoints.add(new Waypoint(x, y, velocity));
    }

    // Returns the list of waypoints
    public List<Waypoint> getWaypoints() {
        return waypoints;
    }

    // Finds the closest waypoint to the given position
    public Waypoint getClosestWaypoint(double x, double y) {
        Waypoint closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Waypoint waypoint : waypoints) {
            double distance = MathUtils.distance(x, y, waypoint.getX(), waypoint.getY());
            if (distance < minDistance) {
                minDistance = distance;
                closest = waypoint;
            }
        }

        return closest;
    }
}


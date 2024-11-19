package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.List;

public class PurePursuit {
    static class Robot {
        Vector2d position;
        double heading; // In radians
        double lookaheadDistance;

        Robot(double x, double y, double heading, double lookaheadDistance) {
            this.position = new Vector2d(x, y);
            this.heading = heading;
            this.lookaheadDistance = lookaheadDistance;
        }
    }
    private final Trajectory trajectory;
    private final double lookaheadDistance;

    public PurePursuit(Trajectory trajectory, double lookaheadDistance) {
        this.trajectory = trajectory;
        this.lookaheadDistance = lookaheadDistance;
    }
    static Vector2d findLookaheadPoint(Robot robot, List<Vector2d> path) {
        for (int i = 0; i < path.size() - 1; i++) {
            Vector2d start = path.get(i);
            Vector2d end = path.get(i + 1);

            // Vector math to find intersection
            double dx = end.x - start.x;
            double dy = end.y - start.y;

            double fx = start.x - robot.position.x;
            double fy = start.y - robot.position.y;

            double a = dx * dx + dy * dy;
            double b = 2 * (fx * dx + fy * dy);
            double c = fx * fx + fy * fy - robot.lookaheadDistance * robot.lookaheadDistance;

            double discriminant = b * b - 4 * a * c;
            if (discriminant >= 0) {
                discriminant = Math.sqrt(discriminant);
                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);

                if (t1 >= 0 && t1 <= 1) {
                    return new Vector2d(start.x + t1 * dx, start.y + t1 * dy);
                }
                if (t2 >= 0 && t2 <= 1) {
                    return new Vector2d(start.x + t2 * dx, start.y + t2 * dy);
                }
            }
        }
        return null; // No valid lookahead point
    }
    public double lineCircleIntersection(Pose2d currentPos, Waypoint pt1,Waypoint pt2,double lookAheadDis) {
        double currentX = currentPos.position.x;
        double currentY = currentPos.position.y;
        double x1 = pt1.getX();
        double y1 = pt1.getY();
        double x2 = pt2.getX();
        double y2 = pt2.getY();

        double dx = x2 - x1;
        double dy = y2 - y1;

        // Coefficients for the quadratic equation
        double A = dx * dx + dy * dy;
        double B = 2 * (dx * (x1 - currentX) + dy * (y1 - currentY));
        double C = (x1 - currentX) * (x1 - currentX) + (y1 - currentY) * (y1 - currentY) - lookAheadDis * lookAheadDis;

        double discriminant = B * B - 4 * A * C;

        if (discriminant < 0) {
            // No intersection
            return 0;
        } else {
            // One or two intersections
            double sqrtDiscriminant = Math.sqrt(discriminant);
            double t1 = (-B + sqrtDiscriminant) / (2 * A);
            double t2 = (-B - sqrtDiscriminant) / (2 * A);

            // Check if t1 or t2 are within the segment [0,1]
            boolean validT1 = t1 >= 0 && t1 <= 1;
            boolean validT2 = t2 >= 0 && t2 <= 1;

            if (validT1 || validT2) {
                // Calculate intersection points
                double intersectX1 = x1 + t1 * dx;
                double intersectY1 = y1 + t1 * dy;
                double intersectX2 = x1 + t2 * dx;
                double intersectY2 = y1 + t2 * dy;

                // Return the closest intersection point to the current position
                double dist1 = validT1 ? Math.hypot(intersectX1 - currentX, intersectY1 - currentY) : Double.MAX_VALUE;
                double dist2 = validT2 ? Math.hypot(intersectX2 - currentX, intersectY2 - currentY) : Double.MAX_VALUE;

                // Return the minimum distance intersection
                return Math.min(dist1, dist2);
            }
        }
        // No valid intersection within the line segment
        return 0;
    }
}
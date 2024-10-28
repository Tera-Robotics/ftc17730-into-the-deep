package org.firstinspires.ftc.teamcode.purepursuit;

public class PurePursuitController {
    private PurePursuitPath path;
    private double lookaheadDistance;
    private double currentX, currentY, currentHeading;

    public PurePursuitController(PurePursuitPath path, double lookaheadDistance) {
        this.path = path;
        this.lookaheadDistance = lookaheadDistance;
    }

    // Set the current position and heading of the robot
    public void updatePosition(double x, double y, double heading) {
        this.currentX = x;
        this.currentY = y;
        this.currentHeading = heading;
    }

    // Calculate the lookahead point based on the current position and heading
    public Waypoint getLookaheadPoint() {
        Waypoint lookaheadPoint = null;

        for (Waypoint waypoint : path.getWaypoints()) {
            double distance = MathUtils.distance(currentX, currentY, waypoint.getX(), waypoint.getY());

            // Check if the waypoint is within the lookahead distance
            if (distance > lookaheadDistance) {
                lookaheadPoint = waypoint;
                break;
            }
        }

        return lookaheadPoint;
    }

    // Calculate motor powers based on the lookahead point
    public double[] calculateMotorPowers() {
        Waypoint lookahead = getLookaheadPoint();
        if (lookahead == null) {
            // If no lookahead point, stop
            return new double[]{0, 0, 0, 0};
        }

        double targetAngle = MathUtils.angleBetweenPoints(currentX, currentY, lookahead.getX(), lookahead.getY());
        double angleDifference = MathUtils.normalizeAngle(targetAngle - currentHeading);

        double speed = lookahead.getVelocity();
        double turnAdjustment = angleDifference * 0.5; // Tuning factor for turn control

        // Simple calculation for mecanum wheel power values
        double frontLeftPower = speed - turnAdjustment;
        double frontRightPower = speed + turnAdjustment;
        double backLeftPower = speed - turnAdjustment;
        double backRightPower = speed + turnAdjustment;

        return new double[]{frontLeftPower, frontRightPower, backLeftPower, backRightPower};
    }

    // Setters for path and lookahead distance, enabling reusability
    public void setPath(PurePursuitPath path) {
        this.path = path;
    }

    public void setLookaheadDistance(double lookaheadDistance) {
        this.lookaheadDistance = lookaheadDistance;
    }
}

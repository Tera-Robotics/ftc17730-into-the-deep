package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;

public class PurePursuit {
    private final Trajectory trajectory;
    private final double lookaheadDistance;

    public PurePursuit(Trajectory trajectory, double lookaheadDistance) {
        this.trajectory = trajectory;
        this.lookaheadDistance = lookaheadDistance;
    }

    public double lineCircleIntersection(Pose2d currentPos, Waypoint pt1,Waypoint pt2,double lookAheadDis) {
        double currentX = currentPos.position.x;
        double currentY = currentPos.position.y;
        double x1 = pt1.getX();
        double y1 = pt1.getY();
        double x2 = pt2.getX();
        double y2 = pt2.getY();

        return currentX;
    }
}
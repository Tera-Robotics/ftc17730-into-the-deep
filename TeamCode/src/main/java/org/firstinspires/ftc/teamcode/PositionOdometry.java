package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PositionOdometry {
    SimpleKalmanFilter skf1 = new SimpleKalmanFilter();
    // Robot constants
    private final double wheelRadius = 0.096; // meters (example: 7.5 cm)
    private final double wheelbaseLength = 0.26; // meters (example: distance between front and back wheels)
    private final double trackWidth = 0.254; // meters (example: distance between left and right wheels)
    // Motors for Mecanum wheels
    private final DcMotorEx leftFront, leftBack, rightFront, rightBack;
    // IMU sensor for heading
    private IMU imu;
    // Previous encoder positions
    private int lastLeftFrontPos = 0;
    private int lastRightFrontPos = 0;
    private int lastLeftBackPos = 0;
    private int lastRightBackPos = 0;

    // Position and heading
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    // Conversion factors (tune these based on your robot’s configuration)
    private static final double COUNTS_PER_INCH = 3.78; // Adjust based on your robot's specific encoder and wheel configuration
    private static final double TRACK_WIDTH = 10.0;     // Distance between left and right wheels (in inches)

    public PositionOdometry(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, IMU imu) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.imu = imu;

        SetEncodersToCurrentPosition();
    }

    public void updatePosition() {
        // Get current encoder positions
        int currentLeftFrontPos = leftFront.getCurrentPosition();
        int currentRightFrontPos = rightFront.getCurrentPosition();
        int currentLeftBackPos = leftBack.getCurrentPosition();
        int currentRightBackPos = rightBack.getCurrentPosition();

        // Calculate change in encoder values
        int deltaLeftFront = currentLeftFrontPos - lastLeftFrontPos;
        int deltaRightFront = currentRightFrontPos - lastRightFrontPos;
        int deltaLeftBack = currentLeftBackPos - lastLeftBackPos;
        int deltaRightBack = currentRightBackPos - lastRightBackPos;

        // Update previous encoder values for next loop
        lastLeftFrontPos = currentLeftFrontPos;
        lastRightFrontPos = currentRightFrontPos;
        lastLeftBackPos = currentLeftBackPos;
        lastRightBackPos = currentRightBackPos;

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Compute the linear velocities in X and Y, and angular velocity (omega)
        double vx = (wheelRadius / 4) * (deltaLeftFront + deltaRightFront + deltaLeftBack + deltaRightBack);
        double vy = (wheelRadius / 4) * (-deltaLeftFront + deltaRightFront + deltaLeftBack - deltaRightBack);

        // Update robot position and orientation (assuming constant velocities over small time steps)
        x += vx * Math.cos(Math.toRadians(heading)) - vy * Math.sin(Math.toRadians(heading));  // Update X
        y += vx * Math.sin(Math.toRadians(heading)) + vy * Math.cos(Math.toRadians(heading));  // Update Y
    }

    public void SetEncodersToCurrentPosition() {
        lastLeftFrontPos = leftFront.getCurrentPosition();
        lastRightFrontPos = rightFront.getCurrentPosition();
        lastLeftBackPos = leftBack.getCurrentPosition();
        lastRightBackPos = rightBack.getCurrentPosition();
    }

    // Getter methods for x, y, and heading
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
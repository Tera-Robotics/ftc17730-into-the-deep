package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.purepursuit.Waypoint;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Auto", group = "into-the-deep", preselectTeleOp = "TeleOp OpMode")
public class Auto extends OpMode {
    private IMU imu;
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;
    private List<Waypoint> waypoints;
    private int currentWaypointIndex = 0;
    private PositionOdometry odometry;
    Pose2d currentPose = new Pose2d(0,0,0);
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard;
    @Override
    public void init() {
        telemetry.speak("Autonomous Code");
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        elapsedtime = new ElapsedTime();

        // this just sets the bulk reading mode for each hub
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();
        leftFront = hardwareMap.get(DcMotorEx .class, "lfDriveMotor");
        leftBack = hardwareMap.get(DcMotorEx.class, "lbDriveMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "rfDriveMotor");
        rightBack = hardwareMap.get(DcMotorEx.class, "rbDriveMotor");

        odometry = new PositionOdometry(leftFront, leftBack, rightFront, rightBack, imu);

        waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(1, 0, 0));
        waypoints.add(new Waypoint(1, 1, 0));
        waypoints.add(new Waypoint(0, 1, 0));
        waypoints.add(new Waypoint(0, 0, 0));

        elapsedtime.reset();
    }
    @Override
    public void loop() {
        telemetry.clearAll();
        odometry.updatePosition();
        currentPose = new Pose2d(odometry.getX(), odometry.getY(), odometry.getHeading());
        Vector2d halfv = currentPose.heading.vec().times(0.5 * 9);
        Vector2d p1 = currentPose.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        //.drawImage("https://drive.google.com/file/d/1eqV6S9xsajhLDhnBWn1BKKOlx8FuVWeZ/view?usp=sharing", 0,0,144,144)
        packet.fieldOverlay()
                .setAlpha(0.4)
                .drawImage("/dash/into-the-deep.png", 0, 0, 144, 144)
                .setAlpha(1.0)
                .setStroke("#C9497E")
                .drawGrid(0, 0, 144, 144, 7, 7)
                .setStroke("#3F51B5")
                .strokeCircle((float) odometry.getX(), (float) odometry.getY(), 9)
                .strokeLine(p1.x, p1.y, p2.x, p2.y);
        dashboard.sendTelemetryPacket(packet);/*
        telemetry.addData("leftBackDrive", leftBack.getCurrentPosition());
        telemetry.addData("leftFrontDrive", leftFront.getCurrentPosition());
        telemetry.addData("rightBackDrive", rightBack.getCurrentPosition());
        telemetry.addData("rightFrontDrive", rightFront.getCurrentPosition());*/
        telemetry.addData("X Position", odometry.getX());
        telemetry.addData("Y Position", odometry.getY());
        telemetry.addData("Heading", odometry.getHeading());
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
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

import org.firstinspires.ftc.teamcode.purepursuit.PurePursuitController;
import org.firstinspires.ftc.teamcode.purepursuit.PurePursuitPath;

import java.util.List;

@Autonomous(name = "Auto", group = "into-the-deep", preselectTeleOp = "TeleOp OpMode")
public class Auto extends OpMode {
    private IMU imu;
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;
    private PositionOdometry odometry;
    Pose2d currentPose = new Pose2d(0,0,0);
    Pose2d startPose = new Pose2d(0, 0, 0);
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard;
    PurePursuitController follower;
    double lbPower, lfPower, rbPower, rfPower;
    @Override
    public void init() {
        telemetry.speak("Autonomous");
        dashboard = FtcDashboard.getInstance();
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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

        PurePursuitPath path = new PurePursuitPath();
        path.addWaypoint(1,0,0);
        path.addWaypoint(1,1,0);
        path.addWaypoint(0,1,0);
        path.addWaypoint(0,0,0);

        follower = new PurePursuitController(path,9);

        elapsedtime.reset();
    }
    @Override
    public void loop() {
        odometry.updatePosition();
        float robotX = (float) odometry.getX();
        float robotY = (float) odometry.getY();
        float robotHeading = (float) odometry.getHeading();
        currentPose = new Pose2d(robotX, robotY, robotHeading);
        Vector2d halfv = currentPose.heading.vec().times(0.5 * 9);
        Vector2d p1 = currentPose.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        follower.updatePosition(robotX, robotY, robotHeading);
        double[] powers = follower.calculateMotorPowers();
        lbPower = powers[0];
        lfPower = powers[1];
        rbPower = powers[2];
        rfPower = powers[3];
        //leftBack.setPower(lbPower);
        //leftFront.setPower(lfPower);
        //rightBack.setPower(rbPower);
        //rightFront.setPower(rfPower);

        telemetry.addData("leftBackPower", lbPower);
        telemetry.addData("leftFrontDrive", lfPower);
        telemetry.addData("rightBackDrive", rbPower);
        telemetry.addData("rightFrontDrive", rfPower);
        telemetry.addData("X Position", robotX);
        telemetry.addData("Y Position", robotY);
        telemetry.addData("Heading", robotHeading);
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
        telemetry.update();
        packet.fieldOverlay()
                .setAlpha(0.4)
                .drawImage("/dash/into-the-deep.png", 0, 0, 144, 144)
                .setAlpha(1.0)
                .setStroke("#C9497E")
                .drawGrid(0, 0, 144, 144, 7, 7)
                .setStroke("#3F51B5")
                .strokeCircle(robotX, robotY, 9)
                .strokeLine(p1.x, p1.y, p2.x, p2.y);
        dashboard.sendTelemetryPacket(packet);
    }
}

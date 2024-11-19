package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "\uD83D\uDD35 Blue Autonomous OpMode", group = "into-the-deep", preselectTeleOp = "TeleOp OpMode")
public class Autonomous2 extends LinearOpMode {
    double TICKS_PER_REVOLUTION = 448;
    double WHEEL_DIAMETER = 0.96;
    IMU imu;
    private ElapsedTime elapsedtime;
    CustomPIDFController gyroControl = new CustomPIDFController(0.00725, 0.00475, 0.01, 0.22);
    double error = 10;
    HardwareMecanum drive = new HardwareMecanum(this);
    @Override
    public void runOpMode() throws InterruptedException {
        drive.init();
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
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
        waitForStart();

        straightFor(800);
        rotateUsingGyro(90);
        straightFor(1600);
        rotateUsingGyro(90);
        straightFor(800);
        telemetry.addLine("Acabaste");
        telemetry.update();
}
    private void rotateUsingGyro(double angle) {
        imu.resetYaw();
        error = angle - getHeading();
        while (Math.abs(error) >= 2 && opModeIsActive()) {
            double currentHeading = getHeading();
            double correction = gyroControl.update(angle, currentHeading);
            error = angle - currentHeading;
            correction = Range.clip(correction, -0.3,0.3);
            if (angle > 0) { correction *= -1; }
            drive.setDrivePower(correction, correction, -correction, -correction);
        }
        drive.stopMotors();
        sleep(100);
    }
    private void straightFor(int position) {
        drive.resetEncoders();
        while (drive.lfDrive.getCurrentPosition() < position && drive.rfDrive.getCurrentPosition() < position) {
            drive.setPowerFourWheels(0.32);
        }
        drive.stopMotors();
        sleep(100);
    }

    private void driveStraigh(int target, double power) {
        double startlfEncoder = drive.lfDrive.getCurrentPosition();
        double startrfEncoder = drive.rfDrive.getCurrentPosition();
        double leftError = target - startlfEncoder;
        double rightError = target - startrfEncoder;
        while (Math.abs(leftError) > 1 && Math.abs(rightError) > 1) {
            leftError = target - startlfEncoder;
            rightError = target - startrfEncoder;
            if (leftError < 0 || rightError < 0){ power *= -1; }
            drive.setDrivePower(power, power, power, power);
        }
    }
    private void rotateToRelativeHeading(double angle) {
        double target = getHeading() - angle;
        error = target - getHeading();
        while (Math.abs(error) >= 3 && opModeIsActive()) {
            double correction = gyroControl.update(target, getHeading());
            error = target - getHeading();
            correction = Range.clip(correction, -0.3,0.3);
            //if (error > 90) { correction *= -1; }
            drive.setDrivePower(-correction, -correction, correction, correction);
            telemetry.addData("target", target);
            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            telemetry.update();
        }
        telemetry.addLine("Exited Loop ----");
        telemetry.addData("error", error);
        telemetry.addData("correction", Math.abs(error) <= target);
        telemetry.update();
        drive.stopMotors();
        sleep(5000);
    }
    public double getHeading() {
        double head = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return head % 360;
    }
}

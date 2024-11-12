package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Config
@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class BasicOpMode_Linear_Meccanum extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    private List<LynxModule> allHubs;
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    public float HEADING_GAIN = 0.2f;  // Gain for heading correction
    float x,y,turn;
    double botHeading, previousBotHeading, headingError, rotationCorrection;
    double denominator;
    double frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    double targetHeading = 0.0;
    double armTargetPos = 0;
    public static double n1 = 0;
    public static double n2 = 0;
    public static double n3 = 0;
    public static double n4 = 0;
    CustomPIDFController PIDF = new CustomPIDFController(n1,n2,n3,n4);
    public static int SLIDER_EXTENDED = 2300;
    int SLIDER_RETRACTED = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx .class, "lfDriveMotor");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "lbDriveMotor");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "rbDriveMotor");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "rfDriveMotor");

        Servo servoClaw1 = hardwareMap.get(Servo.class, "servoClaw1");
        Servo servoClaw2 = hardwareMap.get(Servo.class, "servoClaw2");
        Servo servoClawRotation = hardwareMap.get(Servo.class, "servoClawRotation");


        DcMotorEx armMotor = hardwareMap.get(DcMotorEx .class, "armMotor");
        DcMotorEx elevatorMotor = hardwareMap.get(DcMotorEx .class, "elevatorMotor");

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int sliderPreviousPos = elevatorMotor.getCurrentPosition();
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            if (gamepad2.x) {
                servoClaw1.setPosition(1.0);
                servoClaw2.setPosition(0.0);
            } else if (gamepad2.b) {
                servoClaw1.setPosition(0.0);
                servoClaw2.setPosition(1.0);
            } else {
                servoClaw1.setPosition(0.5);
                servoClaw2.setPosition(0.5);
            }

            if (gamepad2.y) {
                servoClawRotation.setPosition(1);
            } else {
                servoClawRotation.setPosition(0);
            }

            //SLIDER
            //stick y is inverted in this variable. greater than 0.1 is  to go down and lesser than -0.1 is to go up
            float invertStick2R = -gamepad2.right_stick_y;
            if (invertStick2R <= -0.3) {
                elevatorMotor.setPower(0.65);
                elevatorMotor.setTargetPosition(SLIDER_RETRACTED);
            }else if (invertStick2R >= 0.3) {
                elevatorMotor.setPower(1);
                elevatorMotor.setTargetPosition(SLIDER_EXTENDED);
            }else {
                elevatorMotor.setTargetPosition(sliderPreviousPos);
                elevatorMotor.setPower(1);
                sliderPreviousPos = elevatorMotor.getCurrentPosition();
            }

            //ARM
            if (gamepad2.left_stick_y > 0.2) {
                armTargetPos += 1;
            } else if (gamepad2.left_stick_y < -0.2) {
                armTargetPos -= 1;
            }
            //armMotor.setPower(PIDF.update(armTargetPos, armMotor.getCurrentPosition()));

            //MOVEMENT
            x   = gamepad1.left_stick_x;
            y =  -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            turn =  gamepad1.right_stick_x;

            botHeading = getHeading();
            targetHeading = botHeading + turn;
            headingError = targetHeading - botHeading;
            rotationCorrection  = headingError * HEADING_GAIN;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotationCorrection), 1);
            if (turn > 0.2 || turn < -0.2) {
                frontLeftPower   = (y + x + turn) / denominator;
                backLeftPower    = (y - x + turn) / denominator;
                frontRightPower  = (y - x - turn) / denominator;
                backRightPower   = (y + x - turn) / denominator;
            } else {
                frontLeftPower = (y + x + rotationCorrection) / denominator;
                backLeftPower = (y - x + rotationCorrection) / denominator;
                frontRightPower = (y - x - rotationCorrection) / denominator;
                backRightPower = (y + x - rotationCorrection) / denominator;
            }

            frontLeft.setPower(frontLeftPower/2);
            backLeft.setPower(backLeftPower/2);
            frontRight.setPower(frontRightPower/2);
            backRight.setPower(backRightPower/2);

            String powersOfMotors = String.format("LeftFront: %.2f RightFront: %.2f LeftBack %.2f RightBack %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addLine(powersOfMotors);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", botHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Rotation Correction", rotationCorrection);
            telemetry.addData("Loop Times", runtime.milliseconds());
            runtime.reset();
            telemetry.update(); //Remember to update the telemetry or nothing is going to show
            previousBotHeading = botHeading;

        }
    }
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}

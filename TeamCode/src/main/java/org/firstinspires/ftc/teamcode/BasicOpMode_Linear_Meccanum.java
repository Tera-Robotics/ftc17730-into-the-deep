package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.Locale;

@Config
@TeleOp(name="TeleOp OpMode", group="Linear OpMode")
public class BasicOpMode_Linear_Meccanum extends LinearOpMode {
    private IMU imu;
    @SuppressWarnings("FieldMayBeFinal")
    private ElapsedTime runtime = new ElapsedTime();
    float x,y,turn;
    double botHeading, previousBotHeading, headingError, rotationCorrection;
    double denominator;
    double frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    double targetHeading = 0.0;
    int armTargetPos = 0;
    int armPreviousPosition = 0;
    public static float HEADING_GAIN = 0.0095f;  // Gain for heading correction
    public static double n1 = 0.0085;
    public static double n2 = 0;
    public static double n3 = 0.00005;
    public static double n4 = 0;
    CustomPIDFController PIDF = new CustomPIDFController(n1,n2,n3,n4);
    public static int SLIDER_EXTENDED = 2550;
    public static int SLIDER_RETRACTED = 0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        DcMotorEx lfDrive = hardwareMap.get(DcMotorEx .class, "lfDriveMotor");
        DcMotorEx lbDrive = hardwareMap.get(DcMotorEx.class, "lbDriveMotor");
        DcMotorEx rbDrive = hardwareMap.get(DcMotorEx.class, "rbDriveMotor");
        DcMotorEx rfDrive = hardwareMap.get(DcMotorEx.class, "rfDriveMotor");

        Servo servoClaw1 = hardwareMap.get(Servo.class, "servoClaw1");
        Servo servoClaw2 = hardwareMap.get(Servo.class, "servoClaw2");
        Servo servoClawRotation = hardwareMap.get(Servo.class, "servoClawRotation");
        Servo servoBalde = hardwareMap.get(Servo.class, "servoBalde");


        DcMotorEx armMotor = hardwareMap.get(DcMotorEx .class, "armMotor");
        DcMotorEx elevatorMotor = hardwareMap.get(DcMotorEx .class, "elevatorMotor");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int sliderPreviousPos = elevatorMotor.getCurrentPosition();
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lfDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lbDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rbDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rfDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        lfDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lbDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rbDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rfDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                servoClaw1.setPosition(1.0);
                servoClaw2.setPosition(0.0);
            } else if (gamepad1.b) {
                servoClaw1.setPosition(0.0);
                servoClaw2.setPosition(1.0);
            } else {
                servoClaw1.setPosition(0.5);
                servoClaw2.setPosition(0.5);
            }

            //BALDE
            if (gamepad2.x) {
                servoBalde.setPosition(1.0);
            } else if (gamepad2.b) {
                servoBalde.setPosition(0.0);
            } else {
                servoBalde.setPosition(0.5);
            }

            //Claw
            if (gamepad2.y) {
                servoClawRotation.setPosition(0.222);
            } else if (gamepad2.a) {
                servoClawRotation.setPosition(1);
            } else {
                servoClawRotation.setPosition(0.7);
            }
            //SLIDER
            //stick y is inverted in this variable. greater than 0.1 is  to go down and lesser than -0.1 is to go up
            float invertStick2R = -gamepad2.right_stick_y;
            if (invertStick2R <= -0.3) {
                elevatorMotor.setPower(0.5);
                elevatorMotor.setTargetPosition(SLIDER_RETRACTED);
            }else if (invertStick2R >= 0.3) {
                elevatorMotor.setPower(1);
                elevatorMotor.setTargetPosition(SLIDER_EXTENDED);
            }else {
                elevatorMotor.setTargetPosition(sliderPreviousPos);
                elevatorMotor.setPower(1);
                sliderPreviousPos = elevatorMotor.getCurrentPosition();
            }
            if (gamepad2.left_bumper) {
                elevatorMotor.setTargetPosition(armMotor.getCurrentPosition()-100);
                sleep(50);
                elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad2.right_bumper) {
                elevatorMotor.setTargetPosition(armMotor.getCurrentPosition()+100);
                sleep(50);
                elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            //ARM
            if (gamepad2.dpad_down) {
                armMotor.setPower(1 - Math.abs(armMotor.getCurrentPosition()*0.00025));
                armTargetPos += 2;
                armMotor.setTargetPosition(armTargetPos);
            } else if (gamepad2.dpad_up) {
                armMotor.setPower(0.85);
                armTargetPos -= 2;
                armMotor.setTargetPosition(armTargetPos);
            } else {
                armMotor.setPower(0.5);
                armMotor.setPower(armTargetPos);
            }
            double PIDFPower = PIDF.update(armTargetPos, armMotor.getCurrentPosition());
            /*
            if (gamepad2.left_stick_y > 0.2) {
                armTargetPos += 1;
            } else if (gamepad2.left_stick_y < -0.2) {
                armTargetPos -= 1;
            }
            if (armTargetPos > 10) {
                armMotor.setPower(Range.clip(PIDFPower, -1, 1));
            }
            if (armMotor.isOverCurrent()) {
                armMotor.setPower(0);
            }*/

            //MOVEMENT
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            turn = gamepad1.right_stick_x;

            botHeading = getHeading();
            headingError = targetHeading - botHeading;
            rotationCorrection  = headingError * HEADING_GAIN;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
            if (turn > 0.1 || turn < -0.1) { targetHeading = botHeading; }
            frontLeftPower   = ((y + x + turn) / denominator) - rotationCorrection;
            backLeftPower    = ((y - x + turn) / denominator) - rotationCorrection;
            frontRightPower  = ((y - x - turn) / denominator) + rotationCorrection;
            backRightPower   = ((y + x - turn) / denominator) + rotationCorrection;

            frontLeftPower  = Range.clip(frontLeftPower , -1, 1);
            backLeftPower   = Range.clip(backLeftPower  , -1, 1);
            frontRightPower = Range.clip(frontRightPower, -1, 1);
            backRightPower  = Range.clip(backRightPower , -1, 1);
            double powerDivision = 1.2;
            lfDrive.setPower(frontLeftPower/powerDivision);
            lbDrive.setPower(backLeftPower/powerDivision);
            rfDrive.setPower(frontRightPower/powerDivision);
            rbDrive.setPower(backRightPower/powerDivision);

            String powersOfMotors = String.format(Locale.getDefault(), "LeftFront: %.3f LeftBack %.3f \n RightFront: %.3f RightBack %.3f", lfDrive.getPower(), lbDrive.getPower(), rfDrive.getPower(), rbDrive.getPower());
            telemetry.addLine(powersOfMotors);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", botHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Rotation Correction", rotationCorrection);
            telemetry.addData("PIDF", PIDFPower);
            telemetry.addData("armMotor Position", armMotor.getCurrentPosition());
            telemetry.addData("armMotor Target Position", armMotor.getTargetPosition());
            telemetry.addData("ArmMotor Power", armMotor.getPower());
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

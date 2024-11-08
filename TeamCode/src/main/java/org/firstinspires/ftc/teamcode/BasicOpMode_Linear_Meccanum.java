package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class BasicOpMode_Linear_Meccanum extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx .class, "lfDriveMotor");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "lbDriveMotor");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rbDriveMotor");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rfDriveMotor");

        DcMotorEx armMotor = hardwareMap.get(DcMotorEx .class, "armMotor");
        DcMotorEx elevatorMotor = hardwareMap.get(DcMotorEx .class, "elevatorMotor");

        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Credits to: GavinFord https://www.youtube.com/watch?v=gnSW2QpkGXQ
            //For Some Reason the controls are... Skewed? idk why...
            double x   = gamepad1.left_stick_x;
            double y =  -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double turn =  gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin),Math.abs(cos));

            double leftFrontPower   = x-y;
            double leftRearPower    = power * sin/max - turn;
            double rightRearPower  = power * sin/max + turn;
            double rightFrontPower   = power * cos/max - turn;

            if ((power + Math.abs(turn)) > 1) {
                leftFrontPower  /= power + Math.abs(turn);
                rightFrontPower /= power + Math.abs(turn);
                leftRearPower   /= power + Math.abs(turn);
                rightRearPower  /= power + Math.abs(turn);
            }
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftRear.setPower(leftRearPower);
            rightRear.setPower(rightRearPower);
            String powersOfMotors = String.format("LeftFront: %.2f RightFront: %.2f LeftRear %.2f RightRear %.2f", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
            telemetry.addLine(powersOfMotors);
            telemetry.update(); //Remember to update the telemetry or nothing is going to show
        }
    }
}

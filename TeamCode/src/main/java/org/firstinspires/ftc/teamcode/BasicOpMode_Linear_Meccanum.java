package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class BasicOpMode_Linear_Meccanum extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx .class, "frontleft");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "backleft");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "backright");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "frontright");

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Credits to: GavinFord https://www.youtube.com/watch?v=gnSW2QpkGXQ
            double x   = gamepad1.left_stick_x;
            double y =  -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double turn =  gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin),Math.abs(cos));

            double leftFrontPower   = power * cos/max + Math.abs(turn);
            double leftRearPower    = power * sin/max - Math.abs(turn);
            double rightRearPower  = power * sin/max + Math.abs(turn);
            double rightFrontPower   = power * cos/max - Math.abs(turn);

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

            telemetry.update(); //Remember to update the telemetry or nothing is going to show
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestMotorDirections extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx .class, "lfDriveMotor");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "lbDriveMotor");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rbDriveMotor");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rfDriveMotor");


        DcMotorEx armMotor = hardwareMap.get(DcMotorEx .class, "armMotor");
        DcMotorEx elevatorMotor = hardwareMap.get(DcMotorEx .class, "elevatorMotor");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                leftFront.setPower(1);
            } else if (gamepad1.y) {
                leftRear.setPower(1);
            } else if (gamepad1.b) {
                rightFront.setPower(1);
            } else if (gamepad1.a) {
                rightRear.setPower(1);
            } else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }
            telemetry.addData("leftFront", gamepad1.x);
            telemetry.addData("leftRear", gamepad1.y);
            telemetry.addData("rightFront", gamepad1.b);
            telemetry.addData("rightRear", gamepad1.a);
            telemetry.update();
        }
    }
}

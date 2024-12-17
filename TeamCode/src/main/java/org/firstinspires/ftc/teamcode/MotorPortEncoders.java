package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MotorPortEncoders extends LinearOpMode {
    HardwareMecanum drive = new HardwareMecanum(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("leftFront", drive.lfDrive.getCurrentPosition());
            telemetry.addData("leftBack", drive.lbDrive.getCurrentPosition());
            telemetry.addData("rightFront", drive.rfDrive.getCurrentPosition());
            telemetry.addData("rightBack", drive.rbDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}

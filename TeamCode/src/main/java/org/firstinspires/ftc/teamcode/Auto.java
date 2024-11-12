package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.purepursuit.Waypoint;

import java.util.List;

@Autonomous(name = "Auto", group = "into-the-deep", preselectTeleOp = "")
public class Auto extends OpMode {
    private IMU imu_IMU;
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;
    Localization localization = new Localization(0, 0, 0);
    Waypoint w1 = new Waypoint(1,0,0);
    Waypoint w2 = new Waypoint(1,1,0);
    Waypoint w3 = new Waypoint(0,1,0);
    Waypoint w4 = new Waypoint(0,0,0);
    @Override
    public void init() {
        elapsedtime = new ElapsedTime();

        // this just sets the bulk reading mode for each hub
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        leftFront = hardwareMap.get(DcMotorEx .class, "lfDriveMotor");
        leftBack = hardwareMap.get(DcMotorEx.class, "lbDriveMotor");
        rightBack = hardwareMap.get(DcMotorEx.class, "rbDriveMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "rfDriveMotor");

        elapsedtime.reset();
    }

    @Override
    public void loop() {
        // clears the cache on each hub
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        double power = Math.hypot(w1.getX(), w1.getY());

        telemetry.addData("leftBackDrive", leftBack.getCurrentPosition());
        telemetry.addData("leftFrontDrive", leftFront.getCurrentPosition());
        telemetry.addData("rightBackDrive", rightBack.getCurrentPosition());
        telemetry.addData("rightFrontDrive", rightFront.getCurrentPosition());
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
    }
}

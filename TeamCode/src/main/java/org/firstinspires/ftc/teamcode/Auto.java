package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.actions.InterruptAction;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@Photon
@Autonomous(name = "Auto", group = "A", preselectTeleOp = "")
public class Auto extends OpMode {
    private IMU imu_IMU;
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;

    private PIDFController lfDrivePIDF = new PIDFController(10,0,0,0);
    private PIDFController lbDrivePIDF = new PIDFController(10,0,0,0);
    private PIDFController rfDrivePIDF = new PIDFController(10,0,0,0);
    private PIDFController rbDrivePIDF = new PIDFController(10,0,0,0);

    @Override
    public void init() {
        elapsedtime = new ElapsedTime();

        // this just sets the bulk reading mode for each hub
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        leftFront = hardwareMap.get(DcMotorEx.class, "frontleft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backleft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backright");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontright");

        elapsedtime.reset();

        Waypoint p1 = new StartWaypoint(new Pose2d(0,0, new Rotation2d(0)));
        Waypoint p2 = new GeneralWaypoint(20, 0);

        Waypoint p4 = new InterruptWaypoint(
                30, 0, 0.5,
                0.3, 1,
                0, 0, new InterruptAction() {
            @Override
            public void doAction() {

            }
        }
        );
        // we are using the waypoints we made in the above examples
        Path m_path = new Path(p1, p2);

        m_path.init(); // initialize the path
    }

    @Override
    public void loop() {
        // clears the cache on each hub
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // after the first time, it won't actually send new commands


        telemetry.addData("Motor Position", leftBack.getCurrentPosition());
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
    }

    private void DriveStraigh() {

    }
}

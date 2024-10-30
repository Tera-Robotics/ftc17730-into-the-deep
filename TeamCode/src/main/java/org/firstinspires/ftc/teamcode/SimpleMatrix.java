package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class SimpleMatrix extends OpMode {
    double[] initialEncoderValues;
    double[] currentEncoderValues;
    double[][] qMatrix;
    double[][] rMatrix;
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        KalmanFilterEncoders kf = new KalmanFilterEncoders(initialEncoderValues, qMatrix, rMatrix);
        currentEncoderValues = [0,0,0,0];
        kf.updateAsync(currentEncoderValues);
        kf.getState();
    }
}

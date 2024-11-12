package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class CustomPIDFController {
    private double Kp, Kd, Ki;
    private double error, lastError;
    private double derivative = 0;
    private double integralSum = 0;
    private double Kf;
    ElapsedTime timer = new ElapsedTime();
    public CustomPIDFController(double newkP, double newkD, double newKi, double newKf) {
        this.Kp = newkP;
        this.Kd = newkD;
        this.Ki = newKi;
        this.Kf = newKf;
    }
    public double update(double target, int currentPosition) {
        error = target - currentPosition;
        derivative = (error - lastError) / timer.seconds();
        integralSum += (error * timer.seconds());

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;
        timer.reset();
        return out+Kf;
    }
}

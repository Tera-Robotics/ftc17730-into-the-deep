package org.firstinspires.ftc.teamcode;

public class SimpleKalmanFilter {
    public SimpleKalmanFilter() {
    }
    private final double initialEstimate = 0; // your initial state
    private double currentErrorEstimate = 0.1; // your model covariance
    private double errorMeasurement = 0.4; // your sensor covariance
    private double initialMeasurement = 0; // your initial covariance guess
    private double kalmanGain = 1; // your initial Kalman gain guess

    private double previousErrorEstimate = initialEstimate;
    private double previousErrorMeasurement = initialMeasurement;
    private double controlVariableMatrix = 0;
    private double z = 0;

    private double currentEstimate = 0;
    private double previousEstimate = currentEstimate;
    private double currentMeasurement = 0;
    private double previousMeasurement = currentMeasurement;
    public void predict() {

    }
    public double update() {
        //Calculate the Kalman Gain
        kalmanGain = currentErrorEstimate / (currentErrorEstimate + errorMeasurement);

        //Calculate Current Estimate
        currentEstimate = previousEstimate + kalmanGain * (currentMeasurement - previousMeasurement);

        //Calculate new Error Estimate
        currentErrorEstimate = (1 - kalmanGain) * previousErrorEstimate;

        previousErrorEstimate = currentErrorEstimate;
        previousEstimate = currentEstimate;
        previousErrorMeasurement = currentMeasurement;

        return currentEstimate;
    }
}
package org.firstinspires.ftc.teamcode;

import java.util.Objects;
import java.util.Scanner;

public class KalmanFilter {
    public static void main(String[] args) {
        Scanner keyboard = new Scanner(System.in);

        double initialEstimate = 0; // your initial state
        double errorEstimate = 0.1; // your model covariance
        double errorMeasurement = 0.4; // your sensor covariance
        double initialMeasurement = 0; // your initial covariance guess
        double kalmanGain = 1; // your initial Kalman gain guess

        double previousErrorEstimate = initialEstimate;
        double previousErrorMeasurement = initialMeasurement;
        double controlVariableMatrix = 0;
        double z = 0;

        double currentEstimate = 0;
        double previousEstimate = currentEstimate;
        double currentMeasurement = 0;
        double previousMeasurement = currentMeasurement;
        while (keyboard.hasNext("Q")) {

            //Calculate the Kalman Gain
            kalmanGain = errorEstimate / errorEstimate + errorMeasurement;

            //Calculate Current Estimate
            currentEstimate = previousEstimate + kalmanGain * (currentMeasurement - previousMeasurement);

            //Calculate new Error Estimate
            errorEstimate = (1 - kalmanGain) * previousErrorEstimate;

            System.out.println(String.format("KG:%.3f currentEstimate:%.3f errorEstimate:%.3f", kalmanGain, currentEstimate, errorEstimate));
        }
    }
}

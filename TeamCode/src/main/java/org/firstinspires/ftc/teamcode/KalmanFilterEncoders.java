package org.firstinspires.ftc.teamcode;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class KalmanFilterEncoders {
    // Define the matrices needed for the Kalman filter
    private SimpleMatrix F; // State transition matrix
    //private SimpleMatrix B; // Control matrix (not used)
    private SimpleMatrix C; // Measurement matrix
    private SimpleMatrix Q; // Process noise covariance matrix
    private SimpleMatrix R; // Measurement noise covariance matrix
    private SimpleMatrix P; // Error covariance matrix
    private SimpleMatrix x; // State estimate (encoder values)

    // Constructor to initialize the filter with given parameters
    public KalmanFilterEncoders(double[] initialEncoderValues, double[][] qMatrix, double[][] rMatrix) {
        // Initialize the state vector with the initial encoder values
        this.x = new SimpleMatrix(new DMatrixRMaj(initialEncoderValues.length, 1, true, initialEncoderValues));

        // Initialize the state transition matrix A as an identity matrix
        this.F = SimpleMatrix.identity(4);

        // Initialize B as a zero matrix (no control input)
        //this.B = new SimpleMatrix(4, 1);

        // Measurement matrix C as an identity matrix
        this.C = SimpleMatrix.identity(4);

        // Set process noise covariance matrix Q and measurement noise covariance matrix R
        this.Q = new SimpleMatrix(new DMatrixRMaj(qMatrix));
        this.R = new SimpleMatrix(new DMatrixRMaj(rMatrix));

        // Initialize error covariance matrix P as an identity matrix
        this.P = SimpleMatrix.identity(4);
    }

    public void setState(DMatrixRMaj x, DMatrixRMaj P) {
        this.x = new SimpleMatrix(x);
        this.P = new SimpleMatrix(P);
    }

    public void predict() {
        // Project the state forward: x = A * x
        x = F.mult(x);

        // Project the error covariance forward: P = A * P * A' + Q
        P = F.mult(P).mult(F.transpose()).plus(Q);
    }

    private ExecutorService executor = Executors.newSingleThreadExecutor();

    public void updateAsync(double[] measurement) {
        executor.submit(() -> update(measurement));
    }

    public void shutdown() {
        executor.shutdown();
    }
    public void update(double[] measurement) {
        /*
        @param measurement -- Reading of the Encoder Values
         */
        // Wrap the measurement array in a SimpleMatrix
        SimpleMatrix z = new SimpleMatrix(new DMatrixRMaj(measurement.length, 1, true, measurement));

        // Compute the Kalman Gain: K = P * C' * (C * P * C' + R)^(-1)
        SimpleMatrix S = C.mult(P).mult(C.transpose()).plus(R);
        SimpleMatrix K = P.mult(C.transpose().mult(S.invert()));

        // Compute innovation: y = z - C * x
        SimpleMatrix y = z.minus(C.mult(x));

        // Update state estimate: x = x + K * y
        x = x.plus(K.mult(y));

        // Update error covariance: P = (I - K * C) * P
        SimpleMatrix I = SimpleMatrix.identity(P.numRows());
        P = (I.minus(K.mult(C))).mult(P);
    }

    // Method to get the current encoder estimates (post-update)
    public synchronized double[] getEstimatedEncoders() {
        return x.getDDRM().getData();
    }
    public DMatrixRMaj getState() { return x.getMatrix(); }

    public DMatrixRMaj getCovariance() { return P.getMatrix(); }
}

    /*
    // Define the matrices needed for the Kalman filter
    private final double[][] A; // State transition matrix
    private final double[][] B; // Control matrix (unused, set to zero)
    private final double[][] C; // Measurement matrix
    private final double[][] Q; // Process noise covariance matrix
    private final double[][] R; // Measurement noise covariance matrix
    private double[][] P;       // Error covariance matrix
    private double[] x;         // State estimate (encoder values)

    // Constructor to initialize the filter with given parameters
    public KalmanFilterEncoders(double[] initialEncoderValues, double[][] qMatrix, double[][] rMatrix) {
        // Initialize the state vector with the initial encoder values
        this.x = initialEncoderValues.clone();

        // Initialize the state transition matrix A as an identity matrix
        // because each encoder state is expected to retain its previous value
        this.A = new double[][]{
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        };

        // Set B to a zero matrix because there is no external control input
        this.B = new double[][]{
                {0}, {0}, {0}, {0}
        };

        // Measurement matrix C is also an identity matrix because the measurement
        // corresponds directly to the encoder values without transformation
        this.C = new double[][]{
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        };

        // Process noise covariance matrix Q and measurement noise covariance R are defined externally
        // Q represents model uncertainty, R represents sensor measurement uncertainty
        this.Q = qMatrix;
        this.R = rMatrix;

        // Initialize the error covariance matrix P as an identity matrix with low uncertainty
        this.P = new double[][]{
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        };
    }
    // Predict phase of Kalman filter, to project the state and covariance forward
    public void predict() {
        // x' = A * x  -> project the state forward using the state transition matrix
        x = matrixMultiply(A, x);

        // P' = A * P * A^T + Q  -> project the error covariance forward
        P = matrixAdd(matrixMultiply(matrixMultiply(A, P), transpose(A)), Q);
    }
    // Update phase of Kalman filter, to incorporate the new measurement
    public void update(double[] measurement) {
        // Compute the Kalman Gain K = P * C^T * (C * P * C^T + R)^-1
        double[][] Ct = transpose(C);
        double[][] S = matrixAdd(matrixMultiply(matrixMultiply(C, P), Ct), R);
        double[][] K = matrixMultiply(matrixMultiply(P, Ct), inverse(S));

        // y = measurement - C * x  -> innovation or measurement residual
        double[] y = matrixSubtract(measurement, matrixMultiply(C, x));

        // Update the state estimate: x = x + K * y
        x = matrixAdd(x, matrixMultiply(K, y));

        // Update the error covariance: P = (I - K * C) * P
        double[][] I = identityMatrix(P.length);  // Identity matrix of same size as P
        P = matrixMultiply(matrixSubtract(I, matrixMultiply(K, C)), P);
    }

    // Method to get the current encoder estimates (post-update)
    public double[] getEstimatedEncoders() {
        return x;
    }

    // Matrix operations utility methods

    // Matrix multiplication: multiplies a matrix by a vector
    private double[] matrixMultiply(double[][] mat, double[] vec) {
        // Result vector
        double[] result = new double[mat.length];
        for (int i = 0; i < mat.length; i++) {
            for (int j = 0; j < vec.length; j++) {
                result[i] += mat[i][j] * vec[j];
            }
        }
        return result;
    }

    // Matrix addition
    private double[][] matrixAdd(double[][] mat1, double[][] mat2) {
        double[][] result = new double[mat1.length][mat1[0].length];
        for (int i = 0; i < mat1.length; i++) {
            for (int j = 0; j < mat1[i].length; j++) {
                result[i][j] = mat1[i][j] + mat2[i][j];
            }
        }
        return result;
    }

    // Matrix subtraction
    private double[] matrixSubtract(double[] vec1, double[] vec2) {
        double[] result = new double[vec1.length];
        for (int i = 0; i < vec1.length; i++) {
            result[i] = vec1[i] - vec2[i];
        }
        return result;
    }

    // Transpose a matrix
    private double[][] transpose(double[][] mat) {
        double[][] transposed = new double[mat[0].length][mat.length];
        for (int i = 0; i < mat.length; i++) {
            for (int j = 0; j < mat[i].length; j++) {
                transposed[j][i] = mat[i][j];
            }
        }
        return transposed;
    }

    // Identity matrix generation
    private double[][] identityMatrix(int size) {
        double[][] identity = new double[size][size];
        for (int i = 0; i < size; i++) {
            identity[i][i] = 1;
        }
        return identity;
    }

    // Dummy matrix inversion function (replace with actual inversion logic)
    private double[][] inverse(double[][] mat) {
        // Add inversion logic here, using a suitable matrix library or manual implementation
        return new double[mat.length][mat[0].length];  // Placeholder return

    }

}
*/
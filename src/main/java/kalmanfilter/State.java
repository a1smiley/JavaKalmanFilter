package kalmanfilter;

import lombok.Getter;
import org.ejml.simple.SimpleMatrix;

@Getter
class State {
    private double[] statePrediction;
    private double[] stateEstimate;
    private double[][] covariancePrediction;
    private double[][] covarianceEstimate;
    private int timeStep = 0;
    private final EstimateTimeSeries stateHistory = new EstimateTimeSeries();

    State(double[] initialState, double[][] initialCovariance) {
        this.statePrediction = initialState;
        this.stateEstimate = initialState;
        this.covariancePrediction = initialCovariance;
        this.covarianceEstimate = initialCovariance;
    }

    void updateStatePrediction(double[] newStatePrediction) throws KalmanFilterException {
        if (noPredictionStoredForCurrentTimeStep()) {
            this.statePrediction = newStatePrediction;
            stateHistory.storeStatePrediction(timeStep, new DoubleArray(newStatePrediction));
        } else {
            throw new KalmanFilterException();
        }
    }

    void updateStateEstimate(double[] newStateEstimate) throws KalmanFilterException {
        if (noEstimateStoredForCurrentTimeStep()) {
            this.stateEstimate = newStateEstimate;
            stateHistory.storeStateEstimate(timeStep, new DoubleArray(newStateEstimate));
        } else {
            throw new KalmanFilterException();
        }
    }

    void updateCovariancePrediction(SimpleMatrix newCovPrediction) throws KalmanFilterException {
        if (noCovPredictionStoredForCurrentTimeStep()) {
            this.covariancePrediction = getPrimitive2DArray(newCovPrediction);
            stateHistory.storeCovariancePrediction(timeStep, new Double2DArray(this.covariancePrediction));
        } else {
            throw new KalmanFilterException();
        }
    }

    void updateCovarianceEstimate(SimpleMatrix newCovEstimate) throws KalmanFilterException {
        if (noCovEstimateStoredForCurrentTimeStep()) {
            this.covarianceEstimate = getPrimitive2DArray(newCovEstimate);
            stateHistory.storeCovarianceEstimate(timeStep, new Double2DArray(this.covarianceEstimate));
        } else {
            throw new KalmanFilterException();
        }
    }

    void incrementTimeStep() {
        timeStep++;
    }

    private boolean noPredictionStoredForCurrentTimeStep() {
        if (stateHistory.getStatePrediction(this.timeStep) == null) {
            return true;
        } else {
            return false;
        }
    }

    private boolean noEstimateStoredForCurrentTimeStep() {
        if (stateHistory.getStateEstimate(this.timeStep) == null) {
            return true;
        } else {
            return false;
        }
    }

    private boolean noCovPredictionStoredForCurrentTimeStep() {
        if (stateHistory.getCovariancePrediction(this.timeStep) == null) {
            return true;
        } else {
            return false;
        }
    }

    private boolean noCovEstimateStoredForCurrentTimeStep() {
        if (stateHistory.getCovarianceEstimate(this.timeStep) == null) {
            return true;
        } else {
            return false;
        }
    }

    private double[][] getPrimitive2DArray(SimpleMatrix simpleMatrix) {
        int nCols = simpleMatrix.numCols();
        int nRows = simpleMatrix.numRows();
        double[] primitive1DArray = simpleMatrix.getMatrix().getData();
        double[][] primitive2DArray = new double[nCols][nRows];

        for (int i = 0; i < nCols; i++) {
            //noinspection ManualArrayCopy
            for (int j = 0; j < nRows; j++) {
                primitive2DArray[i][j] = primitive1DArray[i * nCols + j];
            }
        }
        return primitive2DArray;
    }

}

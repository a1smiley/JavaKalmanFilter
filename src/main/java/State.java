import lombok.Getter;

@Getter
public class State {
    private int timeStep = 0;
    private double[] statePrediction;
    private double[] stateEstimate;
    private double[][] covariancePrediction;
    private double[][] covarianceEstimate;
    private EstimateTimeSeries stateHistory;

    public State(double[] initialState, double[][] initialCovariance) {
        this.statePrediction = initialState;
        this.stateEstimate = initialState;
        this.covariancePrediction = initialCovariance;
        this.covarianceEstimate = initialCovariance;
    }

    public void updateStatePrediction(double[] newStatePrediction) throws KalmanFilterException {
        if (noPredictionStoredForCurrentTimeStep()) {
            this.statePrediction = newStatePrediction;
            stateHistory.storeStatePrediction(timeStep, new DoubleArray(newStatePrediction));
        } else {
            throw new KalmanFilterException();
        }
    }

    public void updateStateEstimate(double[] newStateEstimate) throws KalmanFilterException {
        if (noEstimateStoredForCurrentTimeStep()) {
            this.stateEstimate = newStateEstimate;
            stateHistory.storeStateEstimate(timeStep, new DoubleArray(newStateEstimate));
        } else {
            throw new KalmanFilterException();
        }
    }

    public void updateCovariancePrediction(double[][] newCovPrediction) throws KalmanFilterException {
        if (noCovPredictionStoredForCurrentTimeStep()) {
            this.covariancePrediction = newCovPrediction;
            stateHistory.storeCovariancePrediction(timeStep, new Double2DArray(newCovPrediction));
        } else {
            throw new KalmanFilterException();
        }
    }

    public void updateCovarianceEstimate(double[][] newCovEstimate) throws KalmanFilterException {
        if (noCovEstimateStoredForCurrentTimeStep()) {
            this.covarianceEstimate = newCovEstimate;
            stateHistory.storeCovarianceEstimate(timeStep, new Double2DArray(newCovEstimate));
        } else {
            throw new KalmanFilterException();
        }
    }

    public void incrementTimeStep() {
        timeStep++;
    }

    private boolean noCovPredictionStoredForCurrentTimeStep() {
        try {
            stateHistory.getCovariancePrediction(this.timeStep);
            return false;
        } catch (NullPointerException e) {
            return true;
        }
    }

    private boolean noCovEstimateStoredForCurrentTimeStep() {
        try {
            stateHistory.getCovarianceEstimate(this.timeStep);
            return false;
        } catch (NullPointerException e) {
            return true;
        }
    }

    private boolean noPredictionStoredForCurrentTimeStep() {
        try {
            stateHistory.getStatePrediction(this.timeStep);
            return false;
        } catch (NullPointerException e) {
            return true;
        }
    }

    private boolean noEstimateStoredForCurrentTimeStep() {
        try {
            stateHistory.getStateEstimate(this.timeStep);
            return false;
        } catch (NullPointerException e) {
            return true;
        }
    }

}

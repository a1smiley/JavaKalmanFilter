package kalmanfilter;

import lombok.Getter;

import java.util.HashMap;

@Getter
class EstimateTimeSeries {
    private final HashMap<Integer, DoubleArray> statePredictionHistory = new HashMap<>();
    private final HashMap<Integer, DoubleArray> stateEstimateHistory = new HashMap<>();
    private final HashMap<Integer, DoubleArray> outputPredictionHistory = new HashMap<>();
    private final HashMap<Integer, DoubleArray> innovationHistory = new HashMap<>();
    private final HashMap<Integer, Double2DArray> covariancePredictionHistory = new HashMap<>();
    private final HashMap<Integer, Double2DArray> covarianceEstimateHistory = new HashMap<>();
    private final HashMap<Integer, Double2DArray> gainHistory = new HashMap<>();

    EstimateTimeSeries() {

    }

    void storeState(int timeStep, State state) throws KalmanFilterException {
        storeStatePrediction(timeStep, new DoubleArray(state.getStatePrediction()));
        storeCovariancePrediction(timeStep, new Double2DArray(state.getCovariancePrediction()));
        storeStateEstimate(timeStep, new DoubleArray(state.getStateEstimate()));
        storeCovarianceEstimate(timeStep, new Double2DArray(state.getCovarianceEstimate()));
    }

    void storeStatePrediction(int timeStep, DoubleArray newStatePrediction) throws KalmanFilterException {
        if (noPredictionStoredForCurrentTimeStep(timeStep)) {
            this.statePredictionHistory.put(timeStep, newStatePrediction);
        } else {
            throw new KalmanFilterException("State prediction value already stored for time index " + timeStep);
        }
    }

    void storeStateEstimate(int timeStep, DoubleArray newStateEstimate) throws KalmanFilterException {
        if (noEstimateStoredForCurrentTimeStep(timeStep)) {
            this.stateEstimateHistory.put(timeStep, newStateEstimate);
        } else {
            throw new KalmanFilterException("State estimate value already stored for time index " + timeStep);
        }
    }

    void storeCovariancePrediction(int timeStep, Double2DArray newCovPrediction) throws KalmanFilterException {
        if (noCovPredictionStoredForCurrentTimeStep(timeStep)) {
            this.covariancePredictionHistory.put(timeStep, newCovPrediction);
        } else {
            throw new KalmanFilterException("Covariance prediction value already stored for time index " + timeStep);
        }
    }

    void storeCovarianceEstimate(int timeStep, Double2DArray newCovEstimate) throws KalmanFilterException {
        if (noCovEstimateStoredForCurrentTimeStep(timeStep)) {
            this.covarianceEstimateHistory.put(timeStep, newCovEstimate);
        } else {
            throw new KalmanFilterException("Covariance estimate value already stored for time index " + timeStep);
        }
    }

    void storeOutputPrediction(int timeStep, DoubleArray newOutputEstimate) throws KalmanFilterException {
        if (noOutputPredictionStoredForCurrentTimeStep(timeStep)) {
            this.outputPredictionHistory.put(timeStep, newOutputEstimate);
        } else {
            throw new KalmanFilterException("Output estimate value already stored for time index " + timeStep);
        }
    }

    void storeInnovation(int timeStep, DoubleArray newInnovation) throws KalmanFilterException {
        if (noInnovationStoredForCurrentTimeStep(timeStep)) {
            this.innovationHistory.put(timeStep, newInnovation);
        } else {
            throw new KalmanFilterException("Innovation value already stored for time index " + timeStep);
        }
    }

    void storeGain(int timeStep, Double2DArray newGain) throws KalmanFilterException {
        if (noGainStoredForCurrentTimeStep(timeStep)) {
            this.gainHistory.put(timeStep, newGain);
        } else {
            throw new KalmanFilterException("Gain value already stored for time index " + timeStep);
        }
    }

    private boolean noPredictionStoredForCurrentTimeStep(int timeStep) {
        if (getStatePrediction(timeStep) == null) {
            return true;
        } else {
            return false;
        }
    }

    private boolean noEstimateStoredForCurrentTimeStep(int timeStep) {
        if (getStateEstimate(timeStep) == null) {
            return true;
        } else {
            return false;
        }
    }

    private boolean noCovPredictionStoredForCurrentTimeStep(int timeStep) {
        if (getCovariancePrediction(timeStep) == null) {
            return true;
        } else {
            return false;
        }
    }

    private boolean noCovEstimateStoredForCurrentTimeStep(int timeStep) {
        if (getCovarianceEstimate(timeStep) == null) {
            return true;
        } else {
            return false;
        }
    }

    private boolean noOutputPredictionStoredForCurrentTimeStep(int timeStep) {
        if (getOutputPrediction(timeStep) == null) {
            return true;
        } else {
            return false;
        }
    }

    private boolean noGainStoredForCurrentTimeStep(int timeStep) {
        if (getGain(timeStep) == null) {
            return true;
        } else {
            return false;
        }
    }

    private boolean noInnovationStoredForCurrentTimeStep(int timeStep) {
        if (getInnovation(timeStep) == null) {
            return true;
        } else {
            return false;
        }
    }

    DoubleArray getStatePrediction(int timeStep) {
        return this.statePredictionHistory.get(timeStep);
    }

    DoubleArray getStateEstimate(int timeStep) {
        return this.stateEstimateHistory.get(timeStep);
    }

    DoubleArray getOutputPrediction(int timeStep) {
        return this.outputPredictionHistory.get(timeStep);
    }

    DoubleArray getInnovation(int timeStep) {
        return this.innovationHistory.get(timeStep);
    }

    Double2DArray getCovariancePrediction(int timeStep) {
        return this.covariancePredictionHistory.get(timeStep);
    }

    Double2DArray getCovarianceEstimate(int timeStep) {
        return this.covarianceEstimateHistory.get(timeStep);
    }

    Double2DArray getGain(int timeStep) {
        return this.gainHistory.get(timeStep);
    }

}

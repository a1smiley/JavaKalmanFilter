package kalmanfilter;

import lombok.Getter;

import java.util.HashMap;

@Getter
class EstimateTimeSeries {
    private final HashMap<Integer, DoubleArray> statePredictionHistory = new HashMap<>();
    private final HashMap<Integer, DoubleArray> stateEstimateHistory = new HashMap<>();
    private final HashMap<Integer, Double2DArray> covariancePredictionHistory = new HashMap<>();
    private final HashMap<Integer, Double2DArray> covarianceEstimateHistory = new HashMap<>();

    EstimateTimeSeries() {

    }

    void storeStatePrediction(int timestep, DoubleArray newStatePrediction) {
        this.statePredictionHistory.put(timestep, newStatePrediction);
    }

    void storeStateEstimate(int timestep, DoubleArray newStateEstimate) {
        this.stateEstimateHistory.put(timestep, newStateEstimate);
    }

    void storeCovariancePrediction(int timestep, Double2DArray newCovPrediction) {
        this.covariancePredictionHistory.put(timestep, newCovPrediction);
    }

    void storeCovarianceEstimate(int timestep, Double2DArray newCovEstimate) {
        this.covarianceEstimateHistory.put(timestep, newCovEstimate);
    }

    DoubleArray getStatePrediction(int timestep) {
        return this.statePredictionHistory.get(timestep);
    }

    DoubleArray getStateEstimate(int timestep) {
        return this.stateEstimateHistory.get(timestep);
    }

    Double2DArray getCovariancePrediction(int timestep) {
        return this.covariancePredictionHistory.get(timestep);
    }

    Double2DArray getCovarianceEstimate(int timestep) {
        return this.covarianceEstimateHistory.get(timestep);
    }
}

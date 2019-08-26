package kalmanfilter;

import java.util.HashMap;

public class EstimateTimeSeries {
    private HashMap<Integer, DoubleArray> statePredictionHistory = new HashMap<>();
    private HashMap<Integer, DoubleArray> stateEstimateHistory = new HashMap<>();
    private HashMap<Integer, Double2DArray> covariancePredictionHistory = new HashMap<>();
    private HashMap<Integer, Double2DArray> covarianceEstimateHistory = new HashMap<>();

    public EstimateTimeSeries() {

    }

    public void storeStatePrediction(int timestep, DoubleArray newStatePrediction) {
        this.statePredictionHistory.put(timestep, newStatePrediction);
    }

    public void storeStateEstimate(int timestep, DoubleArray newStateEstimate) {
        this.stateEstimateHistory.put(timestep, newStateEstimate);
    }

    public void storeCovariancePrediction(int timestep, Double2DArray newCovPrediction) {
        this.covariancePredictionHistory.put(timestep, newCovPrediction);
    }

    public void storeCovarianceEstimate(int timestep, Double2DArray newCovEstimate) {
        this.covarianceEstimateHistory.put(timestep, newCovEstimate);
    }

    public DoubleArray getStatePrediction(int timestep) {
        return this.statePredictionHistory.get(timestep);
    }

    public DoubleArray getStateEstimate(int timestep) {
        return this.stateEstimateHistory.get(timestep);
    }

    public Double2DArray getCovariancePrediction(int timestep) {
        return this.covariancePredictionHistory.get(timestep);
    }

    public Double2DArray getCovarianceEstimate(int timestep) {
        return this.covarianceEstimateHistory.get(timestep);
    }
}

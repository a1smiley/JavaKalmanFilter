import lombok.Data;

import java.util.HashMap;

@Data
public class State {
    private int timeStep = 0;
    private double[] statePrediction;
    private double[] stateEstimate;
    private HashMap<Integer, DoubleArray> predictionHistory = new HashMap<>();
    private HashMap<Integer, DoubleArray> estimateHistory = new HashMap<>();

    public State(double[] initialState) {
        this.statePrediction = initialState;
        this.stateEstimate = initialState;
    }

    public void updateStatePrediction(double[] newStatePrediction) throws KalmanFilterException {
        if (noPredictionStoredForCurrentTimeStep()) {
            this.statePrediction = newStatePrediction;
            predictionHistory.put(timeStep, new DoubleArray(newStatePrediction));
        } else {
            throw new KalmanFilterException();
        }
    }

    public void updateStateEstimate(double[] newStateEstimate) throws KalmanFilterException {
        if (noEstimateStoredForCurrentTimeStep()) {
            this.stateEstimate = newStateEstimate;
            estimateHistory.put(timeStep, new DoubleArray(newStateEstimate));
        } else {
            throw new KalmanFilterException();
        }
    }

    public void incrementTimeStep() {
        timeStep++;
    }

    private boolean noPredictionStoredForCurrentTimeStep() {
        try {
            predictionHistory.get(this.timeStep);
            return false;
        } catch (NullPointerException e) {
            return true;
        }
    }

    private boolean noEstimateStoredForCurrentTimeStep() {
        try {
            estimateHistory.get(this.timeStep);
            return false;
        } catch (NullPointerException e) {
            return true;
        }
    }

}

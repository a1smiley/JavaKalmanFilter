import org.springframework.stereotype.Component;

@Component
public class LinearKalmanFilter {

    StateSpace systemModel;

    public EstimateTimeSeries filter(MeasurementSet measurements) throws KalmanFilterException {
        checkAllFieldsInitialized();

        double outputPrediction;
        double[] gainMatrix;
        for (int i = 0; i < measurements.duration; i++){
            double input = measurements.getInputMeasurement(i);
            generateStatePrediction(input);
            generateCovariancePrediction();
            outputPrediction = generateOutputPrediction(input);
            gainMatrix = generateKalmanGainMatrix();
            generateStateEstimate(outputPrediction, gainMatrix);
            generateCovarianceEstimate(gainMatrix);
            incrementTimeStep();
        }
        return getStateEstimateTimeSeries();
    }

    private void generateStatePrediction(double input) throws KalmanFilterException {
        systemModel.updateStatePrediction(input);
    }

    private void generateCovariancePrediction() throws KalmanFilterException{
        systemModel.updateCovariancePrediction();
    }

    private double generateOutputPrediction(double input) {
        return systemModel.generateOutputPrediction(input);
    }

    private double[] generateKalmanGainMatrix(){
        return systemModel.generateKalmanGainMatrix();
    }

    private void generateStateEstimate(double outputPrediction, double[] gainMatrix) throws KalmanFilterException{
        systemModel.updateStateEstimate(outputPrediction, gainMatrix);
    }

    private void generateCovarianceEstimate(double[] gainMatrix){
        systemModel.updateCovarianceEstimate(gainMatrix);
    }

    private void incrementTimeStep() {
        systemModel.incrementTimeStep();
    }

    private void checkAllFieldsInitialized() throws KalmanFilterException {
        if (systemModel == null) {
            throw new KalmanFilterException("System model must be defined prior to evaluating filter");
        }
    }

    public void setSystemModel(StateSpace systemModel) {
        this.systemModel = systemModel;
    }

    public EstimateTimeSeries getStateEstimateTimeSeries(){
        return this.systemModel.getStateHistory();
    }

}

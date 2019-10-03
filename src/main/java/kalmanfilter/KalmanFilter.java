package kalmanfilter;

import org.ejml.simple.SimpleMatrix;
import org.springframework.stereotype.Component;

@Component
class KalmanFilter {

    private StateSpace systemModel;

    EstimateTimeSeries filter(MeasurementSet measurements) throws KalmanFilterException {
        double[] outputPrediction;
        SimpleMatrix gainMatrix;

        checkAllFieldsInitialized();
        for (int i = 0; i < measurements.getDuration()-1; i++){
            double[] previousInput = measurements.getInputMeasurement(i);
            double[] currentInput = measurements.getInputMeasurement(i+1);
            generateStatePrediction(previousInput);
            generateCovariancePrediction();
            outputPrediction = generateOutputPrediction(currentInput);
            gainMatrix = generateKalmanGainMatrix();
            double[] innovation = calculateInnovation(measurements.getOutputMeasurement(i+1), outputPrediction);
            generateStateEstimate(innovation, gainMatrix);
            generateCovarianceEstimate(gainMatrix);
            incrementTimeStep();
        }

        return getStateEstimateTimeSeries();
    }

    private void generateStatePrediction(double[] input) throws KalmanFilterException {
        systemModel.updateStatePrediction(input);
    }

    private void generateCovariancePrediction() throws KalmanFilterException{
        systemModel.updateCovariancePrediction();
    }

    private double[] generateOutputPrediction(double[] input) {
        return systemModel.generateOutputPrediction(input);
    }

    private SimpleMatrix generateKalmanGainMatrix(){
        return systemModel.generateKalmanGainMatrix();
    }

    private double[] calculateInnovation(double[] measurement, double[] estimate) {
        double[] innovation = new double[measurement.length];
        for (int i = 0; i < measurement.length; i++) {
            innovation[i] = measurement[i] - estimate[i];
        }
        return innovation;
    }

    private void generateStateEstimate(double[] outputPrediction, SimpleMatrix gainMatrix) throws KalmanFilterException{
        systemModel.updateStateEstimate(outputPrediction, gainMatrix);
    }

    private void generateCovarianceEstimate(SimpleMatrix gainMatrix) throws KalmanFilterException{
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

    void setSystemModel(StateSpace systemModel) {
        this.systemModel = systemModel;
    }

    private EstimateTimeSeries getStateEstimateTimeSeries(){
        return this.systemModel.getStateHistory();
    }

}

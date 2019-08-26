package kalmanfilter;

import org.ejml.simple.SimpleMatrix;
import org.springframework.stereotype.Component;

@Component
class LinearKalmanFilter {

    private StateSpace systemModel;

    EstimateTimeSeries filter(MeasurementSet measurements) throws KalmanFilterException {

        checkAllFieldsInitialized();
        double outputPrediction;
        SimpleMatrix gainMatrix;

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

    private SimpleMatrix generateKalmanGainMatrix(){
        return systemModel.generateKalmanGainMatrix();
    }

    private void generateStateEstimate(double outputPrediction, SimpleMatrix gainMatrix) throws KalmanFilterException{
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
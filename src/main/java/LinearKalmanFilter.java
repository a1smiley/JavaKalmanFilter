import org.springframework.stereotype.Component;

@Component
public class LinearKalmanFilter {

    StateSpace systemModel;
    MeasurementSet measurements;


    public EstimateTimeSeries filter(MeasurementSet measurements) throws KalmanFilterException {
        this.measurements = measurements;
        checkAllFieldsInitialized();
        for (int i = 0; i < measurements.duration; i++){
            generateStatePrediction(measurements.getInputMeasurements()[i]);
            generateCovariancePrediction();
            generateOutputPrediction();
            generateKalmanGainMatrix();
            generateStateEstimate();
            generateCovarianceEstimate();
        }
    }

    private void generateStatePrediction(double input) throws KalmanFilterException {
        systemModel.transitionStateEstimate(input);
    }

    private void checkAllFieldsInitialized() throws KalmanFilterException {
        if (systemModel == null) {
            throw new KalmanFilterException("System model must be defined prior to evaluating filter");
        }
    }

    public void setSystemModel(StateSpace systemModel) {
        this.systemModel = systemModel;
    }

}

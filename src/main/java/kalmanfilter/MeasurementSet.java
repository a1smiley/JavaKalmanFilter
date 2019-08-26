package kalmanfilter;

public class MeasurementSet {

    protected int duration;
    private double[] validatedInputMeasurements;
    private double[] validatedOutputMeasurements;

    public MeasurementSet() {}

    public void setInputMeasurements(double[] unvalidatedInputMeasurements){
        this.validatedInputMeasurements = unvalidatedInputMeasurements;
    }

    public void setOutputMeasurements(double[] unvalidatedOutputMeasurements) {
        this.validatedOutputMeasurements =  unvalidatedOutputMeasurements;
    }

    public double getInputMeasurement(int timeStep) {
        return validatedInputMeasurements[timeStep];
    }

    public double getOutputMeasurement(int timeStep) {
        return validatedOutputMeasurements[timeStep];
    }
}

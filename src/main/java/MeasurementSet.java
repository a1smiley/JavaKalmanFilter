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

    public double[] getInputMeasurements() {
        return validatedInputMeasurements;
    }

    public double[] getOutputMeasurements() {
        return validatedOutputMeasurements;
    }
}

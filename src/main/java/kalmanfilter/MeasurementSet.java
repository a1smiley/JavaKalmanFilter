package kalmanfilter;

class MeasurementSet {

    int duration;
    private double[] validatedInputMeasurements;
    private double[] validatedOutputMeasurements;

    MeasurementSet() {}

    void setInputMeasurements(double[] unvalidatedInputMeasurements){
        this.validatedInputMeasurements = unvalidatedInputMeasurements;
    }

    void setOutputMeasurements(double[] unvalidatedOutputMeasurements) {
        this.validatedOutputMeasurements =  unvalidatedOutputMeasurements;
    }

    double getInputMeasurement(int timeStep) {
        return validatedInputMeasurements[timeStep];
    }

    double getOutputMeasurement(int timeStep) {
        return validatedOutputMeasurements[timeStep];
    }
}

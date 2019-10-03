package kalmanfilter;

import lombok.Getter;

import java.util.Arrays;

public class MeasurementSet {
    @Getter
    private int duration;
    private double[][] inputMeasurements;
    private double[][] outputMeasurements;

    MeasurementSet(double[][] inputMeasurements, double[][] outputMeasurements) throws KalmanFilterException {
        log.debug(() -> "Input measurement array is: " + Arrays.toString(inputMeasurements));
        log.debug(() -> "Output measurement array is: " + Arrays.toString(outputMeasurements));

        if (inputMeasurements.length == outputMeasurements.length) {
            this.inputMeasurements = inputMeasurements;
            this.outputMeasurements = outputMeasurements;
            this.duration = inputMeasurements.length;
        } else {
            throw new KalmanFilterException("Unequal number of input samples and output measurements");
        }
    }

    void setInputMeasurements(double[][] inputMeasurements){
        this.inputMeasurements = inputMeasurements;
    }

    void setOutputMeasurements(double[][] outputMeasurements) {
        this.outputMeasurements =  outputMeasurements;
    }

    double[] getInputMeasurement(int timeStep) {
        return inputMeasurements[timeStep];
    }

    double[] getOutputMeasurement(int timeStep) {
        return outputMeasurements[timeStep];
    }
}

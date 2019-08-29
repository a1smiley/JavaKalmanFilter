package kalmanfilter;

import lombok.Getter;

import java.util.Arrays;

public class MeasurementSet {

    @Getter
    private int duration;
    private double[] inputMeasurements;
    private double[] outputMeasurements;

    MeasurementSet(double[] inputMeasurements, double[] outputMeasurements) {
        System.out.println("Input measurement array is: " + Arrays.toString(inputMeasurements));
        System.out.println("Output measurement array is: " + Arrays.toString(outputMeasurements));
        if (inputMeasurements.length == outputMeasurements.length) {
            this.inputMeasurements = inputMeasurements;
            this.outputMeasurements = outputMeasurements;
            this.duration = inputMeasurements.length;
        }
    }

    void setInputMeasurements(double[] inputMeasurements){
        this.inputMeasurements = inputMeasurements;
    }

    void setOutputMeasurements(double[] outputMeasurements) {
        this.outputMeasurements =  outputMeasurements;
    }

    double getInputMeasurement(int timeStep) {
        return inputMeasurements[timeStep];
    }

    double getOutputMeasurement(int timeStep) {
        return outputMeasurements[timeStep];
    }
}

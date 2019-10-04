package kalmanfilter;

import lombok.Getter;
import lombok.Setter;
import lombok.extern.log4j.Log4j2;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.Arrays;

@Log4j2
@Getter
public class MeasurementSet {
    private int duration;
    @Setter
    private double[][] inputMeasurements;
    @Setter
    private double[][] outputMeasurements;

    MeasurementSet(double[][] inputMeasurements, double[][] outputMeasurements) throws KalmanFilterException {
        log.debug(() -> "Input measurement array is: " + Arrays.deepToString(inputMeasurements));
        log.debug(() -> "Output measurement array is: " + Arrays.deepToString(outputMeasurements));

        if (inputMeasurements.length == outputMeasurements.length) {
            this.inputMeasurements = inputMeasurements;
            this.outputMeasurements = outputMeasurements;
            this.duration = inputMeasurements.length;
        } else {
            throw new KalmanFilterException("Unequal number of input samples and output measurements");
        }
    }

    double[] getInputMeasurement(int timeStep) {
        return inputMeasurements[timeStep];
    }

    double[] getOutputMeasurement(int timeStep) {
        return outputMeasurements[timeStep];
    }
}

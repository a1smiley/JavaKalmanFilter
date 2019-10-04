package kalmanfilter;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class MeasurementSetTest {

    MeasurementSet testMeasurements;

    @Before
    public void createMeasurements() throws KalmanFilterException {
        double[][] input = {{1}, {2}};
        double[][] output = {{2}, {4}};
        testMeasurements = new MeasurementSet(input, output);
    }

    @Test
    public void creates_measurementSet_object() {
        assertNotNull(testMeasurements);
    }

    @Test
    public void should_get_measurements_for_single_timeIndex() {
        assertEquals(1, testMeasurements.getInputMeasurement(0)[0], 0.0);
        assertEquals(2, testMeasurements.getInputMeasurement(1)[0], 0.0);
        assertEquals(2, testMeasurements.getOutputMeasurement(0)[0], 0.0);
        assertEquals(4, testMeasurements.getOutputMeasurement(1)[0], 0.0);
    }

    @Test(expected = KalmanFilterException.class)
    public void should_throw_exception_when_input_and_output_length_different() throws KalmanFilterException {
        double[][] input = {{1}};
        double[][] output = {{2}, {4}};
        MeasurementSet measurementSet = new MeasurementSet(input, output);
    }
}

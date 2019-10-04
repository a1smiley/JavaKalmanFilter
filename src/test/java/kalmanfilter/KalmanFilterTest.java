package kalmanfilter;

import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

public class KalmanFilterTest {

    private EstimateTimeSeries buildTest1DKalmanFilter() throws KalmanFilterException {
        double[][] Amat = {{0.5}};
        double[][] Bmat = {{1}};
        double[][] Cmat = {{1}};
        double[][] Dmat = {{0.1}};
        double[][] processNoise = {{0.01}};
        double[][] measurementNoise = {{0.01}};
        double[] initialStateMean = {2.0};
        double[][] initialStateCovar = {{1.0}};
        double[][] testInput = {{1.0}, {1.0}};
        double[][] testOutput = {{2.0}, {2.0}};

        State testState = new State(initialStateMean, initialStateCovar);
        StateSpace testStateSpace = new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, measurementNoise);
        KalmanFilter test1DKalmanFilter = new KalmanFilter();
        test1DKalmanFilter.setSystemModel(testStateSpace, testState);
        MeasurementSet testMeasurements = new MeasurementSet(testInput, testOutput);

        return test1DKalmanFilter.filter(testMeasurements);
    }

    private EstimateTimeSeries buildTest2DKalmanFilter() throws KalmanFilterException {
        double[][] Amat2D = {{1, 1},{0, 0.5}};
        double[][] Bmat2D = {{0}, {1}};
        double[][] Cmat2D = {{1, 1}};
        double[][] Dmat2D = {{1}};
        double[][] processNoise2D = {{0.1, 0.0}, {0.0, 0.1}};
        double[][] measurementNoise2D = {{0.1}};
        double[] initialStateMean2D = {1.0, 1.0};
        double[][] initialStateCovar2D = {{1.0, 0.0}, {0.0, 1.0}};
        double[][] testInput = {{1.0}, {1.0}};
        double[][] testOutput = {{4.0}, {4.0}};

        State testState = new State(initialStateMean2D, initialStateCovar2D);
        StateSpace testStateSpace = new StateSpace(Amat2D, Bmat2D, Cmat2D, Dmat2D, processNoise2D,
                measurementNoise2D);
        KalmanFilter test2DKalmanFilter = new KalmanFilter();
        test2DKalmanFilter.setSystemModel(testStateSpace, testState);
        MeasurementSet testMeasurements = new MeasurementSet(testInput, testOutput);

        return test2DKalmanFilter.filter(testMeasurements);
    }

    private EstimateTimeSeries buildTestMIMOKalmanFilter() throws KalmanFilterException {
        double[][] Amat2D = {{1, 1},{0, 0.5}};
        double[][] Bmat2D = {{0, 1}, {1, 0}};
        double[][] Cmat2D = {{1, 1},{0.5, 0}};
        double[][] Dmat2D = {{0.5, 0.5},{0.5,0.5}};
        double[][] processNoise2D = {{0.1, 0.0}, {0.0, 0.1}};
        double[][] measurementNoise2D = {{0.1, 0.0}, {0.0, 0.1}};
        double[] initialStateMean2D = {1.0, 1.0};
        double[][] initialStateCovar2D = {{1.0, 0.0}, {0.0, 1.0}};
        double[][] testInput = {{1.0, 1.0}, {1.0, 1.0}};
        double[][] testOutput = {{4.0, 3.0}, {4.0, 3.0}};

        State testState = new State(initialStateMean2D, initialStateCovar2D);
        StateSpace testStateSpace = new StateSpace(Amat2D, Bmat2D, Cmat2D, Dmat2D, processNoise2D,
                measurementNoise2D);
        KalmanFilter testMIMOKalmanFilter = new KalmanFilter();
        testMIMOKalmanFilter.setSystemModel(testStateSpace, testState);
        MeasurementSet testMeasurements = new MeasurementSet(testInput, testOutput);

        return testMIMOKalmanFilter.filter(testMeasurements);
    }

    @Test
    public void predicts_scalar_state() throws KalmanFilterException {
        EstimateTimeSeries results1D = buildTest1DKalmanFilter();
        DoubleArray statePredictionResults = results1D.getStatePrediction(1);
        assertEquals(2.0, statePredictionResults.getArray()[0],0.0);
    }

    @Test
    public void predicts_scalar_state_error_covariance() throws KalmanFilterException {
        EstimateTimeSeries results1D = buildTest1DKalmanFilter();
        Double2DArray covarPredictionResults = results1D.getCovariancePrediction(1);
        assertEquals(0.26, covarPredictionResults.getArray()[0][0],0.0);
    }

    @Test
    public void predicts_scalar_output() throws KalmanFilterException {
        EstimateTimeSeries results1D = buildTest1DKalmanFilter();
        double[] expectedOutputPrediction = {2.1};
        assertArrayEquals(expectedOutputPrediction, results1D.getOutputPrediction(1).getArray(), 0.0);
    }

    @Test
    public void calculates_scalar_gain() throws KalmanFilterException {
        EstimateTimeSeries results1D = buildTest1DKalmanFilter();
        double[] expectedGain = {0.9630};
        assertArrayEquals(expectedGain, results1D.getGain(1).getArray()[0], 0.0001);
    }

    @Test
    public void calculates_scalar_innovation() throws KalmanFilterException {
        EstimateTimeSeries results1D = buildTest1DKalmanFilter();
        double[] expectedInnovation = {-0.1};
        assertArrayEquals(expectedInnovation, results1D.getInnovation(1).getArray(), 0.0001);
    }

    @Test
    public void estimates_scalar_state() throws KalmanFilterException {
        EstimateTimeSeries results1D = buildTest1DKalmanFilter();
        DoubleArray stateEstimateResults = results1D.getStateEstimate(1);
        assertEquals(1.9037, stateEstimateResults.getArray()[0],0.0001);
    }

    @Test
    public void estimates_scalar_state_error_covariance() throws KalmanFilterException {
        EstimateTimeSeries results1D = buildTest1DKalmanFilter();
        Double2DArray covarEstimateResults = results1D.getCovarianceEstimate(1);
        assertEquals(0.0096, covarEstimateResults.getArray()[0][0],0.0001);
    }

    @Test
    public void predicts_2D_state() throws KalmanFilterException {
        EstimateTimeSeries results = buildTest2DKalmanFilter();
        DoubleArray statePredictionResults = results.getStatePrediction(1);
        assertEquals(2.0, statePredictionResults.getArray()[0],0.0);
        assertEquals(1.5, statePredictionResults.getArray()[1],0.0);
    }

    @Test
    public void predicts_2D_state_error_covariance() throws KalmanFilterException {
        EstimateTimeSeries results = buildTest2DKalmanFilter();
        double[][] expectedCovPrediction = {{2.1, 0.5},{0.5, 0.35}};
        assertArrayEquals(expectedCovPrediction, results.getCovariancePrediction(1).getArray());
    }

    @Test
    public void predicts_2D_output() throws KalmanFilterException {
        EstimateTimeSeries results = buildTest2DKalmanFilter();
        DoubleArray outputPredictionResults = results.getOutputPrediction(1);
        assertEquals(4.5, outputPredictionResults.getArray()[0],0.0);
    }

    @Test
    public void calculates_2D_innovation() throws KalmanFilterException {
        EstimateTimeSeries results = buildTest2DKalmanFilter();
        DoubleArray innovationResults = results.getInnovation(1);
        assertEquals(-0.5, innovationResults.getArray()[0],0.0);
    }

    @Test
    public void finds_2D_gain() throws KalmanFilterException {
        EstimateTimeSeries results = buildTest2DKalmanFilter();
        double[] expectedGain = {0.7324, 0.2394};
        assertArrayEquals(expectedGain, results.getGain(1).getArray()[0], 0.0001);
    }

    @Test
    public void estimates_2D_state() throws KalmanFilterException {
        EstimateTimeSeries results = buildTest2DKalmanFilter();
        DoubleArray stateEstimateResults = results.getStateEstimate(1);
        assertEquals(1.6338, stateEstimateResults.getArray()[0],0.0001);
        assertEquals(1.3803, stateEstimateResults.getArray()[1],0.0001);
    }

    @Test
    public void estimates_2D_state_error_covariance() throws KalmanFilterException {
        EstimateTimeSeries results = buildTest2DKalmanFilter();
        double[][] expectedOutputArray = {{0.1958, -0.1225},{-0.1225, 0.1465}};
        Double2DArray testUpdateEstimate = results.getCovarianceEstimate(1);
        assertArrayEquals(expectedOutputArray[0], testUpdateEstimate.getArray()[0], 0.0001);
        assertArrayEquals(expectedOutputArray[1], testUpdateEstimate.getArray()[1], 0.0001);
    }

    @Test
    public void predicts_MIMO_state() throws KalmanFilterException {
        EstimateTimeSeries results = buildTestMIMOKalmanFilter();
        DoubleArray statePredictionResults = results.getStatePrediction(1);
        assertEquals(3.0, statePredictionResults.getArray()[0],0.0);
        assertEquals(1.5, statePredictionResults.getArray()[1],0.0);
    }

    @Test
    public void predicts_MIMO_state_error_covariance() throws KalmanFilterException {
        EstimateTimeSeries results = buildTestMIMOKalmanFilter();
        double[][] expectedOutputArray = {{2.1, 0.5},{0.5, 0.35}};
        Double2DArray covarPredictionResults = results.getCovariancePrediction(1);
        assertArrayEquals(expectedOutputArray[0], covarPredictionResults.getArray()[0], 0.0001);
        assertArrayEquals(expectedOutputArray[1], covarPredictionResults.getArray()[1], 0.0001);
    }

    @Test
    public void predicts_MIMO_output() throws KalmanFilterException {
        EstimateTimeSeries results = buildTestMIMOKalmanFilter();
        double[] expectedResults = {5.5, 2.5};
        assertArrayEquals(expectedResults, results.getOutputPrediction(1).getArray(), 0.0);
    }

    @Test
    public void calculates_MIMO_innovation() throws KalmanFilterException {
        EstimateTimeSeries results = buildTestMIMOKalmanFilter();
        double[] expectedResults = {-1.5, 0.5};
        DoubleArray innovationResults = results.getInnovation(1);
        assertArrayEquals(expectedResults, innovationResults.getArray(),0.0);
    }

    @Test
    public void finds_MIMO_gain() throws KalmanFilterException {
        EstimateTimeSeries results = buildTestMIMOKalmanFilter();
        double[][] expectedGain = {{0.4917, 0.6572}, {0.3901, -0.4113}};
        Double2DArray gainResults = results.getGain(1);
        assertArrayEquals(expectedGain[0], gainResults.getArray()[0], 0.0001);
        assertArrayEquals(expectedGain[1], gainResults.getArray()[1], 0.0001);
    }

    @Test
    public void estimates_MIMO_state() throws KalmanFilterException {
        EstimateTimeSeries results = buildTestMIMOKalmanFilter();
        DoubleArray stateEstimateResults = results.getStateEstimate(1);
        assertEquals(2.5910, stateEstimateResults.getArray()[0],0.0001);
        assertEquals(0.7092, stateEstimateResults.getArray()[1],0.0001);
    }

    @Test
    public void estimates_MIMO_state_error_covariance() throws KalmanFilterException {
        EstimateTimeSeries results = buildTestMIMOKalmanFilter();
        double[][] expectedOutputArray = {{0.1314, -0.0823},{-0.0823, 0.1213}};
        Double2DArray testUpdateEstimate = results.getCovarianceEstimate(1);
        assertArrayEquals(expectedOutputArray[0], testUpdateEstimate.getArray()[0], 0.0001);
        assertArrayEquals(expectedOutputArray[1], testUpdateEstimate.getArray()[1], 0.0001);
    }

}

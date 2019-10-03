package kalmanfilter;

import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import static org.junit.Assert.*;

public class StateSpaceTest {

    double[][] Amat = {{0.5}};
    double[][] Bmat = {{1}};
    double[][] Cmat = {{1}};
    double[][] Dmat = {{0.1}};
    double[][] processNoise = {{0.01}};
    double[][] measurementNoise = {{0.01}};
    double[] initialStateMean = {2.0};
    double[][] initialStateCovar = {{0.0}};

    @Test
    public void creates_state_space_object_with_1D_state() throws KalmanFilterException {
        State testState = new State(initialStateMean, initialStateCovar );
        StateSpace testStateSpace = new StateSpace(Amat, Bmat, Cmat, Dmat,
                processNoise, measurementNoise, testState);

        assertNotNull(testState);
        assertNotNull(testStateSpace);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_for_non_square_Amat() throws KalmanFilterException {
        double[][] badAmat = {{0.5}, {1}};

        State testState = new State(initialStateMean, initialStateCovar );
        new StateSpace(badAmat, Bmat, Cmat, Dmat, processNoise, measurementNoise, testState);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_Bmat_dimension_mismatch() throws KalmanFilterException {
        double[][] badBmat = {{1}, {1}};

        State testState = new State(initialStateMean, initialStateCovar );
        new StateSpace(Amat, badBmat, Cmat, Dmat, processNoise, measurementNoise, testState);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_Cmat_dimension_mismatch() throws KalmanFilterException {
        double[][] badCmat = {{0.1, 1}};

        State testState = new State(initialStateMean, initialStateCovar );
        new StateSpace(Amat, Bmat, badCmat, Dmat, processNoise, measurementNoise, testState);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_Dmat_colDimension_mismatch() throws KalmanFilterException {
        double[][] bigAmat = {{0.5, 0.6}, {0.3, 0.4}};
        double[][] bigBmat = {{1, 1}, {1, 1}};
        double[][] bigCmat = {{0.1, 1}};
        double[][] badDmat = {{0.01}};

        State testState = new State(initialStateMean, initialStateCovar );
        new StateSpace(bigAmat, bigBmat, bigCmat, badDmat, processNoise, measurementNoise, testState);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_Dmat_rowDimension_mismatch() throws KalmanFilterException {
        double[][] bigAmat = {{0.5, 0.6}, {0.3, 0.4}};
        double[][] bigBmat = {{1}, {1}};
        double[][] bigCmat = {{1, 1}, {1, 1}};
        double[][] badDmat = {{0.01}};

        State testState = new State(initialStateMean, initialStateCovar );
        new StateSpace(bigAmat, bigBmat, bigCmat, badDmat, processNoise, measurementNoise, testState);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_processNoise_colDimension_mismatch() throws KalmanFilterException {
        double[][] badProcessNoise = {{0.01, 1}};

        State testState = new State(initialStateMean, initialStateCovar );
        new StateSpace(Amat, Bmat, Cmat, Dmat, badProcessNoise, measurementNoise, testState);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_processNoise_rowDimension_mismatch() throws KalmanFilterException {
        double[][] badProcessNoise = {{0.01}, {1}};

        State testState = new State(initialStateMean, initialStateCovar );
        new StateSpace(Amat, Bmat, Cmat, Dmat, badProcessNoise, measurementNoise, testState);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_measurementNoise_colDimension_mismatch() throws KalmanFilterException {
        double[][] badMeasurementNoise = {{0.01, 1}};

        State testState = new State(initialStateMean, initialStateCovar );
        new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, badMeasurementNoise, testState);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_measurementNoise_rowDimension_mismatch() throws KalmanFilterException {
        double[][] badMeasurementNoise = {{0.01}, {1}};

        State testState = new State(initialStateMean, initialStateCovar );
        new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, badMeasurementNoise, testState);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_measurementNoise_outputDimension_mismatch() throws KalmanFilterException {
        double[][] Bmat = {{1}, {1}};
        double[][] Dmat = {{1}, {1}};
        double[][] badMeasurementNoise = {{0.01}};

        State testState = new State(initialStateMean, initialStateCovar );
        new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, badMeasurementNoise, testState);
    }

    @Test
    public void calculates_1D_state_prediction() throws KalmanFilterException {
        State testState = new State(initialStateMean, initialStateCovar);
        StateSpace testStateSpace = new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, measurementNoise, testState);
        double[] testInput = {1.0};
        testStateSpace.updateStatePrediction(testInput);

        EstimateTimeSeries results = testStateSpace.getStateHistory();
        DoubleArray statePredictionResults = results.getStatePrediction(1);
        assertEquals(2.0, statePredictionResults.getArray()[0],0.0);
    }

    @Test
    public void calculates_2D_state_prediction() throws KalmanFilterException {
        double[][] Amat2D = {{1, 1},{0, 0.5}};
        double[][] Bmat2D = {{0}, {1}};
        double[][] Cmat2D = {{1, 1}};
        double[][] Dmat2D = {{1}};
        double[][] processNoise2D = {{0.01, 0.0}, {0.0, 0.01}};
        double[][] measurementNoise2D = {{0.01}};
        double[] initialStateMean2D = {1.0, 1.0};
        double[][] initialStateCovar2D = {{0.0, 0.0}, {0.0, 0.0}};
        State testState2D = new State(initialStateMean2D, initialStateCovar2D);
        StateSpace testStateSpace = new StateSpace(Amat2D, Bmat2D, Cmat2D, Dmat2D, processNoise2D,
                measurementNoise2D, testState2D);
        double[] testInput = {1.0};
        testStateSpace.updateStatePrediction(testInput);

        EstimateTimeSeries results = testStateSpace.getStateHistory();
        DoubleArray statePredictionResults = results.getStatePrediction(1);
        assertEquals(2.0, statePredictionResults.getArray()[0],0.0);
        assertEquals(1.5, statePredictionResults.getArray()[1],0.0);
    }

    @Test
    public void calculates_1D_covariance_prediction() throws KalmanFilterException{
        State testState = new State(initialStateMean, initialStateCovar);
        StateSpace testStateSpace = new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, measurementNoise, testState);
        testStateSpace.updateCovariancePrediction();

        EstimateTimeSeries results = testStateSpace.getStateHistory();
        Double2DArray covarPredictionResults = results.getCovariancePrediction(1);
        assertEquals(0.01, covarPredictionResults.getArray()[0][0],0.0);
    }

    @Test
    public void calculates_2D_covariance_prediction() throws KalmanFilterException {
        double[][] Amat2D = {{1, 1},{0, 0.5}};
        double[][] Bmat2D = {{0}, {1}};
        double[][] Cmat2D = {{1, 1}};
        double[][] Dmat2D = {{1}};
        double[][] processNoise2D = {{0.01, 0.0}, {0.0, 0.01}};
        double[][] measurementNoise2D = {{0.01}};
        double[] initialStateMean2D = {1.0, 1.0};
        double[][] initialStateCovar2D = {{1.0, 0.0}, {0.0, 1.0}};
        State testState2D = new State(initialStateMean2D, initialStateCovar2D);
        StateSpace testStateSpace = new StateSpace(Amat2D, Bmat2D, Cmat2D, Dmat2D, processNoise2D,
                measurementNoise2D, testState2D);
        testStateSpace.updateCovariancePrediction();

        EstimateTimeSeries results = testStateSpace.getStateHistory();
        Double2DArray testInitialCov = results.getCovariancePrediction(0);
        Double2DArray testUpdatePredication = results.getCovariancePrediction(1);

        assertArrayEquals(initialStateCovar2D, testInitialCov.getArray());
        double[][] expectedOutputArray = {{2.01, 0.5},{0.5, 0.26}};
        assertArrayEquals(expectedOutputArray, testUpdatePredication.getArray());
    }

    @Test
    public void generate_1D_output_prediction() throws KalmanFilterException {
        State testState = new State(initialStateMean, initialStateCovar);
        StateSpace testStateSpace = new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, measurementNoise, testState);
        double[] testInput = {1.0};
        double[] result = testStateSpace.generateOutputPrediction(testInput);

        double[] expectedResult = {2.1};
        assertArrayEquals(expectedResult, result, 0.0);
    }

    @Test
    public void generate_MIMO_output_prediction() throws KalmanFilterException {
        double[][] Amat2D = {{1, 1},{0, 0.5}};
        double[][] Bmat2D = {{0, 1}, {1, 0}};
        double[][] Cmat2D = {{1, 1},{0.5, 0}};
        double[][] Dmat2D = {{0.5, 0.5},{0.5,0.5}};
        double[][] processNoise2D = {{0.01, 0.0}, {0.0, 0.01}};
        double[][] measurementNoise2D = {{0.01, 0.0}, {0.0, 0.01}};
        double[] initialStateMean2D = {1.0, 1.0};
        double[][] initialStateCovar2D = {{1.0, 0.0}, {0.0, 1.0}};
        double[] testInput = {2.0, 2.0};
        State testState2D = new State(initialStateMean2D, initialStateCovar2D);
        StateSpace testStateSpace = new StateSpace(Amat2D, Bmat2D, Cmat2D, Dmat2D, processNoise2D,
                measurementNoise2D, testState2D);
        double[] results = testStateSpace.generateOutputPrediction(testInput);

        double[] expectedResults = {4.0, 2.5};
        assertArrayEquals(expectedResults, results, 0.0);
    }

    @Test
    public void generate_1D_Kalman_gain_matrix() throws KalmanFilterException {
        double[][] initialStateCovar = {{1.0}};
        State testState = new State(initialStateMean, initialStateCovar);
        StateSpace testStateSpace = new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, measurementNoise, testState);
        SimpleMatrix result = testStateSpace.generateKalmanGainMatrix();

        double[] expectedResult = {0.9901};
        assertArrayEquals(expectedResult, result.getMatrix().getData(), 0.0001);
    }

    @Test
    public void generate_2D_Kalman_gain_matrix() throws KalmanFilterException {
        double[][] Amat2D = {{1, 1},{0, 0.5}};
        double[][] Bmat2D = {{0}, {1}};
        double[][] Cmat2D = {{1, 1}};
        double[][] Dmat2D = {{1}};
        double[][] processNoise2D = {{0.1, 0.0}, {0.0, 0.1}};
        double[][] measurementNoise2D = {{0.1}};
        double[] initialStateMean2D = {1.0, 1.0};
        double[][] initialStateCovar2D = {{1.0, 0.0}, {0.0, 1.0}};
        State testState = new State(initialStateMean2D, initialStateCovar2D);
        StateSpace testStateSpace = new StateSpace(Amat2D, Bmat2D, Cmat2D, Dmat2D, processNoise2D, measurementNoise2D, testState);
        SimpleMatrix result = testStateSpace.generateKalmanGainMatrix();

        double[] expectedResult = {0.4762, 0.4762};
        assertArrayEquals(expectedResult, result.getMatrix().getData(), 0.0001);
    }

    @Test
    public void calculates_1D_state_estimate() throws KalmanFilterException {
        State testState = new State(initialStateMean, initialStateCovar);
        StateSpace testStateSpace = new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, measurementNoise, testState);
        double[] innovation = {2};
        double[][] gainArray = {{0.5}};
        SimpleMatrix gainMatrix = new SimpleMatrix(gainArray);
        testStateSpace.updateStateEstimate(innovation, gainMatrix);

        EstimateTimeSeries results = testStateSpace.getStateHistory();
        DoubleArray stateEstimateResults = results.getStateEstimate(1);
        assertEquals(3.0, stateEstimateResults.getArray()[0],0.0);
    }

    @Test
    public void calculates_2D_state_estimate() throws KalmanFilterException {
        double[][] Amat2D = {{1, 1},{0, 0.5}};
        double[][] Bmat2D = {{0}, {1}};
        double[][] Cmat2D = {{1, 1}};
        double[][] Dmat2D = {{1}};
        double[][] processNoise2D = {{0.01, 0.0}, {0.0, 0.01}};
        double[][] measurementNoise2D = {{0.01}};
        double[] initialStateMean2D = {1.0, 1.0};
        double[][] initialStateCovar2D = {{0.0, 0.0}, {0.0, 0.0}};
        State testState2D = new State(initialStateMean2D, initialStateCovar2D);
        StateSpace testStateSpace = new StateSpace(Amat2D, Bmat2D, Cmat2D, Dmat2D, processNoise2D,
                measurementNoise2D, testState2D);

        double[] innovation = {2};
        double[][] gainArray = {{0.5},{0.5}};
        SimpleMatrix gainMatrix = new SimpleMatrix(gainArray);
        testStateSpace.updateStateEstimate(innovation, gainMatrix);

        EstimateTimeSeries results = testStateSpace.getStateHistory();
        DoubleArray stateEstimateResults = results.getStateEstimate(1);
        assertEquals(2.0, stateEstimateResults.getArray()[0],0.0);
        assertEquals(2.0, stateEstimateResults.getArray()[1],0.0);
    }

    @Test
    public void calculates_1D_covariance_estimate() throws KalmanFilterException{
        double[][] initialStateCovar = {{1.0}};
        State testState = new State(initialStateMean, initialStateCovar);
        StateSpace testStateSpace = new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, measurementNoise, testState);

        double[][] gainArray = {{0.5}};
        SimpleMatrix gainMatrix = new SimpleMatrix(gainArray);
        testStateSpace.updateCovarianceEstimate(gainMatrix);

        EstimateTimeSeries results = testStateSpace.getStateHistory();
        Double2DArray covarEstimateResults = results.getCovarianceEstimate(1);
        assertEquals(0.5, covarEstimateResults.getArray()[0][0],0.0);
    }

    @Test
    public void calculates_2D_covariance_estimate() throws KalmanFilterException {
        double[][] Amat2D = {{1, 1},{0, 0.5}};
        double[][] Bmat2D = {{0}, {1}};
        double[][] Cmat2D = {{1, 1}};
        double[][] Dmat2D = {{1}};
        double[][] processNoise2D = {{0.01, 0.0}, {0.0, 0.01}};
        double[][] measurementNoise2D = {{0.01}};
        double[] initialStateMean2D = {1.0, 1.0};
        double[][] initialStateCovar2D = {{1.0, 0.0}, {0.0, 1.0}};
        State testState2D = new State(initialStateMean2D, initialStateCovar2D);
        StateSpace testStateSpace = new StateSpace(Amat2D, Bmat2D, Cmat2D, Dmat2D, processNoise2D,
                measurementNoise2D, testState2D);

        double[][] gainArray = {{0.5},{0.5}};
        SimpleMatrix gainMatrix = new SimpleMatrix(gainArray);
        testStateSpace.updateCovarianceEstimate(gainMatrix);

        double[][] expectedOutputArray = {{0.5, -0.5},{-0.5, 0.5}};
        EstimateTimeSeries results = testStateSpace.getStateHistory();
        Double2DArray testUpdateEstimate = results.getCovarianceEstimate(1);
        assertArrayEquals(expectedOutputArray, testUpdateEstimate.getArray());
    }
}
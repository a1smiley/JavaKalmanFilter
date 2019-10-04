package kalmanfilter;

import org.ejml.simple.SimpleMatrix;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

public class StateTest {

    private State testState;

    @Before
    public void create_test_object() {
        double[] initialState = {1, 2};
        double[][] initialCovariance = {{1, 0},{0, 1}};
        testState = new State(initialState, initialCovariance);
    }

    @Test
    public void should_create_state_object() {
        assertNotNull(testState);
    }

    @Test
    public void should_store_initial_state_as_prediction() {
        double[] expectedStatePrediction = {1, 2};
        assertArrayEquals(expectedStatePrediction, testState.getStatePrediction(), 0.0);
    }

    @Test
    public void should_store_initial_state_as_estimate() {
        double[] expectedStateEstimate = {1, 2};
        assertArrayEquals(expectedStateEstimate, testState.getStateEstimate(), 0.0);
    }

    @Test
    public void should_store_initial_covariance_as_prediction() {
        double[][] expectedCovPrediction = {{1, 0}, {0, 1}};
        assertArrayEquals(expectedCovPrediction, testState.getCovariancePrediction());
    }

    @Test
    public void should_store_initial_covariance_as_estimate() {
        double[][] expectedCovEstimate = {{1, 0}, {0, 1}};
        assertArrayEquals(expectedCovEstimate, testState.getCovarianceEstimate());
    }

    @Test
    public void should_accept_simpleMatrix_for_covariance_prediction() {
        double[][] newCovPrediction = {{2, 0}, {0, 2}};
        SimpleMatrix newCovPredictionMat = new SimpleMatrix(newCovPrediction);
        testState.setCovariancePrediction(newCovPredictionMat);
        assertArrayEquals(newCovPrediction, testState.getCovariancePrediction());
    }

    @Test
    public void should_accept_simpleMatrix_for_covariance_estimate() {
        double[][] newCovEstimate = {{2, 0}, {0, 2}};
        SimpleMatrix newCovEstimateMat = new SimpleMatrix(newCovEstimate);
        testState.setCovarianceEstimate(newCovEstimateMat);
        assertArrayEquals(newCovEstimate, testState.getCovarianceEstimate());
    }

    @Test
    public void should_accept_double_array_for_covariance_prediction() {
        double[][] newCovPrediction = {{2, 0}, {0, 2}};
        testState.setCovariancePrediction(newCovPrediction);
        assertArrayEquals(newCovPrediction, testState.getCovariancePrediction());
    }

    @Test
    public void should_accept_double_array_for_covariance_estimate() {
        double[][] newCovEstimate = {{2, 0}, {0, 2}};
        testState.setCovarianceEstimate(newCovEstimate);
        assertArrayEquals(newCovEstimate, testState.getCovarianceEstimate());
    }
}

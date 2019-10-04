package kalmanfilter;

import org.junit.Test;

import static org.junit.Assert.*;

public class StateSpaceTest {

    private double[][] Amat = {{0.5}};
    private double[][] Bmat = {{1}};
    private double[][] Cmat = {{1}};
    private double[][] Dmat = {{0.1}};
    private double[][] processNoise = {{0.01}};
    private double[][] measurementNoise = {{0.01}};

    @Test
    public void creates_state_space_object_with_1D_state() throws KalmanFilterException {
        StateSpace testStateSpace = new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, measurementNoise);
        assertNotNull(testStateSpace);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_for_non_square_Amat() throws KalmanFilterException {
        double[][] badAmat = {{0.5}, {1}};
        new StateSpace(badAmat, Bmat, Cmat, Dmat, processNoise, measurementNoise);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_Bmat_dimension_mismatch() throws KalmanFilterException {
        double[][] badBmat = {{1}, {1}};
        new StateSpace(Amat, badBmat, Cmat, Dmat, processNoise, measurementNoise);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_Cmat_dimension_mismatch() throws KalmanFilterException {
        double[][] badCmat = {{0.1, 1}};
        new StateSpace(Amat, Bmat, badCmat, Dmat, processNoise, measurementNoise);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_Dmat_colDimension_mismatch() throws KalmanFilterException {
        double[][] bigAmat = {{0.5, 0.6}, {0.3, 0.4}};
        double[][] bigBmat = {{1, 1}, {1, 1}};
        double[][] bigCmat = {{0.1, 1}};
        double[][] badDmat = {{0.01}};
        new StateSpace(bigAmat, bigBmat, bigCmat, badDmat, processNoise, measurementNoise);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_Dmat_rowDimension_mismatch() throws KalmanFilterException {
        double[][] bigAmat = {{0.5, 0.6}, {0.3, 0.4}};
        double[][] bigBmat = {{1}, {1}};
        double[][] bigCmat = {{1, 1}, {1, 1}};
        double[][] badDmat = {{0.01}};
        new StateSpace(bigAmat, bigBmat, bigCmat, badDmat, processNoise, measurementNoise);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_processNoise_colDimension_mismatch() throws KalmanFilterException {
        double[][] badProcessNoise = {{0.01, 1}};
        new StateSpace(Amat, Bmat, Cmat, Dmat, badProcessNoise, measurementNoise);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_processNoise_rowDimension_mismatch() throws KalmanFilterException {
        double[][] badProcessNoise = {{0.01}, {1}};
        new StateSpace(Amat, Bmat, Cmat, Dmat, badProcessNoise, measurementNoise);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_measurementNoise_colDimension_mismatch() throws KalmanFilterException {
        double[][] badMeasurementNoise = {{0.01, 1}};
        new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, badMeasurementNoise);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_measurementNoise_rowDimension_mismatch() throws KalmanFilterException {
        double[][] badMeasurementNoise = {{0.01}, {1}};
        new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, badMeasurementNoise);
    }

    @Test(expected = KalmanFilterException.class)
    public void throws_kf_exception_if_measurementNoise_outputDimension_mismatch() throws KalmanFilterException {
        double[][] Bmat = {{1}, {1}};
        double[][] Dmat = {{1}, {1}};
        double[][] badMeasurementNoise = {{0.01}};
        new StateSpace(Amat, Bmat, Cmat, Dmat, processNoise, badMeasurementNoise);
    }

}
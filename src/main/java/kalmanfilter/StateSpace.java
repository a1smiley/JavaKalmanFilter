package kalmanfilter;

import lombok.Getter;
import org.ejml.simple.SimpleMatrix;

@Getter
class StateSpace {

    private SimpleMatrix Amat;
    private SimpleMatrix Bmat;
    private SimpleMatrix Cmat;
    private SimpleMatrix Dmat;
    private SimpleMatrix processNoise;
    private SimpleMatrix measurementNoise;
    private int numPoles;
    private int numInput;
    private int numOutput;

    StateSpace(double[][] stateTransitionA, double[][] stateTransitionB,
                      double[][] outputTransitionC, double[][] outputTransitionD, double[][] processNoise,
                      double[][] measurementNoise) throws KalmanFilterException {
        this.Amat = validateAmat(stateTransitionA);
        this.Bmat = validateBmat(stateTransitionB);
        this.Cmat = validateCmat(outputTransitionC);
        this.Dmat = validateDmat(outputTransitionD);
        this.processNoise = validateProcessNoise(processNoise);
        this.measurementNoise = validateMeasurementNoise(measurementNoise);
    }

    private SimpleMatrix validateAmat(double[][] unvalidatedAmat) throws KalmanFilterException {
        SimpleMatrix Amatrix = new SimpleMatrix(unvalidatedAmat);
        if (Amatrix.numCols() == Amatrix.numRows()) {
            numPoles = Amatrix.numCols();
            return Amatrix;
        } else {
            throw new KalmanFilterException("A matrix must be square.");
        }
    }

    private SimpleMatrix validateBmat(double[][] unvalidatedBmat) throws KalmanFilterException {
        SimpleMatrix Bmatrix = new SimpleMatrix(unvalidatedBmat);
        int nRowsB = Bmatrix.numRows();
        if (nRowsB == numPoles){
            return Bmatrix;
        } else {
            throw new KalmanFilterException("Dimensions of B matrix incompatible with dimensions of A matrix \n"
                + "Square dimension of A: " + numPoles +"\n"
                + "Rows of B: " + nRowsB + "\n");
        }
    }

    private SimpleMatrix validateCmat(double[][] unvalidatedCmat) throws KalmanFilterException {
        SimpleMatrix Cmatrix = new SimpleMatrix(unvalidatedCmat);
        int nColsC = Cmatrix.numCols();
        if (nColsC == numPoles){
            return Cmatrix;
        } else {
            throw new KalmanFilterException("Dimensions of C matrix incompatible with dimensions of state \n"
                    + "Dimension of state: " + numPoles +"\n"
                    + "Columns of C: " + nColsC + "\n");
        }
    }

    private SimpleMatrix validateDmat(double[][] unvalidatedDmat) throws KalmanFilterException {
        SimpleMatrix Dmatrix = new SimpleMatrix(unvalidatedDmat);
        validateDmatInputSize(Dmatrix);
        validateDmatOutputSize(Dmatrix);
        return Dmatrix;
    }

    private boolean validateDmatInputSize(SimpleMatrix Dmatrix) throws KalmanFilterException {
        int nColsD = Dmatrix.numCols();
        int nInputColsB = Bmat.numCols();
        if (nColsD == nInputColsB){
            numInput = nInputColsB;
            return true;
        } else {
            throw new KalmanFilterException("Dimensions of D matrix incompatible with input dimensions of B matrix \n"
                    + "Input dimension of B matrix: " + nInputColsB +"\n"
                    + "Input dimension of D matrix: " + nColsD + "\n");
        }
    }

    private boolean validateDmatOutputSize(SimpleMatrix Dmatrix) throws KalmanFilterException {
        int nRowsD = Dmatrix.numRows();
        int nOutputRowsC = Cmat.numRows();
        if (nRowsD == nOutputRowsC){
            numOutput = nOutputRowsC;
            return true;
        } else {
            throw new KalmanFilterException("Dimensions of D matrix incompatible with output dimensions of C matrix \n"
                    + "Output dimension of C matrix: " + nOutputRowsC +"\n"
                    + "Output dimension of D matrix: " + nRowsD + "\n");
        }
    }

    private SimpleMatrix validateProcessNoise(double[][] unvalidatedProcessNoise) throws KalmanFilterException{
        SimpleMatrix processNoise = new SimpleMatrix(unvalidatedProcessNoise);
        int nRows = processNoise.numRows();
        int nCols = processNoise.numCols();
        if (nRows == nCols && nRows == numPoles){
            return processNoise;
        } else {
            throw new KalmanFilterException("Dimensions of process noise matrix incompatible with dimensions of A matrix \n"
                    + "Square dimension of A matrix: " + numPoles +"\n"
                    + "Square dimension of process noise matrix: " + nCols + "\n");
        }
    }

    private SimpleMatrix validateMeasurementNoise(double[][] unvalidatedMeasurementNoise) throws KalmanFilterException{
        SimpleMatrix measurementNoise = new SimpleMatrix(unvalidatedMeasurementNoise);
        validateMeasurementNoiseIsSquare(measurementNoise);
        validateMeasurementNoiseOutputDimension(measurementNoise);
        return measurementNoise;
    }

    private boolean validateMeasurementNoiseIsSquare(SimpleMatrix measurementNoise) throws KalmanFilterException{
        int nRows = measurementNoise.numRows();
        int nCols = measurementNoise.numCols();
        if (nRows == nCols){
            return true;
        } else {
            throw new KalmanFilterException("Dimensions of measurement noise matrix not square \n"
                    + "Row dimension of measurement noise: " + nRows +"\n"
                    + "Column dimension of measurement noise: " + nCols + "\n");
        }
    }

    private boolean validateMeasurementNoiseOutputDimension(SimpleMatrix measurementNoise) throws KalmanFilterException{
        int nRows = measurementNoise.numRows();
        int outputRowsD = Dmat.numRows();
        if (nRows == outputRowsD){
            return true;
        } else {
            throw new KalmanFilterException("Dimensions of measurement noise matrix incompatible with dimensions of D matrix \n"
                    + "Output dimension of D matrix: " + outputRowsD +"\n"
                    + "Square dimension of measurement noise matrix: " + nRows + "\n");
        }
    }

}

package kalmanfilter;

import org.ejml.simple.SimpleMatrix;

class StateSpace {

    private SimpleMatrix Amat;
    private SimpleMatrix Bmat;
    private SimpleMatrix Cmat;
    private SimpleMatrix Dmat;
    private SimpleMatrix processNoise;
    private SimpleMatrix measurementNoise;
    private State state;
    private int nPoles;

    StateSpace(double[][] stateTransitionA, double[][] stateTransitionB,
                      double[][] outputTransitionC, double[][] outputTransitionD, double[][] processNoise,
                      double[][] measurementNoise, State state) throws KalmanFilterException {
        this.Amat = validateAmat(stateTransitionA);
        this.Bmat = validateBmat(stateTransitionB);
        this.Cmat = validateCmat(outputTransitionC);
        this.Dmat = validateDmat(outputTransitionD);
        this.processNoise = validateProcessNoise(processNoise);
        this.measurementNoise = validateMeasurementNoise(measurementNoise);
        this.state = state;
    }

    void updateStatePrediction(double[] input) throws KalmanFilterException {
        SimpleMatrix stateTransitionRow;
        SimpleMatrix stateInputRow;
        SimpleMatrix unforcedResponse;
        SimpleMatrix forcedResponse;

        SimpleMatrix stateEstimate = buildRowArrayAsSimpleMatrix(state.getStateEstimate());
        SimpleMatrix inputMat = buildRowArrayAsSimpleMatrix(input);
        double[] newStatePrediction = new double[nPoles];
        for (int i = 0; i < nPoles; i++){
            stateTransitionRow = Amat.extractVector(true, i);
            stateInputRow = Bmat.extractVector(true, i);
            unforcedResponse = matrixMultiply(stateTransitionRow, stateEstimate);
            forcedResponse = matrixMultiply(stateInputRow, inputMat);
            newStatePrediction[i] = matrixSum(unforcedResponse, forcedResponse).get(0);
        }
        state.updateStatePrediction(newStatePrediction);
    }

    void updateCovariancePrediction() throws KalmanFilterException {
        SimpleMatrix noiseFreeCovPrediction = matrixTransform(Amat, new SimpleMatrix(state.getCovarianceEstimate()));
        SimpleMatrix newCovPrediction = matrixSum(noiseFreeCovPrediction, processNoise);
        state.updateCovariancePrediction(newCovPrediction);
    }

    double[] generateOutputPrediction(double[] input) {
        SimpleMatrix outputTransitionRow;
        SimpleMatrix inputFeedthroughRow;
        SimpleMatrix stateDrivenOutput;
        SimpleMatrix inputFeedthroughOutput;

        SimpleMatrix inputMat = buildRowArrayAsSimpleMatrix(input);
        SimpleMatrix statePrediction = buildRowArrayAsSimpleMatrix(state.getStatePrediction());
        double[] newOutputPrediction = new double[Cmat.numRows()];
        for (int i = 0; i < nPoles; i++){
            outputTransitionRow = Cmat.extractVector(true, i);
            inputFeedthroughRow = Dmat.extractVector(true, i);
            stateDrivenOutput = matrixMultiply(outputTransitionRow, statePrediction);
            inputFeedthroughOutput = matrixMultiply(inputFeedthroughRow, inputMat);
            newOutputPrediction[i] = matrixSum(stateDrivenOutput, inputFeedthroughOutput).get(0);
        }
        return newOutputPrediction;
    }

    SimpleMatrix generateKalmanGainMatrix(){
        SimpleMatrix covPrediction = new SimpleMatrix(state.getCovariancePrediction());
        SimpleMatrix leadingTerm = matrixMultiply(covPrediction, Cmat.transpose());
        SimpleMatrix trailingTerm = matrixInverse(matrixSum(matrixTransform(Cmat, covPrediction), measurementNoise));
        return matrixMultiply(leadingTerm, trailingTerm);
    }

    void updateStateEstimate(double[] innovation, SimpleMatrix gainMatrix) throws KalmanFilterException{
        SimpleMatrix statePrediction = buildRowArrayAsSimpleMatrix(state.getStatePrediction());
        SimpleMatrix innovationMat = buildRowArrayAsSimpleMatrix(innovation);
        SimpleMatrix newStateEstimate = matrixSum(statePrediction, matrixMultiply(gainMatrix, innovationMat));
        state.updateStateEstimate(newStateEstimate.getMatrix().getData());
    }

    void updateCovarianceEstimate(SimpleMatrix gainMatrix) throws KalmanFilterException {
        SimpleMatrix covPrediction = new SimpleMatrix(state.getCovariancePrediction());
        SimpleMatrix innerTerm = matrixDifference(matrixIdentity(nPoles), matrixMultiply(gainMatrix, Cmat));
        state.updateCovarianceEstimate(matrixMultiply(innerTerm, covPrediction));
    }

    void incrementTimeStep() {
        state.incrementTimeStep();
    }

    EstimateTimeSeries getStateHistory(){
        return state.getStateHistory();
    }

    private SimpleMatrix matrixTransform(SimpleMatrix xMat, SimpleMatrix yMat){
        return xMat.mult(yMat).mult(xMat.transpose());
    }

    private SimpleMatrix matrixSum(SimpleMatrix xMat, SimpleMatrix yMat){
        return xMat.plus(yMat);
    }

    private SimpleMatrix matrixDifference(SimpleMatrix xMat, SimpleMatrix yMat){
        return xMat.minus(yMat);
    }

    private SimpleMatrix matrixMultiply(SimpleMatrix xMat, SimpleMatrix yMat){
        return xMat.mult(yMat);
    }

    private SimpleMatrix matrixInverse(SimpleMatrix xMat){
        return xMat.invert();
    }

    private SimpleMatrix matrixIdentity(int size){
       return SimpleMatrix.identity(size);
    }

    private SimpleMatrix buildRowArrayAsSimpleMatrix(double[] array) {
        return new SimpleMatrix(array.length, 1, false, array);
    }

    private SimpleMatrix validateAmat(double[][] unvalidatedAmat) throws KalmanFilterException {
        SimpleMatrix Amatrix = new SimpleMatrix(unvalidatedAmat);
        if (Amatrix.numCols() == Amatrix.numRows()) {
            nPoles = Amatrix.numCols();
            return Amatrix;
        } else {
            throw new KalmanFilterException("A matrix must be square.");
        }
    }

    private SimpleMatrix validateBmat(double[][] unvalidatedBmat) throws KalmanFilterException {
        SimpleMatrix Bmatrix = new SimpleMatrix(unvalidatedBmat);
        int nRowsB = Bmatrix.numRows();
        if (nRowsB == nPoles){
            return Bmatrix;
        } else {
            throw new KalmanFilterException("Dimensions of B matrix incompatible with dimensions of A matrix \n"
                + "Square dimension of A: " + nPoles +"\n"
                + "Rows of B: " + nRowsB + "\n");
        }
    }

    private SimpleMatrix validateCmat(double[][] unvalidatedCmat) throws KalmanFilterException {
        SimpleMatrix Cmatrix = new SimpleMatrix(unvalidatedCmat);
        int nColsC = Cmatrix.numCols();
        if (nColsC == nPoles){
            return Cmatrix;
        } else {
            throw new KalmanFilterException("Dimensions of C matrix incompatible with dimensions of state \n"
                    + "Dimension of state: " + nPoles +"\n"
                    + "Columns of C: " + nColsC + "\n");
        }
    }

    private SimpleMatrix validateDmat(double[][] unvalidatedDmat) throws KalmanFilterException {
        SimpleMatrix Dmatrix = new SimpleMatrix(unvalidatedDmat);
        Dmatrix = validateDmatInputSize(Dmatrix);
        Dmatrix = validateDmatOutputSize(Dmatrix);
        return Dmatrix;
    }

    private SimpleMatrix validateDmatInputSize(SimpleMatrix Dmatrix) throws KalmanFilterException {
        int nColsD = Dmatrix.numCols();
        int nInputColsB = Bmat.numCols();
        if (nColsD == nInputColsB){
            return Dmatrix;
        } else {
            throw new KalmanFilterException("Dimensions of D matrix incompatible with input dimensions of B matrix \n"
                    + "Input dimension of B matrix: " + nInputColsB +"\n"
                    + "Input dimension of D matrix: " + nColsD + "\n");
        }
    }

    private SimpleMatrix validateDmatOutputSize(SimpleMatrix Dmatrix) throws KalmanFilterException {
        int nRowsD = Dmatrix.numRows();
        int nOutputRowsC = Cmat.numRows();
        if (nRowsD == nOutputRowsC){
            return Dmatrix;
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
        if (nRows == nCols && nRows == nPoles){
            return processNoise;
        } else {
            throw new KalmanFilterException("Dimensions of process noise matrix incompatible with dimensions of A matrix \n"
                    + "Square dimension of A matrix: " + nPoles +"\n"
                    + "Square dimension of process noise matrix: " + nCols + "\n");
        }
    }

    private SimpleMatrix validateMeasurementNoise(double[][] unvalidatedMeasurementNoise) throws KalmanFilterException{
        SimpleMatrix measurementNoise = new SimpleMatrix(unvalidatedMeasurementNoise);
        measurementNoise = validateMeasurementNoiseIsSquare(measurementNoise);
        measurementNoise = validateMeasurementNoiseOutputDimension(measurementNoise);
        return measurementNoise;
    }

    private SimpleMatrix validateMeasurementNoiseIsSquare(SimpleMatrix measurementNoise) throws KalmanFilterException{
        int nRows = measurementNoise.numRows();
        int nCols = measurementNoise.numCols();
        if (nRows == nCols){
            return measurementNoise;
        } else {
            throw new KalmanFilterException("Dimensions of measurement noise matrix not square \n"
                    + "Row dimension of measurement noise: " + nRows +"\n"
                    + "Column dimension of measurement noise: " + nCols + "\n");
        }
    }

    private SimpleMatrix validateMeasurementNoiseOutputDimension(SimpleMatrix measurementNoise) throws KalmanFilterException{
        int nRows = measurementNoise.numRows();
        int outputRowsD = Dmat.numRows();
        if (nRows == outputRowsD){
            return measurementNoise;
        } else {
            throw new KalmanFilterException("Dimensions of measurement noise matrix incompatible with dimensions of D matrix \n"
                    + "Output dimension of D matrix: " + outputRowsD +"\n"
                    + "Square dimension of measurement noise matrix: " + nRows + "\n");
        }
    }

}

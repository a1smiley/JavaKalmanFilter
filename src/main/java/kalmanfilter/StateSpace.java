package kalmanfilter;

import org.ejml.simple.SimpleMatrix;

class StateSpace {

    private SimpleMatrix Amat;
    private SimpleMatrix Bmat;
    private SimpleMatrix Cmat;
    private double Dmat;
    private SimpleMatrix processNoise;
    private SimpleMatrix measurementNoise;
    private State state;
    private int nPoles;

    StateSpace(double[] stateTransitionPoles, double[][] stateTransitionB,
                      double[][] outputTransitionC, double outputTransitionD, double[][] processNoise,
                      double[][] measurementNoise, State state) throws KalmanFilterException {
        this.Amat = expandPolesToStateTransitionMatrix(stateTransitionPoles);
        this.Bmat = validateBmat(stateTransitionB);
        this.Cmat = validateCmat(outputTransitionC);
        this.Dmat = validateDmat(outputTransitionD);
        this.processNoise = validateProcessNoise(processNoise);
        this.measurementNoise = validateMeasurementNoise(measurementNoise);
        this.state = state;
    }

    void updateStatePrediction(double input) throws KalmanFilterException {

        double[] stateEstimate = state.getStateEstimate();
        double[] newStatePrediction = new double[nPoles];
        for (int i = 0; i < nPoles; i++){
            newStatePrediction[i] = Amat.get( i, i) * stateEstimate[i] + Bmat.get( i, 0) * input;
        }
        state.updateStatePrediction(newStatePrediction);
    }

    void updateCovariancePrediction() throws KalmanFilterException {
        SimpleMatrix noiseFreeCovPrediction = matrixTransform(Amat, new SimpleMatrix(state.getCovarianceEstimate()));
        SimpleMatrix newCovPrediction = matrixSum(noiseFreeCovPrediction, processNoise);
        state.updateCovariancePrediction(newCovPrediction);
    }

    double generateOutputPrediction(double input) {
        double[] statePrediction = state.getStatePrediction();
        double newOutputPrediction = 0;
        for (int i = 0; i < nPoles; i++){
            newOutputPrediction = newOutputPrediction + (Cmat.get(0, i) * statePrediction[i]);
        }
        return newOutputPrediction + Dmat * input;
    }

    SimpleMatrix generateKalmanGainMatrix(){
        SimpleMatrix covPrediction = new SimpleMatrix(state.getCovariancePrediction());
        SimpleMatrix leadingTerm = matrixMultiply(covPrediction, Cmat.transpose());
        SimpleMatrix trailingTerm = matrixInverse(matrixSum(matrixTransform(Cmat, covPrediction), measurementNoise));
        return matrixMultiply(leadingTerm, trailingTerm);
    }

    void updateStateEstimate(double innovation, SimpleMatrix gainMatrix) throws KalmanFilterException{
        double[] newStateEstimate = new double[nPoles];
        double[] statePrediction = state.getStatePrediction();
        for (int i = 0; i < nPoles; i++){
            newStateEstimate[i] = statePrediction[i] + (gainMatrix.get(0, i) * innovation);
        }
        state.updateStateEstimate(newStateEstimate);
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

    private SimpleMatrix expandPolesToStateTransitionMatrix(double[] poles){
        nPoles = poles.length;
        double[][] transitionMatrix = new double[nPoles][nPoles];
        for (int i = 0; i < nPoles; i++){
            for (int j = 0; j < nPoles; j++) {
                if (j == i) {
                    transitionMatrix[i][j] = poles[i];
                } else {
                    transitionMatrix[i][j] = 0;
                }
            }
        }
        return new SimpleMatrix(transitionMatrix);
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

    private double validateDmat(double unvalidatedDmat) throws KalmanFilterException {
        int nElementsD = 1;  // For now, Dmat and number of outputs columns for Cmat constrained to be of size 1, but expect to change on future update
        int nOutputColsC = 1;
        if (nElementsD == nOutputColsC){
            return unvalidatedDmat;
        } else {
            throw new KalmanFilterException("Dimensions of D matrix incompatible with dimensions of C matrix \n"
                    + "Output dimension of C matrix: " + nOutputColsC +"\n"
                    + "Dimension of D matrix: " + nElementsD + "\n");
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
        int nRows = measurementNoise.numRows();
        int nCols = measurementNoise.numCols();
        if (nRows == nCols && nRows == nPoles){
            return measurementNoise;
        } else {
            throw new KalmanFilterException("Dimensions of measurement noise matrix incompatible with dimensions of A matrix \n"
                    + "Square dimension of A matrix: " + nPoles +"\n"
                    + "Square dimension of measurement noise matrix: " + nCols + "\n");
        }
    }

}

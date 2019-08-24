public class StateSpace {

    private double[][] Amat;
    private int nPoles;
    private double[] Bmat;
    private double[] Cmat;
    private double Dmat = 0;
    private double[] processNoise;
    private double[] measurementNoise;
    private State state;

    public StateSpace(double[] stateTransitionPoles, double[] stateTransitionB,
                      double[] outputTransitionC, double outputTransitionD, double[] processNoise,
                      double[] measurementNoise, State state) throws KalmanFilterException {
        Amat = expandPolesToStateTransitionMatrix(stateTransitionPoles);
        Bmat = validateBmat(stateTransitionB);
        Cmat = validateCmat(outputTransitionC);
        Dmat = validateDmat(outputTransitionD);
        this.processNoise = processNoise;
        this.measurementNoise = measurementNoise;
        this.state = state;
    }

    public void updateStatePrediction(double input) throws KalmanFilterException {
        double[] stateEstimate = state.getStateEstimate();
        double[] newStatePrediction = new double[nPoles];
        for (int i = 0; i < nPoles; i++){
            newStatePrediction[i] = Amat[i][i] * stateEstimate[i] + Bmat[i] * input;
        }
        state.updateStatePrediction(newStatePrediction);
    }

    public void updateCovariancePrediction() throws KalmanFilterException {
        double[][] noiseFreeCovPrediction = matrixTransform(Amat, state.getCovarianceEstimate());
        double[][] newCovPrediction = matrixSum(noiseFreeCovPrediction, processNoise);
        state.updateCovariancePrediction(newCovPrediction);
    }

    public double generateOutputPrediction(double input) {
        double[] statePrediction = state.getStatePrediction();
        double newOutputPrediction = 0;
        for (int i = 0; i < nPoles; i++){
            newOutputPrediction = newOutputPrediction + (Cmat[i] * statePrediction[i]);
        }
        return newOutputPrediction + Dmat * input;
    }

    public double[] generateKalmanGainMatrix(){
        double[][] covPrediction = state.getCovariancePrediction();
        double[][] leadingTerm = matrixMultiply(covPrediction, matrixTranspose(Cmat));
        double[][] trailingTerm = matrixInverse(matrixSum(matrixTransform(Cmat, covPrediction), measurementNoise));
        return matrixMultiply(leadingTerm, trailingTerm);
    }

    public void updateStateEstimate(double outputPrediction, double[] gainMatrix) throws KalmanFilterException{
        double[] newStateEstimate = new double[nPoles];
        double[] statePrediction = state.getStatePrediction();
        for (int i = 0; i < nPoles; i++){
            newStateEstimate[i] = statePrediction[i] + (gainMatrix[i] * outputPrediction);
        }
        state.updateStateEstimate(newStateEstimate);
    }

    public void updateCovarianceEstimate(double[] gainMatrix) {
        double[][] covPrediction = state.getCovariancePrediction();
        double[][] innerTerm = matrixDifference(matrixIdentity(nPoles), matrixMultiply(gainMatrix, Cmat));
        state.updateCovarianceEstimate(matrixMultiply(innerTerm, covPrediction));
    }

    public void incrementTimeStep() {
        state.incrementTimeStep();
    }

    public EstimateTimeSeries getStateHistory(){
        return state.getStateHistory();
    }

    private double[][] expandPolesToStateTransitionMatrix(double[] poles){
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
        return transitionMatrix;
    }

    private double[] validateBmat(double[] unvalidatedBmat) throws KalmanFilterException {
        int nElementsB = unvalidatedBmat.length;
        if (nElementsB == nPoles){
            double[] validatedBmat = unvalidatedBmat;
            return validatedBmat;
        } else {
            throw new KalmanFilterException("Dimensions of B matrix incompatible with dimensions of A matrix \n"
                + "Max dimension of A: " + nPoles +"\n"
                + "Max dimension of B: " + nElementsB + "\n");
        }
    }

    private double[] validateCmat(double[] unvalidatedCmat) throws KalmanFilterException {
        int nElementsC = unvalidatedCmat.length;
        if (nElementsC == nPoles){
            double[] validatedCmat = unvalidatedCmat;
            return validatedCmat;
        } else {
            throw new KalmanFilterException("Dimensions of C matrix incompatible with dimensions of state \n"
                    + "Size of state: " + nPoles +"\n"
                    + "Max dimension of C: " + nElementsC + "\n");
        }
    }

    private double validateDmat(double unvalidatedDmat) throws KalmanFilterException {
        int nElementsD = 1;  // For now, Dmat and number of outputs columns for Cmat constrained to be of size 1, but expect to change on future update
        int nOutputColsC = 1;
        if (nElementsD == nOutputColsC){
            double validatedDmat = unvalidatedDmat;
            return validatedDmat;
        } else {
            throw new KalmanFilterException("Dimensions of D matrix incompatible with dimensions of C matrix \n"
                    + "Output dimension of C matrix: " + nOutputColsC +"\n"
                    + "Dimension of D matrix: " + nElementsD + "\n");
        }
    }

}

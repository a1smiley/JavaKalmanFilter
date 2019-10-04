package kalmanfilter;

import org.ejml.alg.block.decomposition.chol.CholeskyOuterForm_B64;
import org.ejml.alg.dense.decomposition.chol.CholeskyDecomposition_B64_to_D64;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.simple.SimpleMatrix;
import org.springframework.stereotype.Component;

@Component
class KalmanFilter {

    private StateSpace systemModel;
    private State state;
    private final EstimateTimeSeries stateHistory = new EstimateTimeSeries();
    private double[] currentOutputPrediction;
    private SimpleMatrix currentGainMatrix;
    private double[] currentInnovation;

    void setSystemModel(StateSpace systemModel, State initialState) {
        this.systemModel = systemModel;
        this.state = initialState;
    }

    EstimateTimeSeries filter(MeasurementSet measurements) throws KalmanFilterException {
        checkAllFieldsInitialized();
        for (int i = 0; i < measurements.getDuration()-1; i++){
            double[] previousInput = measurements.getInputMeasurement(i);
            double[] currentInput = measurements.getInputMeasurement(i+1);
            generateStatePrediction(previousInput);
            generateCovariancePrediction();
            generateOutputPrediction(currentInput);
            generateKalmanGainMatrix();
            calculateInnovation(measurements.getOutputMeasurement(i+1));
            generateStateEstimate();
            generateCovarianceEstimate();
            storeValuesToHistory(i+1);
        }
        return stateHistory;
    }

    private void generateStatePrediction(double[] input) {
        SimpleMatrix stateEstimate = buildRowArrayAsSimpleMatrix(state.getStateEstimate());
        SimpleMatrix inputMat = buildRowArrayAsSimpleMatrix(input);
        int nPoles = systemModel.getNumPoles();

        double[] newStatePrediction = new double[nPoles];
        SimpleMatrix stateTransitionRow;
        SimpleMatrix stateInputRow;
        SimpleMatrix unforcedResponse;
        SimpleMatrix forcedResponse;
        for (int i = 0; i < nPoles; i++){
            stateTransitionRow = systemModel.getAmat().extractVector(true, i);
            stateInputRow = systemModel.getBmat().extractVector(true, i);
            unforcedResponse = matrixMultiply(stateTransitionRow, stateEstimate);
            forcedResponse = matrixMultiply(stateInputRow, inputMat);
            newStatePrediction[i] = matrixSum(unforcedResponse, forcedResponse).get(0);
        }
        state.setStatePrediction(newStatePrediction);
    }

    private void generateCovariancePrediction() {
        SimpleMatrix noiseFreeCovPrediction = matrixTransform(systemModel.getAmat(), new SimpleMatrix(state.getCovarianceEstimate()));
        SimpleMatrix newCovPrediction = matrixSum(noiseFreeCovPrediction, systemModel.getProcessNoise());
        state.setCovariancePrediction(newCovPrediction);
    }

    private void generateOutputPrediction(double[] input) {
        SimpleMatrix outputTransitionRow;
        SimpleMatrix inputFeedthroughRow;
        SimpleMatrix stateDrivenOutput;
        SimpleMatrix inputFeedthroughOutput;

        SimpleMatrix inputMat = buildRowArrayAsSimpleMatrix(input);
        SimpleMatrix statePrediction = buildRowArrayAsSimpleMatrix(state.getStatePrediction());
        double[] newOutputPrediction = new double[systemModel.getCmat().numRows()];

        for (int i = 0; i < systemModel.getNumOutput(); i++){
            outputTransitionRow = systemModel.getCmat().extractVector(true, i);
            inputFeedthroughRow = systemModel.getDmat().extractVector(true, i);
            stateDrivenOutput = matrixMultiply(outputTransitionRow, statePrediction);
            inputFeedthroughOutput = matrixMultiply(inputFeedthroughRow, inputMat);
            newOutputPrediction[i] = matrixSum(stateDrivenOutput, inputFeedthroughOutput).get(0);
        }
        this.currentOutputPrediction = newOutputPrediction;
    }

    private void generateKalmanGainMatrix(){
        SimpleMatrix covPrediction = new SimpleMatrix(state.getCovariancePrediction());
        SimpleMatrix leadingTerm = matrixMultiply(covPrediction, systemModel.getCmat().transpose());
        SimpleMatrix trailingTerm = matrixInverse(matrixSum(matrixTransform(systemModel.getCmat(), covPrediction), systemModel.getMeasurementNoise()));
        this.currentGainMatrix = matrixMultiply(leadingTerm, trailingTerm);
    }

    private void calculateInnovation(double[] measurement) {
        double[] innovation = new double[measurement.length];
        for (int i = 0; i < measurement.length; i++) {
            innovation[i] = measurement[i] - this.currentOutputPrediction[i];
        }
        this.currentInnovation = innovation;
    }

    private void generateStateEstimate() {
        SimpleMatrix statePrediction = buildRowArrayAsSimpleMatrix(state.getStatePrediction());
        SimpleMatrix innovationMat = buildRowArrayAsSimpleMatrix(currentInnovation);
        SimpleMatrix newStateEstimate = matrixSum(statePrediction, matrixMultiply(currentGainMatrix, innovationMat));
        state.setStateEstimate(newStateEstimate.getMatrix().getData());
    }

    private void generateCovarianceEstimate() {
        SimpleMatrix covPrediction = new SimpleMatrix(state.getCovariancePrediction());
        SimpleMatrix innerTerm = matrixDifference(matrixIdentity(systemModel.getNumPoles()), matrixMultiply(currentGainMatrix, systemModel.getCmat()));
        state.setCovarianceEstimate(matrixMultiply(innerTerm, covPrediction));
    }

    private void storeValuesToHistory(int timeIndex) throws KalmanFilterException {
        stateHistory.storeState(timeIndex, this.state);
        stateHistory.storeOutputPrediction(timeIndex, new DoubleArray(this.currentOutputPrediction));
        stateHistory.storeGain(timeIndex, get2DArray(this.currentGainMatrix));
        stateHistory.storeInnovation(timeIndex, new DoubleArray(this.currentInnovation));
    }

    private void checkAllFieldsInitialized() throws KalmanFilterException {
        if (systemModel == null) {
            throw new KalmanFilterException("System model must be defined prior to evaluating filter");
        }
    }

    private Double2DArray get2DArray(SimpleMatrix simpleMatrix) {
        int nCols = simpleMatrix.numCols();
        int nRows = simpleMatrix.numRows();
        double[] primitive1DArray = simpleMatrix.getMatrix().getData();
        double[][] primitive2DArray = new double[nCols][nRows];

        for (int i = 0; i < nCols; i++) {
            //noinspection ManualArrayCopy
            for (int j = 0; j < nRows; j++) {
                primitive2DArray[i][j] = primitive1DArray[i * nCols + j];
            }
        }
        return new Double2DArray(primitive2DArray);
    }

    private SimpleMatrix buildRowArrayAsSimpleMatrix(double[] array) {
        return new SimpleMatrix(array.length, 1, false, array);
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

}

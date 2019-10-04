package kalmanfilter;

import lombok.Getter;
import lombok.Setter;
import org.ejml.simple.SimpleMatrix;

@Getter
class State {
    @Setter
    private double[] statePrediction;
    @Setter
    private double[] stateEstimate;
    private double[][] covariancePrediction;
    private double[][] covarianceEstimate;

    State(double[] initialState, double[][] initialCovariance) {
        this.statePrediction = initialState;
        this.stateEstimate = initialState;
        this.covariancePrediction = initialCovariance;
        this.covarianceEstimate = initialCovariance;
    }

    void setCovariancePrediction(SimpleMatrix newCovPrediction) {
            this.covariancePrediction = getPrimitive2DArray(newCovPrediction);
    }

    void setCovariancePrediction(double[][] newCovPrediction) {
        this.covariancePrediction = newCovPrediction;
    }

    void setCovarianceEstimate(SimpleMatrix newCovEstimate) {
        this.covarianceEstimate = getPrimitive2DArray(newCovEstimate);
    }

    void setCovarianceEstimate(double[][] newCovEstimate) {
        this.covarianceEstimate = newCovEstimate;
    }

    private double[][] getPrimitive2DArray(SimpleMatrix simpleMatrix) {
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
        return primitive2DArray;
    }

}

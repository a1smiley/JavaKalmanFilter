package kalmanfilter;

import lombok.Data;

@Data
public class StateSpaceDTO {
    double[][] stateTransitionA;
    double[][] stateTransitionB;
    double[][] outputTransitionC;
    double[][] outputTransitionD;
    double[][] processNoise;
    double[][] measurementNoise;
    double[] initialState;
    double[][] initialCovariance;
}

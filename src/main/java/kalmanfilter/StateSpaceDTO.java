package kalmanfilter;

import lombok.Data;

@Data
public class StateSpaceDTO {
    double[] stateTransitionPoles;
    double[][] stateTransitionB;
    double[][] outputTransitionC;
    double ouputTransitionD;
    double[][] processNoise;
    double[][] measurementNoise;
    double[] initialState;
    double[][] initialCovariance;
}

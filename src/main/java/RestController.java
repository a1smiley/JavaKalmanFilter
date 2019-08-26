import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.PutMapping;

@org.springframework.web.bind.annotation.RestController
public class RestController {
    @Autowired
    LinearKalmanFilter linearKF;

    @PutMapping("kalmanfilter/setup")
    public ResponseEntity kalmanFilterSetup(double[] stateTransitionPoles, double[][] stateTransitionB,
                                            double[][] outputTransitionC, double ouputTransitionD,
                                            double[][] processNoise, double[][] measurementNoise,
                                            double[] initialState, double[][] initialCovariance) {
        try {
            State state = new State(initialState, initialCovariance);
            StateSpace stateSpace = new StateSpace(stateTransitionPoles, stateTransitionB, outputTransitionC, ouputTransitionD,
                    processNoise, measurementNoise, state);
            linearKF.setSystemModel(stateSpace);
            return ResponseEntity.status(HttpStatus.ACCEPTED).build();
        } catch (KalmanFilterException e) {
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e);
        }
    }

    @PostMapping("kalmanfilter/getEstimateTimeSeries")
    public ResponseEntity runFilter(MeasurementSet measurements) throws KalmanFilterException {
        EstimateTimeSeries filterOutput = linearKF.filter(measurements);
        return ResponseEntity.status(HttpStatus.OK).body(filterOutput);
    }

}

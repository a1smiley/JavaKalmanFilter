package kalmanfilter;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.PutMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestParam;

@org.springframework.web.bind.annotation.RestController
public class RestController {
    @Autowired
    LinearKalmanFilter linearKF;

    @PutMapping(path = "kalmanfilter/setup")
    public ResponseEntity<String> kalmanFilterSetup(@RequestBody StateSpaceDTO input) {
        try {
            State state = new State(input.initialState, input.initialCovariance);
            StateSpace stateSpace = new StateSpace(input.stateTransitionA, input.stateTransitionB,
                    input.outputTransitionC, input.outputTransitionD, input.processNoise, input.measurementNoise,
                    state);
            linearKF.setSystemModel(stateSpace);
            return ResponseEntity.status(HttpStatus.ACCEPTED).body("Success");
        } catch (KalmanFilterException e) {
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e.getMessage());
        }
    }

    @PostMapping("kalmanfilter/getEstimateTimeSeries")
    public ResponseEntity<EstimateTimeSeries> runFilter(@RequestBody MeasurementSet measurements) throws KalmanFilterException {
        EstimateTimeSeries filterOutput = linearKF.filter(measurements);
        return ResponseEntity.status(HttpStatus.OK).body(filterOutput);
    }

}

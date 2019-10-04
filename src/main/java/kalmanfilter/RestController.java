package kalmanfilter;

import com.google.gson.Gson;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.PutMapping;
import org.springframework.web.bind.annotation.RequestBody;

@org.springframework.web.bind.annotation.RestController
@Log4j2
public class RestController {

    KalmanFilter linearKF = new KalmanFilter();
    Gson gson = new Gson();

    @PutMapping(path = "kalmanfilter/setup")
    public ResponseEntity<String> kalmanFilterSetup(@RequestBody StateSpaceDTO input) {
        try {
            State state = new State(input.initialState, input.initialCovariance);
            StateSpace stateSpace = new StateSpace(input.stateTransitionA, input.stateTransitionB,
                    input.outputTransitionC, input.outputTransitionD, input.processNoise, input.measurementNoise);
            linearKF.setSystemModel(stateSpace, state);
            log.info(() -> "Set system model from REST input");
            return ResponseEntity.status(HttpStatus.ACCEPTED).body("Success");
        } catch (KalmanFilterException e) {
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e.getMessage());
        }
    }

    @PostMapping("kalmanfilter/getEstimateTimeSeries")
    public ResponseEntity<String> runFilter(@RequestBody MeasurementSet measurements) {
        try {
            EstimateTimeSeries filterOutput = linearKF.filter(measurements);
            String jsonFilterOutput = gson.toJson(filterOutput);
            return ResponseEntity.status(HttpStatus.OK).body(jsonFilterOutput);
        } catch (KalmanFilterException e) {
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e.getMessage());
        }
    }

}

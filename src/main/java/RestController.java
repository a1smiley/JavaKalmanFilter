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
    public ResponseEntity kalmanFilterSetup(double[] stateTransitionPoles,
                                            double[] stateTransitionB,
                                            double[] outputTransitionC,
                                            double ouputTransitionD,
                                            double[] initialState) {
        try {
            State state = new State(initialState);
            StateSpace stateSpace = new StateSpace(stateTransitionPoles, stateTransitionB, outputTransitionC, ouputTransitionD, state);
            linearKF.setSystemModel(stateSpace);
            return ResponseEntity.status(HttpStatus.ACCEPTED).build();
        } catch (KalmanFilterException e) {
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e);
        }
    }

    @PostMapping("kalmanfilter/getEstimateTimeSeries")
    public ResponseEntity runFilter(double[][] measurements) throws KalmanFilterException {
        MeasurementSet measurementSet = new MeasurementSet();
        measurementSet.setOutputMeasurements(measurements);
        EstimateTimeSeries filterOutput = linearKF.filter(measurementSet);
        return ResponseEntity.status(HttpStatus.OK).body(filterOutput);
    }

}

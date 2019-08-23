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
        StateSpace stateSpace = new StateSpace(stateTransitionPoles, stateTransitionB, outputTransitionC, ouputTransitionD);
        State state = new State(initialState);
        linearKF.defineSystemModel(stateSpace);
        linearKF.initializeState(state);
        return ResponseEntity.status(HttpStatus.ACCEPTED).build();
    }

    @PostMapping("kalmanfilter/getEstimateTimeSeries")
    public ResponseEntity runFilter(double[][] measurements){
        MeasurementSet preProcessedMeasurements = new MeasurementSet(measurements);
        EstimateTimeSeries filterOutput = linearKF.filter(preProcessedMeasurements);
        return ResponseEntity.status(HttpStatus.OK).body(filterOutput);
    }

}

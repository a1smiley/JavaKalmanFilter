public class StateSpace {

    public double[][] Amat;
    public double[] Bmat;
    public double[] Cmat;
    public double Dmat = 0;


    public StateSpace(double[] stateTransitionPoles, double[] stateTransitionB,
                      double[] outputTransitionC, double outputTransitionD){
        Amat = expandPolesToStateTransitionMatrix(stateTransitionPoles);
        Bmat = stateTransitionB;
        Cmat = outputTransitionC;
        Dmat = outputTransitionD;
    }

    private double[][] expandPolesToStateTransitionMatrix(double[] poles){
        int nPoles = poles.length;
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

}

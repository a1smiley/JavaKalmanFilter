package kalmanfilter;

class DoubleArray {

    private final double[] array;

    DoubleArray(double[] array){
        this.array = array;
    }

    double[] getPrimitiveArray(){
        return this.array;
    }
}

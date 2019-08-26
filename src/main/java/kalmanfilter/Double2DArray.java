package kalmanfilter;

class Double2DArray {

    private final double[][] array;

    Double2DArray(double[][] array){
        this.array = array;
    }

    double[][] getPrimitiveArray(){
        return this.array;
    }
}

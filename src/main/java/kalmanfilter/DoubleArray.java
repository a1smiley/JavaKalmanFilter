package kalmanfilter;

import lombok.Getter;

class DoubleArray {

    @Getter
    private final double[] array;

    DoubleArray(double[] array){
        this.array = array;
    }
}

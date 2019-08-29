package kalmanfilter;

import lombok.Getter;

class Double2DArray {

    @Getter
    private final double[][] array;

    Double2DArray(double[][] array){
        this.array = array;
    }
}

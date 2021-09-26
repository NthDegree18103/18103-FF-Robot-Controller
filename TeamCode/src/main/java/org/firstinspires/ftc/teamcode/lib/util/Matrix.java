package org.firstinspires.ftc.teamcode.lib.util;

import java.util.Arrays;

public class Matrix {

    Double[][] array;

    public Matrix(Double[][] array) {
        this.array = array;
    }

    public static void main(String[] args) {
        Matrix a = new Matrix(new Double[][]{{1d, 1d},
                                            {2d, 2d}});
        Matrix b = new Matrix(new Double[][]{{3d, 3d},
                                            {4d, 4d}});

        System.out.println(a.scale(2));
        System.out.println(b);
        System.out.println(a.add(b));
        System.out.println(b.add(a));
    }

    public Matrix add(Matrix matrix) {
        Double[][] list = new Double[array.length][array[0].length];
        for (int i = 0; i < list.length; i++) {
            for (int j = 0; j < list[i].length; j++) {
                list[i][j] = array[i][j] + matrix.array[i][j];
            }
        }
        return new Matrix(list);
    }

    public Matrix scale(double n) {
        for (int i = 0; i < array.length; i++) {
            for (int j = 0; j < array[i].length; j++) {
                array[i][j] = n * array[i][j];
            }
        }
        return this;
    }

    @Override
    public String toString() {
        return Arrays.deepToString(array);
    }

    public Double[][] getArray() {
        return array;
    }
}

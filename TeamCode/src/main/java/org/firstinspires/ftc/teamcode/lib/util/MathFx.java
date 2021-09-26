package org.firstinspires.ftc.teamcode.lib.util;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * Author: Akhil G
 */

public class MathFx {

    public static double scale(double lower, double upper, double value) {
        return Math.max(lower, Math.min(value, upper));
    }

    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    public static OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public static String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }

    public static double angleWrap(double lower, double upper, double angle) {
        while (angle < lower) {
            angle += 360;
        }

        while (angle > upper) {
            angle -= 360;
        }

        return angle;

    }

    public static double radAngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        return angle;
    }

    public static double meanDataFusion(double[] data, double[] weights, double bias) {
        double sum = 0;
        double len = 0;
        for (int i = 0; i < data.length; i++) {
            sum += data[i] * weights[i];
            len += weights[i];
        }
        return (sum / len) + bias;
    }

    public static double lowPassFilter(double a, double last, double curr) {
        return last*a + (1-a)*curr;
    }

    public static double linearInterpolation(double lower, double higher, double time) {
        double slope = (higher - lower);
        double bias = lower - (slope * ((int) time));
        return slope * time + bias;
    }

}

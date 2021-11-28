package org.firstinspires.ftc.teamcode.lib.physics;

public class Kinematic {

    double x;
    double y;

    public Kinematic() {
        x = 0.0;
        y = 0.0;
    }

    public Kinematic(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double X() {
        return x;
    }

    public double Y() {
        return y;
    }

    public Kinematic scale(double s) {
        return new Kinematic(x*s, y*s);
    }

    public Kinematic add(Kinematic val) {
        return new Kinematic(x + val.x, y + val.y);
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public static Kinematic dataFusion(Kinematic[] data, double[][] weights) {
        double sumX = 0;
        double sumY = 0;
        double xLen = 0;
        double yLen = 0;
        for (int i = 0; i < data.length; i++) {
            sumX += data[i].X() * weights[0][i];
            sumY += data[i].Y() * weights[1][i];
            xLen += weights[0][i];
            yLen += weights[1][i];
        }
        return new Kinematic(sumX / xLen, sumY / yLen);
    }

    public void filter(double threshold) {
        if (Math.abs(x) < threshold) {
            x = 0;
        }
        if (Math.abs(y) < threshold) {
            y = 0;
        }
    }

}

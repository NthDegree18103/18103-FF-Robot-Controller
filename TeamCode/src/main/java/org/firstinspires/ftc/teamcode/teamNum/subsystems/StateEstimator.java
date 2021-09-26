package org.firstinspires.ftc.teamcode.teamNum.subsystems;

import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.teamNum.states.State;
import org.firstinspires.ftc.teamcode.teamNum.states.drive.IMU;
import org.firstinspires.ftc.teamcode.teamNum.states.drive.MKE;
import org.firstinspires.ftc.teamcode.teamNum.states.drive.VV;

public class StateEstimator implements Subsystem, State {

    IMU imu;
    MKE mke;
    VV vv;
    double x, y, theta, xBias, yBias, thetaBias, x_dot, y_dot;

    public StateEstimator(IMU imu, MKE mke, VV vv) {
        this.imu = imu;
        this.mke = mke;
        this.vv = vv;
        xBias = 0;
        yBias = 0;
        thetaBias = 0;
        x_dot = 0;
        y_dot = 0;
        updatePos();
    }

    public StateEstimator(IMU imu, MKE mke, VV vv, double x, double y, double theta) {
        this.imu = imu;
        this.mke = mke;
        this.vv = vv;
        xBias = x;
        yBias = y;
        thetaBias = theta;
        x_dot = 0;
        y_dot = 0;
        updatePos();
    }

    @Override
    public void update(double dt) {
        imu.update(dt);
        mke.update(dt);
        vv.update(dt);
        updatePos();
    }

    public void updatePos() {
        x = MathFx.meanDataFusion(new double[]{imu.getX(), mke.getX(), vv.getX()},
                                    new double[]{1, 1, 1}, xBias);

        y = MathFx.meanDataFusion(new double[]{imu.getY(), mke.getY(), vv.getY()},
                                    new double[]{1, 1, 1}, yBias);

        theta = MathFx.meanDataFusion(new double[]{imu.getTheta(), mke.getTheta(), vv.getTheta()},
                                    new double[]{1, 1, 1}, thetaBias);

        x_dot = MathFx.meanDataFusion(new double[]{imu.getX_dot(), mke.getX_dot(), vv.getX_dot()},
                new double[]{1, 1, 1}, 0);

        y_dot = MathFx.meanDataFusion(new double[]{imu.getY_dot(), mke.getY_dot(), vv.getY_dot()},
                new double[]{1, 1, 1}, 0);

    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public void setxBias(double xBias) {
        this.xBias = xBias;
    }

    public void setyBias(double yBias) {
        this.yBias = yBias;
    }

    public void setThetaBias(double thetaBias) {
        this.thetaBias = thetaBias;
    }

    public void zeroTheta() {
        setThetaBias(-getTheta());
    }

    public void zeroPos() {
        zeroTheta();
        setxBias(-getX());
        setyBias(-getY());
    }

    public double getY_dot() {
        return y_dot;
    }

    public double getX_dot() {
        return x_dot;
    }

    public double getX_dDot() {return imu.getX_dDot();}

    public double getY_dDot() {return imu.getY_dDot();}

}

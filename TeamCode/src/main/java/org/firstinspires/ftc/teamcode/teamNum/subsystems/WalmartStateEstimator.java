package org.firstinspires.ftc.teamcode.teamNum.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.teamNum.states.State;
import org.firstinspires.ftc.teamcode.teamNum.states.drive.IMU;
import org.firstinspires.ftc.teamcode.teamNum.states.drive.MKE;
import org.firstinspires.ftc.teamcode.teamNum.states.drive.VV;

public class WalmartStateEstimator extends StateEstimator implements Subsystem, State {

    public WalmartStateEstimator(IMU imu, MKE mke) {
        super(imu, mke, null);
    }

    public WalmartStateEstimator(IMU imu, MKE mke, double x, double y, double theta) {
        super(imu, mke, null, x, y, theta);
    }

    @Override
    public void update(double dt, Telemetry telemetry) {
        imu.update(dt, telemetry);
        mke.update(dt, telemetry);
        updatePos();
    }

    public void updatePos() {
        x = MathFx.meanDataFusion(new double[]{imu.getX(), mke.getX()},
                                    new double[]{1, 1}, xBias);

        y = MathFx.meanDataFusion(new double[]{imu.getY(), mke.getY()},
                                    new double[]{1, 1}, yBias);

        theta = MathFx.meanDataFusion(new double[]{imu.getTheta(), mke.getTheta()},
                                    new double[]{1, 1}, thetaBias);

        x_dot = MathFx.meanDataFusion(new double[]{imu.getX_dot(), mke.getX_dot()},
                new double[]{1, 1}, 0);

        y_dot = MathFx.meanDataFusion(new double[]{imu.getY_dot(), mke.getY_dot()},
                new double[]{1, 1}, 0);

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

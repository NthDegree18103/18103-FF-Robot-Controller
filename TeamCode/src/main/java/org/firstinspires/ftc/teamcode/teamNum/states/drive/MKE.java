package org.firstinspires.ftc.teamcode.teamNum.states.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.lib.util.Matrix;
import org.firstinspires.ftc.teamcode.teamNum.Constants;
import org.firstinspires.ftc.teamcode.teamNum.states.DriveState;

public class MKE extends DriveState {

    DcMotorEx fl, fr, bl, br;
    double fl_0, fr_0, bl_0, br_0;
    double x, y, theta, x_dot, y_dot;

    public MKE(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;

        fl_0 = 0;
        bl_0 = 0;
        fr_0 = 0;
        br_0 = 0;

        x = 0;
        y = 0;
        theta = 0;
        x_dot = 0;
        y_dot = 0;
    }

    public MKE(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br, double x, double y, double theta) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;

        fl_0 = 0;
        bl_0 = 0;
        fr_0 = 0;
        br_0 = 0;

        this.x = x;
        this.y = y;
        this.theta = theta;
        x_dot = 0;
        y_dot = 0;
    }

    /*
     * MKESim.java Tested
     */
    @Override
    public void update(double dt) {
        double fl_1 = fl.getCurrentPosition() - fl_0;
        double fr_1 = fr.getCurrentPosition() - fr_0;
        double bl_1 = bl.getCurrentPosition() - bl_0;
        double br_1 = br.getCurrentPosition() - br_0;

        Matrix v1 =  new Matrix(new Double[][]{{-fl_1}, {fl_1}, {fl_1}});
        Matrix v2 =  new Matrix(new Double[][]{{fr_1}, {fr_1}, {-fr_1}});
        Matrix v3 =  new Matrix(new Double[][]{{-br_1}, {br_1}, {-br_1}});
        Matrix v4 =  new Matrix(new Double[][]{{bl_1}, {bl_1}, {bl_1}});

        Matrix v = v1.add(v2).add(v3).add(v4).scale(Motors.GoBILDA_312.getDistPerTicks(Constants.R) / 4);

        System.out.println(v);

        v.getArray()[2][0] = v.getArray()[2][0] / (Constants.L);

        double theta_1 = theta + v.getArray()[2][0];
        theta_1 = MathFx.radAngleWrap(theta_1);

        double a = v.getArray()[1][0]*v.getArray()[1][0] + v.getArray()[0][0]*v.getArray()[0][0];
        double x2 = MathFx.lowPassFilter(.8, x, x + Math.sqrt(a) * Math.cos(-theta_1 + Math.PI/2));
        double y2 = MathFx.lowPassFilter(.8, y, y += Math.sqrt(a) * Math.sin(-theta_1 + Math.PI/2));

        x_dot = (x2 - x)/dt;
        y_dot = (y2 - y)/dt;

        x = x2;
        y = y2;

        theta_1 += v.getArray()[2][0];
        theta_1 = MathFx.radAngleWrap(theta_1);
        theta = MathFx.lowPassFilter(.8, theta, theta_1);

        fl_0 += fl_1;
        fr_0 += fr_1;
        bl_0 += bl_1;
        br_0 += br_1;
    }

    @Override
    public double getTheta() {
        return theta;
    }
    @Override
    public double getX() {
        return x;
    }
    @Override
    public double getY() {
        return y;
    }

    @Override
    public double getX_dot() {
        return x_dot;
    }

    @Override
    public double getY_dot() {
        return y_dot;
    }

}

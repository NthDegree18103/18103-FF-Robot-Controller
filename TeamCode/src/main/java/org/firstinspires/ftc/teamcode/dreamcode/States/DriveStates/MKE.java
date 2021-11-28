package org.firstinspires.ftc.teamcode.dreamcode.States.DriveStates;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.physics.Kinematic;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.lib.util.Matrix;
import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveState;

public class MKE extends DriveState {

    DcMotorEx fl, fr, bl, br;
    double fl_0, fr_0, bl_0, br_0;
    double x, y, theta, x_dot, y_dot, a_dot;

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
        a_dot = 0;
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
        a_dot = 0;
    }

    /*
     * MKESim.java Tested
     */
    @Override
    /*public void update(double dt, Telemetry telemetry) {
        double fl_1 = fl.getCurrentPosition() - fl_0;
        double fr_1 = fr.getCurrentPosition() - fr_0;
        double bl_1 = bl.getCurrentPosition() - bl_0;
        double br_1 = br.getCurrentPosition() - br_0;

        Matrix v1 =  new Matrix(new Double[][]{{-fl_1}, {fl_1}, {fl_1}});
        Matrix v2 =  new Matrix(new Double[][]{{fr_1}, {fr_1}, {-fr_1}});
        Matrix v3 =  new Matrix(new Double[][]{{-br_1}, {br_1}, {-br_1}});
        Matrix v4 =  new Matrix(new Double[][]{{bl_1}, {bl_1}, {bl_1}});

        Matrix v = v1.add(v2).add(v3).add(v4).scale(Motors.GoBILDA_312.getDistPerTicks(Constants.R) / 4);

        //System.out.println(v);

        v.getArray()[2][0] = v.getArray()[2][0] / (Constants.L);

        double theta_1 = theta + v.getArray()[2][0];
        theta_1 = MathFx.radAngleWrap(theta_1);

        double a = v.getArray()[1][0]*v.getArray()[1][0] + v.getArray()[0][0]*v.getArray()[0][0];
        double x2 = MathFx.lowPassFilter(.8, x, x + Math.sqrt(a) * Math.cos(-theta_1 + Math.PI/2));
        double y2 = MathFx.lowPassFilter(.8, y, y + Math.sqrt(a) * Math.sin(-theta_1 + Math.PI/2));

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

        telemetry.addData("X: ", getX());
        telemetry.addData("Y: ", getY());
        telemetry.addData("A: ", getA());

    }

     */
    public void update(double dt, Telemetry telemetry) {
        double fl_1 = (MathFx.lowPassFilter(0.9, -fl.getCurrentPosition(), fl_0) - fl_0)/dt;
        double fr_1 = (MathFx.lowPassFilter(0.9, -fr.getCurrentPosition(), fr_0) - fr_0)/dt;
        double bl_1 = (MathFx.lowPassFilter(0.9, bl.getCurrentPosition(), bl_0) - bl_0)/dt;
        double br_1 = (MathFx.lowPassFilter(0.9, br.getCurrentPosition(), br_0) - br_0)/dt;

        /*telemetry.addData("fl: ", fl.getCurrentPosition());
        telemetry.addData("fr: ", fr.getCurrentPosition());
        telemetry.addData("bl: ", bl.getCurrentPosition());
        telemetry.addData("br: ", br.getCurrentPosition());*/

        x_dot = Motors.GoBILDA_435.getDistPerTicks(Constants.R) * (fr_1 + bl_1 + br_1 + fl_1) / 6;// /4 * 2/3 scale factor
        y_dot = Motors.GoBILDA_435.getDistPerTicks(Constants.R) * (-fr_1 + bl_1 - br_1 + fl_1) * 9 / 56; // 6/7 * 3/4
        a_dot = Motors.GoBILDA_435.getDistPerTicks(Constants.R) * (-fr_1 - bl_1 + br_1 + fl_1) / (4 * 1.1 * (Constants.L/2 + Constants.B/2));

        theta += a_dot*dt;

        /*telemetry.addData("Theta: ", theta);*/

        double xd = x_dot*Math.cos(theta) - y_dot*Math.sin(theta);
        double yd = x_dot*Math.sin(theta) + y_dot*Math.cos(theta);

        x_dot = xd;
        y_dot = yd;

        //telemetry.addData("DXA: ", x_dot);
        //telemetry.addData("DYA: ", y_dot);

        x += x_dot*dt;
        y += y_dot*dt;

        fl_0 += fl_1*dt;
        fr_0 += fr_1*dt;
        bl_0 += bl_1*dt;
        br_0 += br_1*dt;

        /*telemetry.addData("X: ", getX());
        telemetry.addData("Y: ", getY());
        telemetry.addData("A: ", getA());
        telemetry.addData("DX: ", x_dot);
        telemetry.addData("DY: ", y_dot);*/

    }

    @Override
    public double getA() {
        return theta;
    }

    @Override
    public Kinematic getPos() {
        return new Kinematic(getX(), getY());
    }

    @Override
    public double getX() {
        return x;
    }
    @Override
    public double getY() {
        return y;
    }

    public double getX_dot() {
        return x_dot;
    }

    public double getY_dot() {
        return y_dot;
    }

}
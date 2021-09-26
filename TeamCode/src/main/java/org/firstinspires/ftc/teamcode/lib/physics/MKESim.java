package org.firstinspires.ftc.teamcode.lib.physics;

import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.lib.util.Matrix;
import org.firstinspires.ftc.teamcode.teamNum.Constants;

public class MKESim {

    double fl_0, fr_0, bl_0, br_0;
    double x, y, theta;

    public MKESim() {
        fl_0 = 0;
        bl_0 = 0;
        fr_0 = 0;
        br_0 = 0;

        x = 0;
        y = 0;
        theta = 0;
    }

    public static void main(String[] args) {
        MKESim sim = new MKESim();
        sim.update(13.5*Math.PI/ Motors.GoBILDA_312.getDistPerTicks(2d), 4.5*Math.PI/ Motors.GoBILDA_312.getDistPerTicks(2d),
                13.5*Math.PI/ Motors.GoBILDA_312.getDistPerTicks(2d), 4.5*Math.PI/ Motors.GoBILDA_312.getDistPerTicks(2d));
        System.out.println(sim.x);
        System.out.println(sim.y);
        System.out.println(Math.toDegrees(sim.theta));
    }

    public void update(double fl, double fr, double bl, double br) {
        double fl_1 = fl - fl_0;
        double fr_1 = fr - fr_0;
        double bl_1 = bl - bl_0;
        double br_1 = br - br_0;

        Matrix v1 =  new Matrix(new Double[][]{{-fl_1}, {fl_1}, {fl_1}});
        Matrix v2 =  new Matrix(new Double[][]{{fr_1}, {fr_1}, {-fr_1}});
        Matrix v3 =  new Matrix(new Double[][]{{-br_1}, {br_1}, {-br_1}});
        Matrix v4 =  new Matrix(new Double[][]{{bl_1}, {bl_1}, {bl_1}});

        Matrix v = v1.add(v2).add(v3).add(v4).scale(Motors.GoBILDA_312.getDistPerTicks(2d) / 4); // 2 in

        System.out.println(v);

        v.getArray()[2][0] = v.getArray()[2][0] / (Constants.L);

        theta += v.getArray()[2][0];
        theta = MathFx.radAngleWrap(theta);

        double a = v.getArray()[1][0]*v.getArray()[1][0] + v.getArray()[0][0]*v.getArray()[0][0];
        x += Math.sqrt(a) * Math.cos(-theta + Math.PI/2);
        y += Math.sqrt(a) * Math.sin(-theta + Math.PI/2);

        theta += v.getArray()[2][0];
        theta = MathFx.radAngleWrap(theta);

        fl_0 += fl_1;
        fr_0 += fr_1;
        bl_0 += bl_1;
        br_0 += br_1;

    }

}

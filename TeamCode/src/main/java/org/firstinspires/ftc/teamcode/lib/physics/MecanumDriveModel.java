package org.firstinspires.ftc.teamcode.lib.physics;

import org.firstinspires.ftc.teamcode.lib.drivers.Motors;

public class MecanumDriveModel implements Model {

    MotorModel fl, fr, bl, br;
    MotorModel[] driveMotors;
    double x, y, T, xd, yd, Td;
    double r, l, b;


    public MecanumDriveModel(double r, double l, double b) {
        fl = new MotorModel(Motors.GoBILDA_312, 100d, 2d, 0.8, 1);
        fr = new MotorModel(Motors.GoBILDA_312, 100d, 2d, 0.8, 1);
        bl = new MotorModel(Motors.GoBILDA_312, 100d, 2d, 0.8, 1);
        br = new MotorModel(Motors.GoBILDA_312, 100d, 2d, 0.8, 1);
        driveMotors = new MotorModel[]{fl, fr, bl, br};
        this.r = r;
        this.b = b / 2;
        this.l = l / 2;
        reset();
    }

    public static void main(String[] args) {
        MecanumDriveModel model = new MecanumDriveModel(2, 16, 16);
        for (int i = 0; i * 0.01 < 5; i++) {
            System.out.print(i + " ");
            model.run(12d, 12d, 12d, 12d,0.01);
            System.out.println(model.getX());
            //prev_time = timeStamp;
        }
    }

    public void run(double flPow, double frPow, double brPow, double blPow, double dt) {
        fl.run(flPow, dt);
        fr.run(frPow, dt);
        bl.run(blPow, dt);
        br.run(brPow, dt);

        xd = r * (bl.getV() + br.getV() + fl.getV() + fr.getV())/4;
        yd = r * (bl.getV() - br.getV() - fl.getV() + fr.getV())/4;
        Td = r * (-bl.getV() + br.getV() - fl.getV() + fr.getV())/(4 * (l + b));

        xd = Math.cos(T)*xd - Math.sin(T)*yd;
        yd = Math.sin(T)*xd + Math.cos(T)*yd;

        x += xd*dt;
        y += yd*dt;
        T += Td*dt;
    }

    public void run(double pow, double dt) {
        for (MotorModel i : driveMotors) {i.run(pow, dt);}
        xd = r * (bl.getV() + br.getV() + fl.getV() + fr.getV())/4;
        yd = r * (bl.getV() - br.getV() - fl.getV() + fr.getV())/4;
        yd = r * (-bl.getV() + br.getV() - fl.getV() + fr.getV())/(4 * (l + b));
        x += xd*dt;
        y += yd*dt;
        T += Td*dt;
    }

    public void reset() {
        x = 0;
        y = 0;
        T = 0;
        xd = 0;
        yd = 0;
        Td = 0;
    }

    public double getY() {
        return y;
    }

    public double getX() {
        return x;
    }

}

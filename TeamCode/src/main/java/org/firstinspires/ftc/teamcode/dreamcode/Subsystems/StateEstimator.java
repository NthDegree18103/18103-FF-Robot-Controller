package org.firstinspires.ftc.teamcode.dreamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveStates.IMU;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveStates.MKE;
import org.firstinspires.ftc.teamcode.dreamcode.States.State;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.physics.Kinematic;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

public class StateEstimator implements Subsystem, State {

    double a, a0, x0, y0, sI, s, sdot;
    Kinematic pos = new Kinematic(), vel = new Kinematic();
    DcMotorEx spinner;
    IMU imu;
    MKE mke;
    double[][] weights = new double[][] {{0, 1},
                                         {0, 1}};


    public StateEstimator(IMU imu, MKE mke) {
        this.imu = imu;
        this.mke = mke;
        a0 = 0;
        x0 = 0;
        y0 = 0;
        this.spinner = null;
        sI = 0;
        s = 0;
    }

    public StateEstimator(IMU imu, MKE mke, DcMotorEx spinner) {
        this.imu = imu;
        this.mke = mke;
        a0 = 0;
        x0 = 0;
        y0 = 0;
        this.spinner = spinner;
        sI = 0;
        s = 0;
    }

    @Override
    public void update(double dt, Telemetry telemetry) {
        imu.update(dt, telemetry);
        mke.update(dt, telemetry);
        pos = Kinematic.dataFusion(new Kinematic[]{imu.getPos(), mke.getPos()}, weights);
        pos = pos.add(new Kinematic(x0, y0));
        a = MathFx.meanDataFusion(new double[]{mke.getA()}, new double[]{1}, a0);
        vel = new Kinematic(mke.getX_dot(), mke.getY_dot());
        telemetry.addData("X: ", pos.X());
        telemetry.addData("Y: ", pos.Y());
        telemetry.addData("A: ", a);

        if (spinner != null) {
            s = MathFx.lowPassFilter(0.75, sI, spinner.getCurrentPosition() * Motors.GoBILDA_223.getDistPerTicks(96/Constants.mmPerInch));
            sdot = (s - sI)/dt;
            sI = s;
        }
    }

    @Override
    public void stop() {

    }

    public void zero() {
        a0 = -a;
        x0 = -pos.X();
        y0 = -pos.Y();
    }

    public double getX() {
        return pos.X();
    }

    public double getY() {
        return pos.Y();
    }

    public double getA() {
        return a;
    }

    public double getX_dot() {
        return vel.X();
    }

    public double getY_dot() {
        return vel.Y();
    }

    public double getS() {
        return s;
    }

    public double getSdot() {
        return sdot;
    }

}

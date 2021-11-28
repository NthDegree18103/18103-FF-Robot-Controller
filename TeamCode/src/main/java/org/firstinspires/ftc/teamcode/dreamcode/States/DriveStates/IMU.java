package org.firstinspires.ftc.teamcode.dreamcode.States.DriveStates;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveState;
import org.firstinspires.ftc.teamcode.lib.physics.Kinematic;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

public class IMU extends DriveState {

    double a;
    boolean collision = false;
    Kinematic pos, vel, acc;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    public IMU(BNO055IMU imu) {
        this.imu = imu;

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity  = imu.getGravity();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        pos = new Kinematic();
        vel = new Kinematic();
        acc = new Kinematic(/*imu.getLinearAcceleration().xAccel, imu.getLinearAcceleration().yAccel*/);
        a = getHeading();
    }

    public IMU(BNO055IMU imu, double x, double y) {
        this.imu = imu;

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity  = imu.getGravity();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        pos = new Kinematic(x, y);
        vel = new Kinematic();
        acc = new Kinematic();
        a = getHeading();
    }

    public IMU(BNO055IMU imu, double x, double y, double a) {
        this.imu = imu;

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity  = imu.getGravity();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        pos = new Kinematic(x, y);
        vel = new Kinematic();
        acc = new Kinematic();
        this.a = a;
    }

    public double getHeading() {
        return angles.firstAngle;
    }

    public double getRoll() {
        return angles.secondAngle;
    }

    public double getPitch() {
        return angles.thirdAngle;
    }

    @Override
    public void update(double dt, Telemetry telemetry) {
        double ax = imu.getLinearAcceleration().xAccel;
        double ay = imu.getLinearAcceleration().yAccel;
        Kinematic curAcc = new Kinematic(
                MathFx.lowPassFilter(.9, acc.X(), ax),
                MathFx.lowPassFilter(.9, acc.Y(), ay));
        curAcc.filter(0.01);
        Kinematic dv = MathFx.integrate(dt, curAcc, acc);
        dv.filter(0.01);
        Kinematic curVel = vel.add(dv);
        pos = pos.add(MathFx.integrate(dt, curVel, vel));
        a = getHeading();
        /*
        telemetry.addData("XA: ", Math.round(curAcc.X() * 100) / 100.0);
        telemetry.addData("YA: ", Math.round(curAcc.Y() * 100) / 100.0);
        telemetry.addData("XV: ", curVel.X());
        telemetry.addData("YV: ", curVel.Y());
        telemetry.addData("Collision: ", getCollision(curAcc, acc));
        telemetry.addData("X: ", getX());
        telemetry.addData("Y: ", getY());
        telemetry.addData("A: ", getA());

         */

        acc = curAcc;
        vel = curVel;
    }

    public boolean getCollision(Kinematic curA, Kinematic prevA) {
        double currentJerkX = curA.X() - prevA.X();
        double currentJerkY = curA.Y() - prevA.Y();

        if ( ( Math.abs(currentJerkX) > Constants.COLLISION_THRESHOLD_DELTA_G ) ||
                ( Math.abs(currentJerkY) > Constants.COLLISION_THRESHOLD_DELTA_G) ) {
            collision = true;
        } else {
            collision = false;
        }

        return collision;
    }

    public double getGravity() {
        return Double.parseDouble(String.valueOf(gravity));
    }

    public double getMag() {
        return Math.sqrt(gravity.xAccel*gravity.xAccel
                + gravity.yAccel*gravity.yAccel
                + gravity.zAccel*gravity.zAccel);
    }

    public double getTemp() {
        return Double.parseDouble(String.valueOf(imu.getTemperature()));
    }

    @Override
    public double getX() {
        return pos.X();
    }

    @Override
    public double getY() {
        return pos.Y();
    }

    @Override
    public double getA() {
        return a;
    }

    @Override
    public Kinematic getPos() {
        return pos;
    }
}

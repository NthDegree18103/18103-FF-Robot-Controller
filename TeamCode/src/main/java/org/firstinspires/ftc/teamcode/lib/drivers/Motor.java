package org.firstinspires.ftc.teamcode.lib.drivers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.control.PIDF;

@Deprecated
public class Motor {

    DcMotorEx motor;
    PIDF pidf;
    Motors model;

    public Motor(Motors model) {
        this.model = model;
    }

    public Motor(Motors model, double kp, double ki, double kd, double ks, double kv, double ka) {
        this.model = model;
        pidf = new PIDF(kp, ki, kd, ks, kv, ka);
    }

    public void init(HardwareMap hMap, String name) {
        motor = hMap.get(DcMotorEx.class, name);
    }

    public void setPidf(PIDF pidf) {
        this.pidf = pidf;
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    public int getCurrentPosition() {
        return getMotor().getCurrentPosition();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void getPower() {
        motor.getPower();
    }

    public double getVelocityTPS() {
        double initPos = motor.getCurrentPosition();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < .01);
        return (motor.getCurrentPosition() - initPos) / (timer.seconds());
    }

    public double getVelocityIPS(double radius) {
        return getVelocityTPS() / model.getTicksPerDist(radius);
    }

    public void setVelocity(double velocity) {

    }

}

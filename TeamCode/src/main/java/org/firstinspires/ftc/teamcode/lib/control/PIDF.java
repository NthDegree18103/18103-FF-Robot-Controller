package org.firstinspires.ftc.teamcode.lib.control;

public class PIDF {

    double kp, ki, kd, ks, kv, ka;
    double prev_error, integral, derivative, prev_time;

    public PIDF(double kp, double ki, double kd, double ks, double kv, double ka) {
        setConstants(kp, ki, kd, ks, kv, ka);
    }

    public double getOutput(double error, double targetVel, double targetAcc, double currentTime) {
        double dt = prev_time - currentTime;
        double kf = ks + kv * (targetVel) + ka * (targetAcc);
        integral += error * dt;
        derivative = (error - prev_error)/dt;
        prev_error = error;
        prev_time = currentTime;
        return (kp * error) + (ki * integral) + (kd * derivative) + kf;
    }

    public void setConstants(double kp, double ki, double kd, double ks, double kv, double ka) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        prev_error = 0;
        integral = 0;
        derivative = 0;
        prev_time = 0;
    }

    public void reset() {
        setConstants(kp, ki, kd, ks, kv, ka);
    }

}

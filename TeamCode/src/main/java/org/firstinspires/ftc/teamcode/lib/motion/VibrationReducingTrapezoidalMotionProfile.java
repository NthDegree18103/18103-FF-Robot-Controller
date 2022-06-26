package org.firstinspires.ftc.teamcode.lib.motion;

import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

/*
 * Author: Akhil G
 */

public class VibrationReducingTrapezoidalMotionProfile extends Profile {
    double[] p, v, a;
    ProfileState[] states;
    double dt, distance, direction, maxV, maxA, fN;
    double t_acc, t_cruise, t_total;
    int timeSteps;

    public VibrationReducingTrapezoidalMotionProfile(double distance, double maxV, double maxA, double fN) {
        this.fN = fN;
        setDistance(distance);
        setKinematics(maxV, maxA);
        setTime();
        setOptimizedKinematics();
        setLists();
        generateProfile();
    }

    public static void main(String[] args) {
        VibrationReducingTrapezoidalMotionProfile profile = new VibrationReducingTrapezoidalMotionProfile(3*24, Motors.GoBILDA_312.getSurfaceVelocity(2), 100d, 80);
        for (int i = 0; i* 0.001 < profile.getT_total(); i++) {
            double timeStamp = i * 0.001;
            System.out.println(/*i + " " + */profile.getVelocity(timeStamp));
        }
    }

    @Override
    public void generateProfile() {
        for (int i = 0; i < timeSteps; i++) {
            double timestamp = i * dt;
            if (timestamp < t_acc) {
                a[i] = maxA;
                v[i] = maxA * timestamp;
                p[i] = maxA * timestamp * timestamp / 2;
                states[i] = ProfileState.Accelerating;
            } else if (timestamp < t_acc + t_cruise) {
                a[i] = 0;
                v[i] = maxV;
                p[i] = (t_acc * maxV / 2 + (timestamp - t_acc) * maxV);
                states[i] = ProfileState.Coasting;
            } else if (timestamp < t_total) {
                double adjustedTime = timestamp - t_acc - t_cruise;
                a[i] = -maxA;
                v[i] = maxV - (maxA * (adjustedTime));
                p[i] = (t_acc * maxV / 2 + t_cruise * maxV + (2 * maxV - maxA * adjustedTime) * adjustedTime / 2);
                states[i] = ProfileState.Decelerating;
            } else {
                a[i] = 0;
                v[i] = 0;
                p[i] = distance;
                states[i] = ProfileState.Stopping;
            }
        }
    }

    @Override
    public double getVelocity(double timeStamp) {
        timeStamp = MathFx.scale(0, timeStamp, timeStamp);
        double timeStep = timeStamp/dt;
        if (timeStep < v.length) {
            double lower = v[(int) timeStep];
            double higher = v[((int) timeStep) + 1];
            return MathFx.linearInterpolation(lower, higher, timeStep);
        }

        return 0d;
    }

    @Override
    public double getAcceleration(double timeStamp) {
        timeStamp = MathFx.scale(0, timeStamp, timeStamp);
        double timeStep = timeStamp/dt;
        if (timeStep < a.length) {
            double lower = a[(int) timeStep];
            double higher = a[((int) timeStep) + 1];
            return MathFx.linearInterpolation(lower, higher, timeStep);
        }

        return 0d;
    }

    @Override
    public double getPosition(double timeStamp) {
        timeStamp = MathFx.scale(0, timeStamp, timeStamp);
        double timeStep = timeStamp/dt;
        if (timeStep < p.length) {
            double lower = p[(int) timeStep];
            double higher = p[((int) timeStep) + 1];
            return MathFx.linearInterpolation(lower, higher, timeStep);
        }

        return distance;
    }

    @Override
    public double getSetPoint() {
        return distance;
    }

    @Override
    public double getDirection() {
        return direction;
    }

    @Override
    public ProfileState getState(double timeStamp) {
        timeStamp = MathFx.scale(0, timeStamp, timeStamp);
        int timeStep = (int)(timeStamp/dt);
        if (timeStep < states.length) {
            return states[timeStep];
        }

        return ProfileState.Stopping;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public void setKinematics(double maxV, double maxA) {
        this.maxV = maxV;
        this.maxA = maxA;
        direction = 1;
        if (distance < 0) {
            direction = -1;
        }
        this.maxV *= direction;
        this.maxA *= direction;
    }

    public void setTime() {
        // TODO Replace w/ universal constant
        dt = 0.001;
        t_acc = maxV/maxA;
        t_cruise = distance/maxV - maxV/maxA;

        if (t_cruise < 0) {
            maxV = Math.sqrt(distance * maxA);
            t_acc = maxV / maxA;
            t_cruise = 0;
        }

        t_total = 2*t_acc + t_cruise;
        timeSteps = (int) Math.ceil(t_total/dt);
        timeSteps = timeSteps + 1;
    }

    public void setLists() {
        p = new double[timeSteps];
        v = new double[timeSteps];
        a = new double[timeSteps];
        states = new ProfileState[timeSteps];
    }

    public void setOptimizedKinematics() {
        t_acc = ((int) (t_acc*fN))/fN;
        maxV = t_acc * maxA;
        t_cruise = (distance - maxV*t_acc)/maxV;

        t_total = 2*t_acc + t_cruise;
        timeSteps = (int) Math.ceil(t_total/dt);
        timeSteps = timeSteps + 1;
    }

    public double getT_total() {
        return t_total;
    }

}
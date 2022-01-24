package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.OldFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;

@Disabled
@Autonomous
public class RedFreightCleaned extends Robot {

    int pathStep = 0;
    ElapsedTime timer = new ElapsedTime();
    double kp = 0.5, kv = 1/Motors.GoBILDA_435.getSurfaceVelocity(2), ka = 0;
    Profile currDriveProfile = null;

    @Override
    public void init() {
        super.init();
        getIo().runServoLeft();
    }

    @Override
    public void start() {
        super.start();
        getIo().runServoLeft();
        timer.reset();
    }

    @Override
    public void loop() {
        super.loop();
        if (pathStep == 0) {
            PIDDrive(33, 1);
        } else if (pathStep == 1) {
            if (timer.seconds() < 1.25) {
                super.getIo().runIntake(0.5);
            } else {
                getIo().runIntake(0);
                getIo().runServoRight();
                pathStep++;
                timer.reset();
            }
        } else if (pathStep == 2) {
            if (timer.seconds() < 1) {
                super.getIo().runIntake(-0.4);
            } else if (timer.seconds() > 1 && timer.seconds() < 2.25) {
                getIo().runServoLeft();
            }
            else {
                getIo().runIntake(0);
                pathStep++;
                timer.reset();
            }
        } else if (pathStep == 3) {
            PIDDrive(-3, 0.5);
        } else if (pathStep == 4) {
            EncoderStrafe(3.2, 1, -0.5);
        } else if (pathStep == 5) {
            if (timer.seconds() < 3.5) {
                super.getSpinner().spin(0.4);
            } else {
                pathStep++;
                timer.reset();
                super.getSpinner().spin(0);
            }
        } else if (pathStep == 6) {
            EncoderTurn(-Math.PI/3.2, -Math.PI/10, 0.5); // Still A Problem
        } else if (pathStep == 7) {
            PIDDrive(85, 1);
        }
        else {
            super.getSpinner().spin(0);
            super.getDrive().setDriveMotors(0);
        }
    }

    public void PIDDrive(double targetState, double tol) {
        if (currDriveProfile == null) {
            currDriveProfile = new TrapezoidalMotionProfile(targetState, Motors.GoBILDA_435.getSurfaceVelocity(2), 100d);
            timer.reset();
        }

        if (Math.abs(targetState - super.getEstimator().getX()) > tol) {
            double Pe = currDriveProfile.getPosition(time) - super.getEstimator().getX();
            double Ve = currDriveProfile.getVelocity(time) - super.getEstimator().getX_dot();
            double Ae = 0;//profile.getAcceleration(time) - super.getEstimator().getY_dDot();
            double u = kp * Pe + kv * Ve + ka * Ae;
            super.getDrive().setDriveMotors(u);
        } else {
            super.getDrive().setDriveMotors(0);
            pathStep++;
            timer.reset();
            currDriveProfile = null;
        }
    }

    public void EncoderTurn(double targetState, double tol, double pow) {
        if (Math.abs(targetState - super.getEstimator().getA()) > tol) {
            super.getDrive().setRotateMotors(pow);
        } else {
            pathStep++;
            super.getDrive().setDriveMotors(0);
        }
    }

    public void EncoderStrafe(double targetState, double tol, double pow) {
        if (Math.abs(super.getEstimator().getY() - targetState) > tol) {
            super.getDrive().setStrafeMotors(pow);
        } else {
            pathStep++;
            super.getDrive().setDriveMotors(0);
        }
    }

}
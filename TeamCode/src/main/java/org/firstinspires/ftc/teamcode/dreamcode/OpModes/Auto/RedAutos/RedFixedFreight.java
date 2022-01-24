package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.RedAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;

@Autonomous
public class RedFixedFreight extends Robot {

    int pathStep = 0;
    ElapsedTime timer = new ElapsedTime();
    double setPoint = 33, tolerance = 1, kp = 0.5, kv = 1/Motors.GoBILDA_435.getSurfaceVelocity(2), ka = 0;
    Profile profile = new TrapezoidalMotionProfile(setPoint, Motors.GoBILDA_435.getSurfaceVelocity(2), 100d);

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
            telemetry.addData("Error", (setPoint - super.getEstimator().getX()));
            if (setPoint - super.getEstimator().getX() > tolerance) {
                double Pe = profile.getPosition(time) - super.getEstimator().getX();
                double Ve = profile.getVelocity(time) - super.getEstimator().getX_dot();
                double Ae = 0;//profile.getAcceleration(time) - super.getEstimator().getY_dDot();
                double u = kp * Pe + kv * Ve + ka * Ae;
                super.getDrive().setDriveMotors(u);
            } else {
                super.getDrive().setDriveMotors(0);
                pathStep++;
                timer.reset();
            }
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
                setPoint = -3;
                tolerance = -.5;
                profile = new TrapezoidalMotionProfile(setPoint, Motors.GoBILDA_435.getSurfaceVelocity(2), 100d);
            }
        } else if (pathStep == 3) {
            telemetry.addData("Error", (setPoint - super.getEstimator().getX()));
            telemetry.addData("PathStep", (pathStep));
            telemetry.update();
            if (setPoint - super.getEstimator().getX() < tolerance) {
                double Pe = profile.getPosition(time) - super.getEstimator().getX();
                double Ve = profile.getVelocity(time) - super.getEstimator().getX_dot();
                double Ae = 0;//profile.getAcceleration(time) - super.getEstimator().getY_dDot();
                double u = kp * Pe + kv * Ve + ka * Ae;
                super.getDrive().setDriveMotors(u);
            } else {
                timer.reset();
                //while (timer.seconds() < .5) {
                //    super.getDrive().setStrafeMotors(-0.3);
                //}
                super.getDrive().setDriveMotors(0);
                //timer.reset();
                pathStep++;
                telemetry.addData("PathStep End", (pathStep));
                telemetry.update();
            }
        } else if (pathStep == 4) {
            if (super.getEstimator().getY() < 3.2) {
                super.getDrive().setStrafeMotors(-0.5);
            } else {
                pathStep++;
                super.getDrive().setDriveMotors(0);
            }
        } else if (pathStep == 5) {
            if (timer.seconds() < 3.5) {
                super.getSpinner().spin(0.4);
            } else {
                pathStep++;
                //telemetry.addData("PathStep", (pathStep));
                //telemetry.update();
                setPoint = 3.5 * 23.5;
                timer.reset();
                profile = new TrapezoidalMotionProfile(setPoint, Motors.GoBILDA_312.getSurfaceVelocity(2), 100d);
                super.getSpinner().spin(0);
            }
        } else if (pathStep == 6) {
            if (super.getEstimator().getA() > -Math.PI/3.2) {
                super.getDrive().setRotateMotors(0.5);
            } else {
                pathStep++;
                super.getDrive().setDriveMotors(0);
                setPoint = 85;
                tolerance = 1;
                profile = new TrapezoidalMotionProfile(setPoint, Motors.GoBILDA_312.getSurfaceVelocity(2), 100d);
            }
        } else if (pathStep == 7) {
            telemetry.addData("Error", (setPoint - super.getEstimator().getX()));
            if (setPoint - super.getEstimator().getX() > tolerance) {
                double Pe = profile.getPosition(time) - super.getEstimator().getX();
                double Ve = profile.getVelocity(time) - super.getEstimator().getX_dot();
                double Ae = 0;//profile.getAcceleration(time) - super.getEstimator().getY_dDot();
                double u = kp * Pe + kv * Ve + ka * Ae;
                super.getDrive().setDriveMotors(u);
            } else {
                super.getDrive().setDriveMotors(0);
                pathStep++;
                timer.reset();
            }
        }
        else {
            super.getSpinner().spin(0);
            super.getDrive().setDriveMotors(0);
        }
    }

    /*public void followPath(Profile profile) {
        if (currDriveProfile == null) {
            currDriveProfile = profile;
            targetPos = getEstimator().getY() + profile.getSetPoint();
        }
        driveTime += getDt();
        if (getDrive().iterativeProfiledDrive(targetPos, tolerance, getEstimator(),
                currDriveProfile, driveTime, kp, kv, ka)) {
            currDriveProfile = null;
            driveTime = 0;
            pathStep++;
        }
    }*/

}
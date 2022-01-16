package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;

@Autonomous
public class StraightCarousel extends Robot {

    int pathStep = 0;
    ElapsedTime timer = new ElapsedTime();
    double setPoint = -20, tolerance = 0, kp = 0.5, kv = 1/Motors.GoBILDA_435.getSurfaceVelocity(2), ka = 0;
    Profile profile = new TrapezoidalMotionProfile(setPoint, Motors.GoBILDA_435.getSurfaceVelocity(2), 100d);

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
        timer.reset();
    }

    @Override
    public void loop() {
        super.loop();
        if (pathStep == 0) {
            if (timer.seconds() < 0.5) {
                super.getIo().runUpIntake();
            } else {
                getIo().runIntake(0);
                pathStep++;
                timer.reset();
            }
        } else if (pathStep == 1) {
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
                super.getDrive().setDriveMotors(0);
                pathStep++;
                timer.reset();
                telemetry.addData("PathStep End", (pathStep));
                telemetry.update();
            }
        } else if (pathStep == 2) {
            if (timer.seconds() < 3) {
                super.getSpinner().spin(0.4);
            } else {
                pathStep++;
                //telemetry.addData("PathStep", (pathStep));
                //telemetry.update();
                setPoint = 3.5 * 23.5;
                profile = new TrapezoidalMotionProfile(setPoint, Motors.GoBILDA_312.getSurfaceVelocity(2), 100d);
                super.getSpinner().spin(0);
            }
        } else if (pathStep == 3) {
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
                //telemetry.addData("PathStep End", (pathStep));
                //telemetry.update();
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
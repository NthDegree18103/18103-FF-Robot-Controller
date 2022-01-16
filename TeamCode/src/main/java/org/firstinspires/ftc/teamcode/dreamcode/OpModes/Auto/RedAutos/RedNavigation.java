package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.RedAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;

@Autonomous
public class RedNavigation extends Robot {

    int pathStep = 0;
    ElapsedTime timer = new ElapsedTime();
    double setPoint = 23.5*2, tolerance = 1, kp = 0.5, kv = 1/Motors.GoBILDA_435.getSurfaceVelocity(2), ka = 0;
    Profile profile = new TrapezoidalMotionProfile(setPoint, Motors.GoBILDA_435.getSurfaceVelocity(2), 100d);

    @Override
    public void init() {
        super.init();
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
                setPoint = -23.5;
                pathStep++;
                timer.reset();
            }
        } else if (pathStep == 1) {
            if (setPoint + super.getEstimator().getY() < tolerance) {
                double Pe = setPoint - super.getEstimator().getY();
                double u = kp * Pe;
                super.getDrive().setStrafeMotors(u);
            } else {
                super.getDrive().setDriveMotors(0);
                //setPoint = 23.5;
                pathStep++;
                timer.reset();
            }
        } else {
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
package org.firstinspires.ftc.teamcode.teamNum.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.teamNum.Constants;
import org.firstinspires.ftc.teamcode.teamNum.Robot;

@Autonomous
public class DuckingWork extends Robot {

    ElapsedTime timer = new ElapsedTime();
    int pathStep = 0;
    double setPoint = -38/Constants.drivePIDMagicNumber, tolerance = 0, kp = 1, kv = 1/Motors.GoBILDA_312.getSurfaceVelocity(2), ka = .01;
    Profile profile = new TrapezoidalMotionProfile(setPoint, Motors.GoBILDA_312.getSurfaceVelocity(2), 100d);

    @Override
    public void init() {
        super.init();
        timer.reset();
    }

    @Override
    public void loop() {
        super.loop();
        switch (pathStep) {
            case 0:
                telemetry.addData("Error", (setPoint + super.getEstimator().getY()));
                if (setPoint + super.getEstimator().getY() < tolerance) {
                    double Pe = profile.getPosition(time) + super.getEstimator().getY();
                    double Ve = profile.getVelocity(time) + super.getEstimator().getY_dot();
                    double Ae = profile.getAcceleration(time) + super.getEstimator().getY_dDot();
                    double u = kp * Pe + kv * Ve + ka * Ae;
                    super.getDrive().setDriveMotors(u);
                } else {
                    super.getDrive().setDriveMotors(0);
                    pathStep += 4;
                }
            case 1:
                timer.reset();
                super.getSpinner().setSpin(0.5);
                if (timer.seconds() > 5) {
                    pathStep++;
                }
            case 2:
                setPoint = 108/Constants.drivePIDMagicNumber;
                profile = new TrapezoidalMotionProfile(setPoint, Motors.GoBILDA_312.getSurfaceVelocity(2), 100d);
                telemetry.addData("Error", Math.abs(setPoint - super.getEstimator().getY()));
                if (setPoint - super.getEstimator().getY() > tolerance) {
                    double Pe = profile.getPosition(time) - super.getEstimator().getY();
                    double Ve = profile.getVelocity(time) - super.getEstimator().getY_dot();
                    double Ae = profile.getAcceleration(time) - super.getEstimator().getY_dDot();
                    double u = kp * Pe + kv * Ve + ka * Ae;
                    super.getDrive().setDriveMotors(u);
                } else {
                    super.getDrive().setDriveMotors(0);
                    pathStep++;
                }
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

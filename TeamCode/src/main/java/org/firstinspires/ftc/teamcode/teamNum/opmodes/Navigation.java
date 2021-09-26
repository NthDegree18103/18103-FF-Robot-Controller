package org.firstinspires.ftc.teamcode.teamNum.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.teamNum.Robot;

@Autonomous
public class Navigation extends Robot {

    int pathStep = 0;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        switch (pathStep) {
            case 0:
                if(followPath(new TrapezoidalMotionProfile(48,
                        Motors.GoBILDA_312.getSurfaceVelocity(2), 100d))) {
                    pathStep++;
                }
            case 1:
                /*if (currDriveProfile == null) {
                    currDriveProfile = new TrapezoidalMotionProfile(48, MotorModel.GoBILDA_312.getSurfaceVelocity(2), 100d);
                    targetPos = getEstimator().getY() + 48;
                }
                if (Math.abs(targetPos - getEstimator().getY()) > tolerance) {
                    driveTime += getDt();
                    double Pe = currDriveProfile.getPosition(driveTime) - getEstimator().getY();
                    double Ve = currDriveProfile.getVelocity(driveTime) - getEstimator().getY_dot();
                    double Ae = currDriveProfile.getAcceleration(driveTime) - getEstimator().getY_dDot();
                    double u = kp * Pe + kv * Ve + ka * Ae;
                    getDrive().setDriveMotors(u);
                }
                getDrive().setDriveMotors(0);
                 */
                if(followPath(new TrapezoidalMotionProfile(48,
                        Motors.GoBILDA_312.getSurfaceVelocity(2), 100d))) {
                    pathStep++;
                }
            case 3:
                if(followPath(new TrapezoidalMotionProfile(-48*2,
                        Motors.GoBILDA_312.getSurfaceVelocity(2), 100d))) {
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

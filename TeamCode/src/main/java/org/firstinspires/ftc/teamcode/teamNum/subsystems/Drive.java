package org.firstinspires.ftc.teamcode.teamNum.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.teamNum.Constants;
import org.firstinspires.ftc.teamcode.teamNum.states.DriveMode;

import java.util.Arrays;

public class Drive implements Subsystem {

    DcMotorEx fl, fr, bl, br;
    DcMotorEx[] driveMotors;

    public Drive(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;

        driveMotors = new DcMotorEx[]{fl, fr, bl, br};
    }

    @Override
    public void update(double dt) {

    }

    /**
     * Sets Drive to go forward/backwards
     * @param power Speed of movement
     */
    public void setDriveMotors(double power) {
        for (DcMotorEx i : driveMotors) {
            i.setPower(power);
        }
    }

    /**
     * Sets Drive to go left/right
     * @param power Speed of movement
     */
    public void setStrafeMotors(double power) {
        fl.setPower(power);
        bl.setPower(-power);
        fr.setPower(-power);
        br.setPower(power);
    }

    /**
     * Sets Drive to rotate
     * @param power Speed of Movement
     */
    public void setRotateMotors(double power) {
        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(-power);
        br.setPower(-power);
    }

    /**
     * Sets Drive to go forward/backwards w/ motion profile & PIDSVA
     * @param distance target setpoint
     * @param tolerance lowest acceptable final error
     * @param robot State Estimator used for feedforward/feedback loop
     */
    public void motionProfileDrive(double distance, double tolerance, StateEstimator robot) {
        TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(distance, Motors.GoBILDA_312.getSurfaceVelocity(2), 100d);
        double targetPos = fr.getCurrentPosition() * Motors.GoBILDA_312.getDistPerTicks(2) + distance;
        double kp = 0;
        double kv = 0;
        double ka = 0;
        ElapsedTime timer = new ElapsedTime();
        double time = 0;
        while (Math.abs(targetPos - robot.getY()) > tolerance) {
            double timeStamp = timer.seconds();
            double Pe = motionProfile.getPosition(timeStamp) - robot.getY();
            double Ve = motionProfile.getVelocity(timeStamp) - robot.getY_dot();
            double Ae = motionProfile.getAcceleration(timeStamp) - robot.getY_dDot();
            double u = kp * Pe + kv * Ve + ka * Ae;
            setDriveMotors(u);
        }
        setDriveMotors(0);
    }

    public boolean iterativeProfiledDrive(double setPoint, double tolerance, StateEstimator robot,
                                       Profile profile, double time, double kp, double kv, double ka) {
        if (Math.abs(setPoint - robot.getY()) > tolerance) {
            double Pe = profile.getPosition(time) - robot.getY();
            double Ve = profile.getVelocity(time) - robot.getY_dot();
            double Ae = profile.getAcceleration(time) - robot.getY_dDot();
            double u = kp * Pe + kv * Ve + ka * Ae;
            setDriveMotors(u);
            return false;
        } else {
            setDriveMotors(0);
            return true;
        }
    }

    public boolean iterativeProfiledStrafe(double setPoint, double tolerance, StateEstimator robot,
                                          Profile profile, double time, double kp, double kv, double ka) {
        if (Math.abs(setPoint - robot.getX()) > tolerance) {
            double Pe = profile.getPosition(time) - robot.getX();
            double Ve = profile.getVelocity(time) - robot.getX_dot();
            double Ae = profile.getAcceleration(time) - robot.getX_dDot();
            double u = kp * Pe + kv * Ve + ka * Ae;
            setStrafeMotors(u);
            return false;
        } else {
            setDriveMotors(0);
            return true;
        }
    }

    /**
     * Mecanum Drive Control
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Left/Right (Strafe) Force (GamePad Left Stick x)
     * @param turn Rotational Force (GamePad Right Stick x)
     * @param mode Drivetrain Speed Setting (Sport, Normal, Economy)
     */
    public void POVMecanumDrive(double y, double x, double turn, DriveMode mode) {
        turn *= 0.5; //Custom reduction bc it was requested.
        double v1 = -(y - (turn * Constants.strafeScaling) - (x/Constants.turnScaling));
        double v2 = -(y - (turn * Constants.strafeScaling) + (x/Constants.turnScaling));
        double v3 = -(y + (turn * Constants.strafeScaling) - (x/Constants.turnScaling));
        double v4 = -(y + (turn * Constants.strafeScaling) + (x/Constants.turnScaling));

        Double[] v = new Double[]{Math.abs(v1), Math.abs(v2), Math.abs(v3), Math.abs(v4)};
        Arrays.sort(v);
        if (v[3] > 1) {
            v1 /= v[3];
            v2 /= v[3];
            v3 /= v[3];
            v4 /= v[3];
        }

        fl.setPower(v1 * mode.getScaling());
        bl.setPower(v2 * mode.getScaling());
        br.setPower(v3 * mode.getScaling());
        fr.setPower(v4 * mode.getScaling());
    }
    
}

package org.firstinspires.ftc.teamcode.dreamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;

import java.util.Arrays;

public class Drive implements Subsystem {

    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    DcMotorEx[] driveMotors;
    double kp = 0.5, kv = 1/Motors.GoBILDA_435.getSurfaceVelocity(2), ka = 0;

    public Drive(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;

        driveMotors = new DcMotorEx[]{fl, fr, bl, br};
    }

    @Override
    public void update(double dt, Telemetry telemetry) {

    }

    @Override
    public void stop() {
        setDriveMotors(0);
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
     * Mecanum Drive Control
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Left/Right (Strafe) Force (GamePad Left Stick x)
     * @param turn Rotational Force (GamePad Right Stick x)
     * @param mode Drivetrain Speed Setting (Sport, Normal, Economy)
     */
    public void POVMecanumDrive(double y, double x, double turn, DriveMode mode) {
        double v1 = -(y - (turn * Constants.strafeScaling) - (x/Constants.turnScaling));
        double v2 = -(y - (turn * Constants.strafeScaling) + (x/Constants.turnScaling));
        double v3 = -(y + (turn * Constants.strafeScaling) - (x/Constants.turnScaling));
        double v4 = -(y + (turn * Constants.strafeScaling) + (x/Constants.turnScaling));

        double v = Math.max(Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.abs(v3)), Math.abs(v4));
        if (v > 1) {
            v1 /= v;
            v2 /= v;
            v3 /= v;
            v4 /= v;
        }

        fl.setPower(v1 * mode.getScaling());
        bl.setPower(v2 * mode.getScaling());
        br.setPower(v3 * mode.getScaling());
        fr.setPower(v4 * mode.getScaling());
    }


    public boolean pathFollower(StateEstimator robot, Profile profile, double tolerance, double time) {
        double setPoint = profile.getSetPoint();
        if (setPoint - robot.getX() > tolerance) {
            double Pe = profile.getPosition(time) - robot.getX();
            double Ve = profile.getVelocity(time) - robot.getX_dot();
            double Ae = 0;//profile.getAcceleration(time) - super.getEstimator().getY_dDot();
            double u = kp * Pe + kv * Ve + ka * Ae;
            setDriveMotors(u);
            return false;
        } else {
            stop();
            return true;
        }
    }
}

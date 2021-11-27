package org.firstinspires.ftc.teamcode.dreamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;

import java.util.Arrays;

public class Drive implements Subsystem {

    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    DcMotorEx[] driveMotors;

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

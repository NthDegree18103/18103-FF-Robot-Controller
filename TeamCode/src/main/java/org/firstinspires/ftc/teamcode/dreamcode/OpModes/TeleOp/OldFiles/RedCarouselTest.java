package org.firstinspires.ftc.teamcode.dreamcode.OpModes.TeleOp.OldFiles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;

@Disabled
@TeleOp
public class RedCarouselTest extends Robot {

    ElapsedTime timer = new ElapsedTime();
    double setPoint = 12.5*2*Math.PI, tolerance = 1, kp = 0.5, kv = 1/ Motors.GoBILDA_223.getSurfaceVelocity(96/Constants.mmPerInch), ka = 0;
    Profile profile = new TrapezoidalMotionProfile(setPoint, Motors.GoBILDA_223.getSurfaceVelocity(2/Constants.mmPerInch)/2, 100d);

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        super.getDrive().POVMecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, DriveMode.Sport);
        //super.getSpinner().negCapSpin(-gamepad1.right_trigger);
        super.getIo().runIntake(gamepad1.right_stick_y);
        //start intake rts
        if (gamepad1.left_bumper){
            super.getIo().runDownIntake();
        }
        else if (gamepad1.right_bumper){
            super.getIo().runUpIntake();
        }
        //start servo
        if (gamepad1.y) {
            super.getIo().runServoLeft();
        }
        else if (gamepad1.a) {
            super.getIo().runServoMid();
        }
        else if (gamepad1.b) {
            super.getIo().runServoRight();
        }
        //spinner
        if (gamepad1.x) {
            spinnerState = true;
            timer.reset();
        }

        if (spinnerState) {
            if (setPoint - super.getEstimator().getS() < tolerance) {
                double Pe = profile.getPosition(time) - super.getEstimator().getS();
                double Ve = profile.getVelocity(time) - super.getEstimator().getSdot();
                double Ae = 0;
                double u = kp * Pe + kv * Ve + ka * Ae;
                super.getSpinner().capSpin(u);
            } else {
                super.getSpinner().capSpin(0);
                spinnerState = false;
                setPoint = 12.5*2*Math.PI;
                timer.reset();
            }
        }
    }
}

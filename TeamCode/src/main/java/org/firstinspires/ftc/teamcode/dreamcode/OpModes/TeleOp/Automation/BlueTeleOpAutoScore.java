package org.firstinspires.ftc.teamcode.dreamcode.OpModes.TeleOp.Automation;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;

@TeleOp
public class BlueTeleOpAutoScore extends Robot {
    int scoringState = 0;
    double startPoint = 0, startA = 0, startY = 0;
    ElapsedTime timer= new ElapsedTime();
    double kp = 0.5, kv = 1/Motors.GoBILDA_435.getSurfaceVelocity(2), ka = 0;
    Profile FirstDriveProfile = new TrapezoidalMotionProfile(-48, Motors.GoBILDA_435.getSurfaceVelocity(2), 100d);

    @Override
    public void init() {
        super.init();
        timer.reset();
    }

    @Override
    public void loop() {
        super.loop();
        super.getDrive().POVMecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, DriveMode.Sport);
        super.getSpinner().negCapSpin(gamepad1.left_trigger);
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

        if (gamepad1.x) {
            scoringState = 1;
            startPoint = super.getEstimator().getX();
        }

        if (gamepad1.dpad_left) {
            scoringState = 5;
        }

        if (scoringState == 1) {
            PIDDrive(FirstDriveProfile, startPoint - 48, 1);
        } else if (scoringState == 2) {
            EncoderTurn(-Math.PI/1.8 + startA, 0.5);
        } else if (scoringState == 3) {
            EncoderYDrive(16 + startY, 1, 0.5);
        } else if (scoringState == 4) {
            if (timer.seconds() < 1.25) {
                super.getIo().runIntake(0.5);
            } else {
                getIo().runIntake(0);
                getIo().runServoRight();
                scoringState++;
                timer.reset();
            }
        }


    }

    public void PIDDrive(Profile profile, double targetState, double tol) {
        if (Math.abs(targetState - super.getEstimator().getX()) > tol) {
            double Pe = profile.getPosition(time) - (super.getEstimator().getX() - startPoint);
            double Ve = profile.getVelocity(time) - super.getEstimator().getX_dot();
            double Ae = 0;//profile.getAcceleration(time) - super.getEstimator().getY_dDot();
            double u = kp * Pe + kv * Ve + ka * Ae;
            super.getDrive().setDriveMotors(u);
        } else {
            super.getDrive().setDriveMotors(0);
            scoringState++;
            timer.reset();
            startA = super.getEstimator().getA();
            startPoint = super.getEstimator().getX();
            startY = super.getEstimator().getY();
        }
    }

    public void EncoderTurn(double targetState, double pow) {
        if (super.getEstimator().getA() > targetState) {
            super.getDrive().setRotateMotors(pow);
        } else {
            scoringState++;
            super.getDrive().setDriveMotors(0);
            startA = super.getEstimator().getA();
            startPoint = super.getEstimator().getX();
            startY = super.getEstimator().getY();
        }
    }

    public void EncoderYDrive(double targetState, double tol, double pow) {
        if (Math.abs(super.getEstimator().getY() - targetState) > tol) {
            super.getDrive().setDriveMotors(pow);
        } else {
            scoringState++;
            super.getDrive().setDriveMotors(0);
            startA = super.getEstimator().getA();
            startPoint = super.getEstimator().getX();
            timer.reset();
        }
    }

}

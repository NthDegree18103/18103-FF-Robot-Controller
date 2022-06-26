package org.firstinspires.ftc.teamcode.dreamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeOuttake implements Subsystem {

    DcMotorEx intake;
    Servo servoTest;
    private int clawState = 0;

    public IntakeOuttake(DcMotorEx intake, Servo servoTest) {
        this.intake = intake;
        this.servoTest = servoTest;
    }

    @Override
    public void update(double dt, Telemetry telemetry) {
        //telemetry.addData("Intake", intake.getPower());
        //telemetry.addData("servo position", servoTest.getPosition());
    }

    @Override
    public void stop() {
        runIntake(0);
    }

    public void runIntake(double pow) {
        intake.setPower(pow);
    }

    public void runUpIntake() {intake.setPower(0.75);} //rts

    public void runDownIntake() {intake.setPower(-0.75);} //rts

    public void runServoRight() {
        servoTest.setPosition(0.8);
    } // tyler

    public void runServoMid() {
        servoTest.setPosition(0.9);
    }

    public void runServoLeft() {
        servoTest.setPosition(1);
    }

    public void clawStateManager(boolean left, boolean right) {
        if (left || right) {
            if (left) {
                clawState = Math.max(0, clawState - 1);
            }
            if (right) {
                clawState = Math.min(2, clawState + 1);
            }
            if (clawState == 0) {
                runServoLeft();
            } else if (clawState == 2) {
                runServoRight();
            } else {
                runServoMid();
            }
        }
    }

    public void armStateManager(boolean down, boolean up) {
        if (down || up) {
            if (down) {
                runDownIntake();
            } else {
                runUpIntake();
            }
        }
        else {
            runIntake(0);
        }
    }

}

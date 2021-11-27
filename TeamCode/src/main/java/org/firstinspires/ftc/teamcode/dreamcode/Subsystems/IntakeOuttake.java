package org.firstinspires.ftc.teamcode.dreamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeOuttake implements Subsystem {

    DcMotorEx intake;
    Servo servoTest;

    public IntakeOuttake(DcMotorEx intake, Servo servoTest) {
        this.intake = intake;
        this.servoTest = servoTest;
    }

    @Override
    public void update(double dt, Telemetry telemetry) {
        //telemetry.addData("Intake", intake.getPower());
        //telemetry.addData("servo position", servoTest.getPosition());
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

}

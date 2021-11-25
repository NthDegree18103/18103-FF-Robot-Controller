package org.firstinspires.ftc.teamcode.teamNum.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeOuttake implements Subsystem {


    DcMotorEx intake;
    Servo servoTest;

    public IntakeOuttake(DcMotorEx intake) {
        this.intake = intake;
    }

    @Override
    public void update(double dt, Telemetry telemetry) {
        telemetry.addData("Intake", intake.getPower());
    }

    public void runIntake(double pow) {
        intake.setPower(pow);
    }

    public void runServoRight() {
        servoTest.setPosition(0);
    } // tyler

    public void runServoMid() {
        servoTest.setPosition(0.5);
    }

    public void runServoLeft() {
        servoTest.setPosition(1);
    }

}

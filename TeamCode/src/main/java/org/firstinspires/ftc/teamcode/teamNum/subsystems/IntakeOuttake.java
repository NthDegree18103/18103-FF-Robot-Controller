package org.firstinspires.ftc.teamcode.teamNum.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeOuttake implements Subsystem {


    DcMotorEx intake;

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

}

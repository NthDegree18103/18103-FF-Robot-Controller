package org.firstinspires.ftc.teamcode.dreamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spinner implements Subsystem {

    DcMotorEx spinner;

    public Spinner(DcMotorEx spinner) {
        this.spinner = spinner;
    }

    @Override
    public void update(double dt, Telemetry telemetry) {
        //telemetry.addData("Spinner", spinner.getPower());
    }

    public void spin(double pow) {
        spinner.setPower(pow);
    }

    public void capSpin(double pow) {
        spinner.setPower(Math.min(pow, 0.5));
    }

}

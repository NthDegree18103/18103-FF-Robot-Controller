package org.firstinspires.ftc.teamcode.teamNum.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spinner implements Subsystem {

    DcMotorEx spinner;

    public Spinner(DcMotorEx spinner) {
        this.spinner = spinner;
    }

    @Override
    public void update(double dt, Telemetry telemetry) {
        telemetry.addData("Spinner", spinner.getPower());
    }

    public void spin(double pow) {
        spinner.setPower(pow);
    }

}

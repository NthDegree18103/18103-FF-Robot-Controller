package org.firstinspires.ftc.teamcode.teamNum.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Spinner implements Subsystem {

    DcMotorEx spinner;

    public Spinner(DcMotorEx spinner) {
        this.spinner = spinner;
    }

    @Override
    public void update(double dt) {

    }

    public void spin(double pow) {
        spinner.setPower(pow);
    }

}

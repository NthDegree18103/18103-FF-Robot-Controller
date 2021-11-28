package org.firstinspires.ftc.teamcode.dreamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;

@TeleOp
public class TeleOp2 extends Robot {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        super.getDrive().POVMecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, DriveMode.Sport);

        super.getSpinner().capSpin(gamepad1.left_trigger);

        super.getIo().armStateManager(gamepad1.dpad_down, gamepad1.dpad_up);

        super.getIo().clawStateManager(gamepad1.left_bumper, gamepad1.right_bumper);

    }
}

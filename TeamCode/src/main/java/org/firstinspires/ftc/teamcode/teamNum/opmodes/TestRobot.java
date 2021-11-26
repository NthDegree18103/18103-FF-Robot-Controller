package org.firstinspires.ftc.teamcode.teamNum.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamNum.Robot;
import org.firstinspires.ftc.teamcode.teamNum.states.DriveMode;

@TeleOp
public class TestRobot extends Robot {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        super.getDrive().POVMecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, DriveMode.Sport);
        super.getSpinner().setSpin(gamepad1.left_trigger);
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


    }
}

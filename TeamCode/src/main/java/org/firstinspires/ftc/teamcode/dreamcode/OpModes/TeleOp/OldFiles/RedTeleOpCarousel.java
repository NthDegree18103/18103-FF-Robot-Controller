package org.firstinspires.ftc.teamcode.dreamcode.OpModes.TeleOp.OldFiles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;

@Disabled
@TeleOp
public class RedTeleOpCarousel extends Robot {
    ElapsedTime timer = new ElapsedTime();

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
        super.getSpinner().negCapSpin(-gamepad1.right_trigger);
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
            timer.reset();
            while (timer.seconds() < 3) {
                super.getSpinner().negCapSpin(-0.5);
            }
        }

    }
}

package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Abstraction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;

@Autonomous
public class StraightNav2 extends AutoTemplate {

    Profile p1 = new TrapezoidalMotionProfile(24);
    Profile p2 = new TrapezoidalMotionProfile(-24);


    @Override
    public void loop() {
        super.loop();
        if (pathStep == 0) {
            runDrive(p1, 1);
        } else if (pathStep == 1) {
            if (timer.seconds() < 2) {
                super.getSpinner().spin(0.5);
            } else {
                super.stopRobot();
                timer.reset();
                pathStep++;
            }
            //runDrive(p2, 1);
        }
        else {
            super.stopRobot();
        }
    }

}

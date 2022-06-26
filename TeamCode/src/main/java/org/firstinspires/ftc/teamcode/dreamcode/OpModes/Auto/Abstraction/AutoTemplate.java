package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Abstraction;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;

public class AutoTemplate extends Robot {

    int pathStep = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
    }

    public void runDrive(Profile profile, double tolerance) {
        if (super.getDrive().pathFollower(super.getEstimator(), profile, tolerance, timer.time())) {
            pathStep++;
            timer.reset();
        }
    }
}
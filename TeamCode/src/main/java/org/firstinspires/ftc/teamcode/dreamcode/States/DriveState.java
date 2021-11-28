package org.firstinspires.ftc.teamcode.dreamcode.States;

import org.firstinspires.ftc.teamcode.lib.physics.Kinematic;

public abstract class DriveState implements State {

    public abstract double getX();
    public abstract double getY();
    public abstract double getA();
    public abstract Kinematic getPos();
    //public abstract double getX_dot();
    //public abstract double getY_dot();

}

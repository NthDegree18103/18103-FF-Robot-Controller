package org.firstinspires.ftc.teamcode.teamNum.states;

public abstract class DriveState implements State {

    public abstract double getX();
    public abstract double getY();
    public abstract double getTheta();
    public abstract double getX_dot();
    public abstract double getY_dot();

}

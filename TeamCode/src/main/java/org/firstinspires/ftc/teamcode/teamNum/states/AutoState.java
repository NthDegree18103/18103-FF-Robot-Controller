package org.firstinspires.ftc.teamcode.teamNum.states;

public enum AutoState {
    None(36, -45),
    One(60, 45),
    Four(84, -45);

    private final double dist;
    private final double angle;

    AutoState(double dist, double angle) {
        this.dist = dist;
        this.angle = angle;
    }

    public double getDist() {
        return dist;
    }

    public double getAngle() {return angle;}

}

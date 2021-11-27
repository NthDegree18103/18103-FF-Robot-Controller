package org.firstinspires.ftc.teamcode.dreamcode.States;

public enum DriveMode {

    Sport(1, 1, "Sport"),
    Optimized(0.5,  0, "Balanced"),
    Economy(0.25, -1, "Economy");

    private final double scaling;
    private final double id;
    private final String name;

    DriveMode(double scaling, double id, String name) {
        this.scaling = scaling;
        this.id = id;
        this.name = name;
    }

    public double getScaling() {
        return scaling;
    }

    public double getId() {
        return id;
    }

    public String getName() {
        return name;
    }

}

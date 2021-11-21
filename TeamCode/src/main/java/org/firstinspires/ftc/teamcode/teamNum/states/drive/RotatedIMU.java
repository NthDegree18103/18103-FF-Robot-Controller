package org.firstinspires.ftc.teamcode.teamNum.states.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.teamNum.Constants;
import org.firstinspires.ftc.teamcode.teamNum.states.DriveState;

public class RotatedIMU extends IMU {


    public RotatedIMU(BNO055IMU imu, double x, double y, double theta) {
        super(imu, x, y, theta);
    }

    public RotatedIMU(BNO055IMU imu) {
        super(imu);
    }

    public double getRoll() {
        return angles.firstAngle;
    }

    public double getHeading() {
        return angles.secondAngle;
    }

    public double getPitch() {
        return angles.thirdAngle;
    }

    public boolean getCollision() {
        boolean collision = false;

        double curr_world_linear_accel_x = imu.getLinearAcceleration().xAccel;
        double currentJerkX = curr_world_linear_accel_x - lastAccelX;
        lastAccelX = curr_world_linear_accel_x;
        double curr_world_linear_accel_z = imu.getLinearAcceleration().zAccel;
        double currentJerkZ = curr_world_linear_accel_z - lastAccelY;
        lastAccelY = curr_world_linear_accel_z;


        if ( ( Math.abs(currentJerkX) > Constants.COLLISION_THRESHOLD_DELTA_G ) ||
                ( Math.abs(currentJerkZ) > Constants.COLLISION_THRESHOLD_DELTA_G) ) {
            collision = true;
        }

        return collision;
    }

    public void integrate(double dt) {
        currAccelX = MathFx.lowPassFilter(0.8, lastAccelX, imu.getLinearAcceleration().xAccel);
        currAccelY = MathFx.lowPassFilter(0.8, lastAccelY, imu.getLinearAcceleration().zAccel);

        double meanAccelX = (currAccelX + lastAccelX)/2;
        double meanAccelY = (currAccelY + lastAccelY)/2;

        x_dot += meanAccelX * dt / 2;
        y_dot += meanAccelY * dt / 2;

        x += x_dot*dt;
        y += y_dot*dt;

        x_dot += meanAccelX * dt / 2;
        y_dot += meanAccelY * dt / 2;

        lastAccelX = currAccelX;
        lastAccelY = currAccelY;
    }

}

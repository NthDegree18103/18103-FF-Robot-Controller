package org.firstinspires.ftc.teamcode.teamNum.states.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

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

public class IMU extends DriveState {

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    private double lastAccelX = 0.0;
    private double lastAccelY = 0.0;
    private double currAccelX = 0.0;
    private double currAccelY = 0.0;
    private double x, y, x_dot, y_dot, theta;

    public IMU(BNO055IMU imu) {
        this.imu = imu;

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        IMUParameters.temperatureUnit     = BNO055IMU.TempUnit.FARENHEIT;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUParameters.loggingEnabled      = true;
        IMUParameters.loggingTag          = "IMU";
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //NaiveAccelerationIntegrator();

        imu.initialize(IMUParameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity  = imu.getGravity();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        x = 0;
        y = 0;
        x_dot = 0;
        y_dot = 0;
    }

    public IMU(BNO055IMU imu, double x, double y, double theta) {
        this.imu = imu;

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        IMUParameters.temperatureUnit     = BNO055IMU.TempUnit.FARENHEIT;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUParameters.loggingEnabled      = true;
        IMUParameters.loggingTag          = "IMU";
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //NaiveAccelerationIntegrator();

        imu.initialize(IMUParameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity  = imu.getGravity();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        this.x = x;
        this.y = y;
        this.theta = theta;
        x_dot = 0;
        y_dot = 0;
    }

    public double getHeading() {
        return angles.firstAngle;
    }

    public double getRoll() {
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
        double curr_world_linear_accel_y = imu.getLinearAcceleration().yAccel;
        double currentJerkY = curr_world_linear_accel_y - lastAccelY;
        lastAccelY = curr_world_linear_accel_y;


        if ( ( Math.abs(currentJerkX) > Constants.COLLISION_THRESHOLD_DELTA_G ) ||
                ( Math.abs(currentJerkY) > Constants.COLLISION_THRESHOLD_DELTA_G) ) {
            collision = true;
        }

        return collision;
    }

    public double getGravity() {
        return Double.parseDouble(String.valueOf(gravity));
    }

    public double getMag() {
        return Math.sqrt(gravity.xAccel*gravity.xAccel
                + gravity.yAccel*gravity.yAccel
                + gravity.zAccel*gravity.zAccel);
    }

    public double getTemp() {
        return Double.parseDouble(String.valueOf(imu.getTemperature()));
    }

    public void integrate(double dt) {
        currAccelX = MathFx.lowPassFilter(0.8, lastAccelX, imu.getLinearAcceleration().xAccel);
        currAccelY = MathFx.lowPassFilter(0.8, lastAccelY, imu.getLinearAcceleration().yAccel);

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

    @Override
    public void update(double dt) {
        integrate(dt);
    }


    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public double getTheta() {
        theta = MathFx.lowPassFilter(.8, theta, getHeading() + theta);
        return theta;
    }

    public double getX_dot() {
        return x_dot;
    }

    public double getY_dot() {
        return y_dot;
    }

    public double getX_dDot() {return currAccelX;}

    public double getY_dDot() {return currAccelY;}

}

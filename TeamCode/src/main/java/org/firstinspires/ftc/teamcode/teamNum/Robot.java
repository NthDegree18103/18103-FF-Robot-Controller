package org.firstinspires.ftc.teamcode.teamNum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.teamNum.states.drive.IMU;
import org.firstinspires.ftc.teamcode.teamNum.states.drive.MKE;
import org.firstinspires.ftc.teamcode.teamNum.states.drive.VV;
import org.firstinspires.ftc.teamcode.teamNum.subsystems.Drive;
import org.firstinspires.ftc.teamcode.teamNum.subsystems.Spinner;
import org.firstinspires.ftc.teamcode.teamNum.subsystems.StateEstimator;
import org.firstinspires.ftc.teamcode.teamNum.subsystems.Subsystem;

public class Robot extends OpMode {

    double dt = 0.01;
    ElapsedTime timer = new ElapsedTime();

    Subsystem[] subsystems;

    DcMotorEx fl, fr, bl, br, spin;
    DcMotorEx[] driveMotors;
    Drive drive;
    Spinner spinner;
    Profile currDriveProfile;
    double targetPos = 0;
    double tolerance = 2.5;
    double kp = 0;
    double kv = 0;
    double ka = 0;
    double driveTime = 0;

    StateEstimator estimator;

    @Override
    public void init() {
        initDrive();
        initSpinner();
        //initStateEstimator();
        subsystems = new Subsystem[]{/*estimator,*/ drive, spinner};
    }

    @Override
    public void loop() {
        dt = timer.seconds();
        timer.reset();
        for (Subsystem system: subsystems) {
            system.update(getDt());
        }
    }

    public void initDrive() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);

        driveMotors = new DcMotorEx[]{fl, fr, bl, br};

        for (DcMotorEx motor : driveMotors) {
            //motor.setPositionPIDFCoefficients(Constants.DRIVE_P);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        drive = new Drive(fl, fr, bl, br);
    }

    public void initSpinner() {
        spin = hardwareMap.get(DcMotorEx.class, "spin");

        spin.setDirection(DcMotorSimple.Direction.REVERSE);

        spinner = new Spinner(spin);
    }

    public void initStateEstimator() {
        estimator = new StateEstimator(new IMU(hardwareMap.get(BNO055IMU.class, "imu")),
                                        new MKE(fl, fr, bl, br),
                                        new VV(hardwareMap.get(WebcamName.class, "Webcam"),
                                                hardwareMap.appContext.getResources().getIdentifier(
                                                "cameraMonitorViewId", "id",
                                                        hardwareMap.appContext.getPackageName())));
        timer = new ElapsedTime();
    }

    public double getDt() {
        return dt;
    }

    public StateEstimator getEstimator() {
        return estimator;
    }

    public Drive getDrive() {
        return drive;
    }

    public Spinner getSpinner() {
        return spinner;
    }

    public boolean followPath(Profile profile) {
        if (currDriveProfile == null) {
            currDriveProfile = profile;
            targetPos = getEstimator().getY() + profile.getSetPoint();
        }
        driveTime += getDt();
        if (getDrive().iterativeProfiledDrive(targetPos, tolerance, getEstimator(),
                currDriveProfile, driveTime, kp, kv, ka)) {
            currDriveProfile = null;
            driveTime = 0;
            return true;
        }
        return false;
    }

}

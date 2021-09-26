package org.firstinspires.ftc.teamcode.teamNum.states.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.teamNum.Constants.halfField;
import static org.firstinspires.ftc.teamcode.teamNum.Constants.mmTargetHeight;
import static org.firstinspires.ftc.teamcode.teamNum.Constants.quadField;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.teamNum.Constants;
import org.firstinspires.ftc.teamcode.teamNum.states.DriveState;

import java.util.ArrayList;
import java.util.List;

// Vuforia Vision
public class VV extends DriveState {

    private VuforiaLocalizer vuforia;
    private WebcamName webcamName;

    private OpenGLMatrix lastKnownLocation = MathFx.createMatrix(0, 0, 0, 0, 0, 0);

    private double x, y, theta, x_dot, y_dot;

    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    public VV(WebcamName webcam, int Id) {
        webcamName = webcam;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(Id);
        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = true;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        initVuforia();
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(Constants.robotFromCamera, parameters.cameraDirection);
        }
        x = 0;
        y = 0;
        x_dot = 0;
        y_dot = 0;
        theta = 0;
        //updateRobotLocation();
    }

    public VV(WebcamName webcam, int Id, double x, double y, double theta) {
        webcamName = webcam;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(Id);
        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = true;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        initVuforia();
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(Constants.robotFromCamera, parameters.cameraDirection);
        }
        this.x = x;
        this.y = y;
        x_dot = 0;
        y_dot = 0;
        this.theta = theta;
        //updateRobotLocation();
    }

    public VuforiaTrackable search(VuforiaTrackable target) {
        if (((VuforiaTrackableDefaultListener)target.getListener()).isVisible()) {
            return target;
        } else {
            return null;
        }
    }

    public List<VuforiaTrackable> search() {
        List<VuforiaTrackable> visible = new ArrayList<VuforiaTrackable>();
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                visible.add(trackable);
            }
        }
        return visible;
    }

    public void updateRobotLocation(double dt) {
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (search(trackable) != null) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener())
                        .getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }
                VectorF translation = lastKnownLocation.getTranslation();
                double x2 = MathFx.lowPassFilter(.8, x, translation.get(0)/Constants.mmPerInch);
                double y2 = MathFx.lowPassFilter(.8, y, translation.get(1)/Constants.mmPerInch);
                x_dot = (x2 - x)/dt;
                y_dot = (y2 - y)/dt;
                x = x2;
                y = y2;
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastKnownLocation, EXTRINSIC, XYZ, DEGREES);
                theta = MathFx.lowPassFilter(.8, theta, rotation.firstAngle);
            }
            break;
        }
    }

    @Override
    public void update(double dt) {
        updateRobotLocation(dt);
    }

    private void initVuforia() {
        // TODO Update for 2022 Game
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        allTrackables.addAll(targetsUltimateGoal);
        // Setting Traceable Target Locations
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
    }

    @Override
    public double getX() {
        //getRobotLocation();
        return x;
    }

    @Override
    public double getY() {
        //getRobotLocation();
        return y;
    }

    @Override
    public double getTheta() {
        //getRobotLocation();
        return theta;
    }

    @Override
    public double getX_dot() {
        return x_dot;
    }

    @Override
    public double getY_dot() {
        return y_dot;
    }
}

package org.firstinspires.ftc.teamcode.teamNum;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

public final class Constants {
    // Drive Subsystem
    public static final double L = 18d; // Wheel Base Separation (in)
    public static final double R = 2d; // Drive Wheel Radius (in)
    public static final double COLLISION_THRESHOLD_DELTA_G = 0.5;

    // Vuforia Vision
    public static final String VUFORIA_KEY = "AWWCp8z/////AAABmQqV/K50N0OTqlyIYanMsyQ6huM5ckTKtdjF0/gyTwTINZPIGhLWxx3ag5PUmAw90BOHnZh3arwMSH0sjWZUM7wTJG/rcPmsj3MFp2eSPPc+osid/6jBjyg8YuhBYFN8jO3YvFlo/24qqX8K1DWOX8GU7dAfZEIhI71HCmY+pRWIGxKWyXxkpf3xULPPommaHqF7wSA/z37uQs+zSTs9SJKxiGvUlF7oYkVkURIuzovMKiK7rRqQT/dmCKH/JFpxgl8Er3O50/DL03EMmmNbjkiqA4vAU7wwD8rTkHympjAl7MnSmQRtXWxyRUildftpaQr7rD8vuz+4A6j/+/nKeTUanIi1fPMuE0Xa+Cth7SDr";
    public static final float mmPerInch = 25.4f;
    public static final float mmTargetHeight = (6) * mmPerInch;
    // Perimeter Targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;
    // Camera
    public static final float phoneXRotate = 0;
    public static final float phoneYRotate = -90;
    public static final float phoneZRotate = 0;
    public static final float CAMERA_FORWARD_DISPLACEMENT  = 0 * mmPerInch;
    public static final float CAMERA_VERTICAL_DISPLACEMENT = 0 * mmPerInch;
    public static final float CAMERA_LEFT_DISPLACEMENT = 0 * mmPerInch;
    public static final OpenGLMatrix robotFromCamera = MathFx.createMatrix(CAMERA_FORWARD_DISPLACEMENT,
            CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT, phoneXRotate, phoneYRotate,
            phoneZRotate);

    public static double turnScaling = 1;
    public static double strafeScaling = 1.5;
    public static final double drivePIDMagicNumber = 9.2;

}

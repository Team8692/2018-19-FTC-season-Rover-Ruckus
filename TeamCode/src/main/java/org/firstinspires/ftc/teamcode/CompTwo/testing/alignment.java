package org.firstinspires.ftc.teamcode.CompTwo.testing;

import android.content.res.Configuration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
@Autonomous(name = "testing aling")
@Deprecated
public class alignment extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
            initHardware();
            initCam();
            initVuforia();
            activateVuforia();
            runVuforia();

            while(targetVisible == false) {
                runVuforia();
                turn(0.1,-10,10);
                //TODO: turn robot left
            }

            if (targetVisible == true) {
                if (targetN == 'R' || targetN == 'F') {
                    drive(0.65,y,y);
                    turn(0.75,-90,90);
                    //TODO: depot to the left, crater to the right

                } else if (targetN == 'S' || targetN == 'C') {
                    drive(0.65,x,x);
                    turn(0.75,90,-90);
                    //TODO: depot to the right, crater to the left
                }

        }
    }

    public DcMotor left;
    public DcMotor right;
    public DcMotor leftb;
    public DcMotor rightb;
    public DcMotor intake;
    //public DcMotor intake2;
    public DcMotor lift;
    public Servo dump; //to radians 2. get arc length (does not need to multiply by 2, since robot is symmetrical) 3. drive calculated arc length

    public void initHardware(){
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        leftb = hardwareMap.get(DcMotor.class, "leftb");
        rightb = hardwareMap.get(DcMotor.class, "rightb");
        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake2 = hardwareMap.get(DcMotor.class, "intake2");
        lift = hardwareMap.get(DcMotor.class, "lift");
        dump = hardwareMap.get(Servo.class, "dump");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right.setDirection(DcMotor.Direction.REVERSE);
        rightb.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    static final double COUNTS_PER_MOTOR_REV = 1680;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double ROBOT_RADIUS = 9.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_DEGREE = (Math.PI / 180) * ROBOT_RADIUS * COUNTS_PER_INCH;
    public void drive(double speed, double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            left.setPower(Math.abs(speed));
            right.setPower(Math.abs(speed));
            leftb.setPower(Math.abs(speed));
            rightb.setPower(Math.abs(speed));

            while (opModeIsActive() && (left.isBusy() && right.isBusy())) {
                // Display it for the driver.
                telemetry.addData("RoboPath1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("RoboPath2", "Running at %7d :%7d", left.getCurrentPosition(),
                        right.getCurrentPosition());
                telemetry.update();
            }
            left.setPower(0);
            right.setPower(0);
            leftb.setPower(0);
            rightb.setPower(0);
        }
    }

    //Anthony: added turning code using arc length
    public void turn(double speed, double leftDegrees, double rightDegrees) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = left.getCurrentPosition() + (int) (leftDegrees * COUNTS_PER_DEGREE);
            newRightTarget = right.getCurrentPosition() + (int) (rightDegrees * COUNTS_PER_DEGREE);
            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            left.setPower(Math.abs(speed));
            right.setPower(Math.abs(speed));
            leftb.setPower(Math.abs(speed));
            rightb.setPower(Math.abs(speed));

            while (opModeIsActive() && (left.isBusy() && right.isBusy())) {
                // Display it for the driver.
                telemetry.addData("RoboPath1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("RoboPath2", "Running at %7d :%7d", left.getCurrentPosition(),
                        right.getCurrentPosition());
                telemetry.update();
            }
            left.setPower(0);
            right.setPower(0);
            leftb.setPower(0);
            rightb.setPower(0);
        }
    }

    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final String VUFORIA_KEY =
            "ASkny8n/////AAABmXv23DoGZEwTunMed6nLgzQsNaiOSado20bJHEYv7upxFhmyTqKz3DUy6WNEuTW1FukUH83rccl/O3qq164oQhgUboVNSPcdOWOM/7I93lOciv8Wd4fMGI5lVPKiCA342SaoRuXFHOrNyqMQ8QIojYCEsdlqRb6qMhp2QiTHe/HhjQmPXFk2/T+v4iBC2rwQy5NoywkMFwCZghFjCZxYzaJKSTVPRLq3UmlcaT4+2HpcxTnrGVTjFqQpt9BMgCrDtb/NGfusUr/7njUY1p07XYWK0NIinOsDrBQ0jQA/k7tpxpBNOFOSFtYI9MLId0+emEPAevVSEo4eb1YBhnmbBZjRT2UOlvkCvH9r2FPnozFq";
    public VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters;


    public void initCam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    //tenserflow init, activation, run
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public TFObjectDetector tfod;

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;// the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;
    public boolean targetVisible = false;

    int x, y, z;
    char targetN;
    VuforiaTrackables targetsRoverRuckus;
    // the height of the center of the target image above the floor
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private OpenGLMatrix lastLocation = null;

    void initVuforia() {
        targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");

        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Space");

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        final int CAMERA_FORWARD_DISPLACEMENT = 0;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
        int orientation = hardwareMap.appContext.getResources().getConfiguration().orientation;

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT,
                        CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, orientation == Configuration.ORIENTATION_LANDSCAPE?90:0, 0));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(
                    phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
    }

    void activateVuforia() {
        targetsRoverRuckus.activate();
    }

    void pauseVuforia() {
        targetsRoverRuckus.deactivate();
    }

    void runVuforia() {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;
                targetN = trackable.getName().charAt(0);
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform =
                        ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch,
                    translation.get(2) / mmPerInch);
            x = Math.round(translation.get(0));
            y = Math.round(translation.get(1));
            z = Math.round(translation.get(2));
            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                    rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }
}

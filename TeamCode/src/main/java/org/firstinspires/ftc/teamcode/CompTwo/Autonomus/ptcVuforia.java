//package org.firstinspires.ftc.teamcode.CompTwo.Autonomus;
//
//import android.content.res.Configuration;
//import java.util.ArrayList;
//import java.util.List;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//
//import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
//
//public class ptcVuforia extends cam {
//
//    private static final float mmPerInch = 25.4f;
//    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;
//    // the width of the FTC field (from the center point to the outer panels)
//    private static final float mmTargetHeight = (6) * mmPerInch;
//    public boolean targetVisible = false;
//    int x, y, z;
//    char targetN;
//    VuforiaTrackables targetsRoverRuckus;
//    // the height of the center of the target image above the floor
//    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//    private OpenGLMatrix lastLocation = null;
//
//    void initVuforia() {
//        targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
//
//        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
//        blueRover.setName("Rover");
//        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
//        redFootprint.setName("Footprint");
//        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
//        frontCraters.setName("Craters");
//        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
//        backSpace.setName("Space");
//
//        allTrackables.addAll(targetsRoverRuckus);
//
//        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
//                .translation(0, mmFTCFieldWidth, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
//        blueRover.setLocation(blueRoverLocationOnField);
//
//        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
//                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
//        redFootprint.setLocation(redFootprintLocationOnField);
//
//        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
//                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
//        frontCraters.setLocation(frontCratersLocationOnField);
//
//        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
//                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
//        backSpace.setLocation(backSpaceLocationOnField);
//
//        final int CAMERA_FORWARD_DISPLACEMENT = 0;   // eg: Camera is 110 mm in front of robot center
//        final int CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera is 200 mm above ground
//        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
//        int orientation = hardwareMap.appContext.getResources().getConfiguration().orientation;
//
//        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT,
//                        CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
//                        CAMERA_CHOICE == FRONT ? 90 : -90, orientation == Configuration.ORIENTATION_LANDSCAPE?90:0, 0));
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(
//                    phoneLocationOnRobot, parameters.cameraDirection);
//        }
//
//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start tracking");
//        telemetry.update();
//    }
//
//    void activateVuforia() {
//        targetsRoverRuckus.activate();
//    }
//
//    void pauseVuforia() {
//        targetsRoverRuckus.deactivate();
//    }
//
//    void runVuforia() {
//        targetVisible = false;
//        for (VuforiaTrackable trackable : allTrackables) {
//            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                telemetry.addData("Visible Target", trackable.getName());
//                targetVisible = true;
//                targetN = trackable.getName().charAt(0);
//                // getUpdatedRobotLocation() will return null if no new information is available since
//                // the last time that call was made, or if the trackable is not currently visible.
//                OpenGLMatrix robotLocationTransform =
//                        ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                if (robotLocationTransform != null) {
//                    lastLocation = robotLocationTransform;
//                }
//                break;
//            }
//        }
//
//        // Provide feedback as to where the robot is located (if we know).
//        if (targetVisible) {
//            // express position (translation) of robot in inches.
//            VectorF translation = lastLocation.getTranslation();
//            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch,
//                    translation.get(2) / mmPerInch);
//            x = Math.round(translation.get(0));
//            y = Math.round(translation.get(1));
//            z = Math.round(translation.get(2));
//            // express the rotation of the robot in degrees.
//            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
//                    rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//        } else {
//            telemetry.addData("Visible Target", "none");
//        }
//        telemetry.update();
//    }
//}
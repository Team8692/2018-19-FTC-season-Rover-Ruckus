package org.firstinspires.ftc.teamcode.CompTwo.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
@Autonomous(name = "Testing Minerals")
@Disabled
public class minerals extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initCam();
        initTenserFlow();
        activateTenserFlow();
        waitForStart();
        while(opModeIsActive()) {
            activateTenserFlow();
            //wait for start up
            sleep(500);
            runTenserFlow();

            //distance between minerals: 14.5in, distance from robot to middle mineral: 23.5in
            //angle from robot to left or right minerals is arctan(14.5/23.5)=32deg
            //distance from robot to left or right minerals is sqrt(14.5^2+23.5^2)=27.6
            if (mineralPostion == -1) {
                telemetry.addLine("Left");
                turn(0.5, -31.7, 31.7);
                drive(0.75, 27.6, 27.6);
            } else if (mineralPostion == 1) {
                telemetry.addLine("Right");
                turn(0.5, 31.7, -31.7);
                drive(0.75, 27.6, 27.6);
            } else {
                telemetry.addLine("Center");
                drive(0.75, 23.5, 23.5);
            }
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
    int mineralPostion;

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    void initTenserFlow() {
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
    }

    void activateTenserFlow() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    void pauseTenserFlow() {
        if (tfod != null) {
            tfod.deactivate();
        }
    }

    void runTenserFlow() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() > 1) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                            telemetry.addData("Estimate Degrees to Object", Math.round(recognition.estimateAngleToObject(AngleUnit.DEGREES)));
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            mineralPostion = -1;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            mineralPostion = 1;
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            mineralPostion = 0;
                        }
                    }
                }
                telemetry.update();
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
}

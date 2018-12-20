//package org.firstinspires.ftc.teamcode.CompTwo.Autonomus;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//
//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//
//public class cam extends LinearOpMode {
//    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
//    private static final String VUFORIA_KEY =
//            "ASkny8n/////AAABmXv23DoGZEwTunMed6nLgzQsNaiOSado20bJHEYv7upxFhmyTqKz3DUy6WNEuTW1FukUH83rccl/O3qq164oQhgUboVNSPcdOWOM/7I93lOciv8Wd4fMGI5lVPKiCA342SaoRuXFHOrNyqMQ8QIojYCEsdlqRb6qMhp2QiTHe/HhjQmPXFk2/T+v4iBC2rwQy5NoywkMFwCZghFjCZxYzaJKSTVPRLq3UmlcaT4+2HpcxTnrGVTjFqQpt9BMgCrDtb/NGfusUr/7njUY1p07XYWK0NIinOsDrBQ0jQA/k7tpxpBNOFOSFtYI9MLId0+emEPAevVSEo4eb1YBhnmbBZjRT2UOlvkCvH9r2FPnozFq";
//    public VuforiaLocalizer vuforia;
//    VuforiaLocalizer.Parameters parameters;
//    public HardwareMap hardwareMap;
//    public Telemetry telemetry;
//
//    public void initCam(OpMode captured) {
//        this.hardwareMap = captured.hardwareMap;
//        this.telemetry = captured.telemetry;
//
//        //now that hardwareMap is NOT NULL, hardwareMap can be used as normal
//        //that is, hardwareMap isn't grabbed from the parent LinearOpMode class, but captured
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = CAMERA_CHOICE;
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//    }
//
//    public HardwareMap getHardwareMap() {
//        return hardwareMap;
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initCam(this);
//    }
//}

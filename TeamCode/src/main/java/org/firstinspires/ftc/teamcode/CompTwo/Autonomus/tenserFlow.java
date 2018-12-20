//package org.firstinspires.ftc.teamcode.CompTwo.Autonomus;
//
//import java.util.List;
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//
//public class tenserFlow extends cam {
//    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
//    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
//    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
//    public TFObjectDetector tfod;
//    int mineralPostion;
//
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources()
//                .getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
//    }
//
//    void initTenserFlow() {
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }
//
//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start tracking");
//        telemetry.update();
//    }
//
//    void activateTenserFlow() {
//        if (tfod != null) {
//            tfod.activate();
//        }
//    }
//
//    void pauseTenserFlow() {
//        if (tfod != null) {
//            tfod.deactivate();
//        }
//    }
//
//    void runTenserFlow() {
//        if (tfod != null) {
//            // getUpdatedRecognitions() will return null if no new information is available since
//            // the last time that call was made.
//            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            if (updatedRecognitions != null) {
//                telemetry.addData("# Object Detected", updatedRecognitions.size());
//                if (updatedRecognitions.size() > 1) {
//                    int goldMineralX = -1;
//                    int silverMineral1X = -1;
//                    int silverMineral2X = -1;
//                    for (Recognition recognition : updatedRecognitions) {
//                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                            goldMineralX = (int) recognition.getLeft();
//                            telemetry.addData("Estimate Degrees to Object", Math.round(recognition.estimateAngleToObject(AngleUnit.DEGREES)));
//                        } else if (silverMineral1X == -1) {
//                            silverMineral1X = (int) recognition.getLeft();
//                        } else {
//                            silverMineral2X = (int) recognition.getLeft();
//                        }
//                    }
//                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                            telemetry.addData("Gold Mineral Position", "Left");
//                            mineralPostion = -1;
//                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                            telemetry.addData("Gold Mineral Position", "Right");
//                            mineralPostion = 1;
//                        } else {
//                            telemetry.addData("Gold Mineral Position", "Center");
//                            mineralPostion = 0;
//                        }
//                    }
//                }
//                telemetry.update();
//            }
//        }
//    }
//}
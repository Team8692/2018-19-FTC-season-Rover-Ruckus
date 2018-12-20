//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//
//public class AutoTransitioner extends Thread {
//  private static final AutoTransitioner INSTANCE = new AutoTransitioner();
//  private OpMode onStop;
//  private String nextTarget;
//  private OpModeManagerImpl manager;
//  private boolean run = true;
//
//  private AutoTransitioner() {
//    this.start();
//  }
//
//  public static void initTeleOnStop(OpMode onStop, String nextTarget) {
//    INSTANCE.setNewTransition(onStop, nextTarget);
//  }//when autonomous is stopped or 30 seconds passed, getActiveOpMode will return null. When getActiveOpmode detects a null value, tells manager to initialize nextTarget.
//
//  @Override
//  public void run() {
//    try {
//      while (run) {//thread will die by itself if run = false
//        synchronized (this) {
//          if (onStop != null && manager.getActiveOpMode() != onStop) {
//            Thread.sleep(500); //Wait 0.5 second to prevent weird conditions
//            manager.initActiveOpMode(nextTarget); //Request initialization of the teleop
//            reset(); //Reset the transmitter
//          }
//        }
//        sleep(50);
//      }
//    } catch (InterruptedException e) {
//      e.printStackTrace();
//    }
//  }
//
//  private void setNewTransition(OpMode onStop, String transitionTo) {
//    synchronized (this) { //Synchronized to prevent weird conditions
//      this.onStop = onStop;
//      this.nextTarget = transitionTo;
//      this.manager = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
//    }
//  }
//
//  private void reset() {
//    this.onStop = null;
//    this.nextTarget = null;
//    this.manager = null;
//    //this.run = false;
//  }
//}
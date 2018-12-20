package org.firstinspires.ftc.teamcode.CompTwo.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Depot Only")
@Disabled
public class depot extends autonomus {
    int stated = 1;

    @Override
    public void handleAutoLoop() {
        while (opModeIsActive()) {
            switch (stated) {


                case 1:
                    minerals();
                    stated++;
                    break;
                case 2:
                    align();
                    stated++;
                    break;
                case 3:
                    depot();
                    stated++;
                    break;
            }
        }
    }

    @Override
    public void minerals() {
        super.minerals();
        if(robot.mineralPostion == -1) {
            robot.turn(0.5, 31.7, -31.7);
            robot.drive(0.75, 27.6, 27.6);
        } else if(robot.mineralPostion == 1) {
            robot.turn(0.5, -31.7, 31.7);
            robot.drive(0.75, 27.6, 27.6);
        } else {
            robot.drive(0.75, 20, 20);
        }
    }

    @Override
    public void align(){

    }
    @Override
    public void depot(){
        robot.dump.setPosition(1);
        sleep(500);
        robot.dump.setPosition(0);
    }

}

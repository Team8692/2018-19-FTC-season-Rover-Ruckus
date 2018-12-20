package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Depot Auto")
public class DepotAuto extends FullAuto {
    @Override
    public void depot(){
        drive(0.5,5);
        dump.setPosition(1);
        sleep(600);
        dump.setPosition(0);
    }

    @Override
    public void align(){

    }
    @Override
    public void crater(){

    }
}

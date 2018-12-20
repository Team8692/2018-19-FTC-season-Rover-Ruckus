package org.firstinspires.ftc.teamcode.SAFE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Crater Auto")
public class CraterAuto extends FullAuto {

    @Override
    public void crater(){
        drive(0.5,10);
    }

    @Override
    public void depot(){

    }

    @Override
    public void align(){

    }
}

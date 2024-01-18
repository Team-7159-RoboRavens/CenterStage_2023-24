package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="argh")
public class VeryQuickTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MachineVision mv = new MachineVision(hardwareMap, this);
        sleep(1500);
        int placement = mv.run();

        waitForStart();
    }
}

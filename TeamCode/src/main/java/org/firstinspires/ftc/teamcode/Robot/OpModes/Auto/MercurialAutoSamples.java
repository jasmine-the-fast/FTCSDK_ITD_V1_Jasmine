package org.firstinspires.ftc.teamcode.Robot.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTRobotV1;
import org.firstinspires.ftc.teamcode.Robot.Structure.Library.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Autonomous(name = "Sample_Auto", group = "OpMode")
public class MercurialAutoSamples extends OpMode {
    @Override
    public void init() {

    }
    @Override
    public void loop() {
        //leave empty, use start instead
    }
    @Override
    public void start() {
        new Sequential(
                new Wait(0.2)
        )
                .schedule();
    }
}

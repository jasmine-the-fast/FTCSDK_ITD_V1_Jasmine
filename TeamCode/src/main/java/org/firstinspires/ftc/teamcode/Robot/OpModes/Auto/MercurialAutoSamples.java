package org.firstinspires.ftc.teamcode.Robot.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.AutoPaths;
import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTRobotSubsystems;
import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTRobotV1;
import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTSubsystems.BTChassis;
import org.firstinspires.ftc.teamcode.Robot.Structure.Library.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Autonomous(name = "Auto_Merc_Sub", group = "OpMode")
public class MercurialAutoSamples extends OpMode {
    public static BTRobotSubsystems robot_subsystems = new BTRobotSubsystems();


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
                BTChassis.followPath(AutoPaths.scorePreload),
                new Wait(5.0),
                BTChassis.followPathChain(AutoPaths.grabPickup1),
                new Wait(5.0)
        )
                .schedule();
    }
}

package org.firstinspires.ftc.teamcode.Robot.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTRobotSubsystems;
import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTRobotV1;
import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTSubsystems.BTChassis;
import org.firstinspires.ftc.teamcode.Robot.Structure.Library.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Mercurial.Attach
@BTChassis.Attach
@BulkRead.Attach

@Autonomous(name = "Auto_Merc_Sub", group = "OpMode")
public class MercurialAutoSamples extends OpMode {
    public static BTRobotSubsystems robot_subsystems = new BTRobotSubsystems();
    public static Follower follower;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        robot_subsystems.init(follower);
        follower.setStartingPose(BTRobotSubsystems.startPose);
    }

    @Override
    public void loop() {
        //leave empty, use start instead
    }
    @Override
    public void start() {
        new Sequential(
                BTChassis.followPath(BTRobotSubsystems.scorePreload),
                new Wait(5.0),
                BTChassis.followPathChain(BTRobotSubsystems.grabPickup1),
                new Wait(5.0)
        )
                .schedule();
    }
}

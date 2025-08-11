package org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTSubsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;

import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.AutoPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;


import java.util.function.Supplier;

import dev.frozenmilk.dairy.core.Feature;
import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;

public class BTChassis implements Subsystem {

    public static final BTChassis INSTANCE = new BTChassis();
    public static Follower follower;
    public static Telemetry telemetry;
    public static DashboardPoseTracker dashboardPoseTracker;
    public static AutoPaths ourPaths;

    public BTChassis() {}

    @Override
    public boolean isActive() {
        return Subsystem.super.isActive();
    }

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return null;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {

    }

    /*
    @Override
    public void cleanup(@NonNull Wrapper opMode) {
        Subsystem.super.cleanup(opMode);
    }
    */

/*
    @NonNull
    @Override
    public Feature deregister() {
        return Subsystem.super.deregister();
    }
*/

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        telemetry = opMode.getOpMode().telemetry;
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(FeatureRegistrar.getActiveOpMode().hardwareMap, FConstants.class, LConstants.class);

        dashboardPoseTracker = BTChassis.follower.getDashboardPoseTracker();

        ourPaths.buildPaths(follower);

        follower.setStartingPose(AutoPaths.startPose);

    }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        Subsystem.super.postUserInitHook(opMode);
    }

    @Override
    public void postUserInitLoopHook(@NonNull Wrapper opMode) {
        Subsystem.super.postUserInitLoopHook(opMode);
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
        Subsystem.super.postUserLoopHook(opMode);
    }

    @Override
    public void postUserStartHook(@NonNull Wrapper opMode) {
        Subsystem.super.postUserStartHook(opMode);
    }

    @Override
    public void postUserStopHook(@NonNull Wrapper opMode) {
        Subsystem.super.postUserStopHook(opMode);
    }



    @Override
    public void preUserInitLoopHook(@NonNull Wrapper opMode) {
        Subsystem.super.preUserInitLoopHook(opMode);
    }

    @Override
    public void preUserLoopHook(@NonNull Wrapper opMode) {
        Subsystem.super.preUserLoopHook(opMode);
    }

    @Override
    public void preUserStartHook(@NonNull Wrapper opMode) {
        Subsystem.super.preUserStartHook(opMode);
    }

    @Override
    public void preUserStopHook(@NonNull Wrapper opMode) {
        Subsystem.super.preUserStopHook(opMode);
    }

    @NonNull
    @Override
    public Feature register() {
        return Subsystem.super.register();
    }

    @Nullable
    @Override
    public Command getDefaultCommand() {
        return Subsystem.super.getDefaultCommand();
    }

    @Override
    public void setDefaultCommand(@Nullable Command value) {
        Subsystem.super.setDefaultCommand(value);
    }

    @NonNull
    @Override
    public <T> SubsystemObjectCell<T> subsystemCell(@NonNull Supplier<T> supplier) {
        return Subsystem.super.subsystemCell(supplier);
    }

    public static Lambda followPath(Path path) {
        return new Lambda("follow-path")
                .addRequirements(INSTANCE)
                .setInit(() -> follower.followPath(path, true))
                .setExecute(() -> {
                    follower.update();
                    telemetry.addData("x", follower.getPose().getX());
                    telemetry.addData("y", follower.getPose().getY());
                    telemetry.addData("heading", follower.getPose().getHeading());
                })
                .setFinish(() -> !follower.isBusy() || follower.isRobotStuck());
    }

    public static Lambda followPathChain(PathChain chain) {
        return new Lambda("follow-path-chain")
                .addRequirements(INSTANCE)
                .setInit(() -> follower.followPath(chain, true))
                .setExecute(() -> {
                    follower.update();
                    follower.telemetryDebug(telemetry);
                })
                .setFinish(() -> !follower.isBusy() || follower.isRobotStuck());
    }
}

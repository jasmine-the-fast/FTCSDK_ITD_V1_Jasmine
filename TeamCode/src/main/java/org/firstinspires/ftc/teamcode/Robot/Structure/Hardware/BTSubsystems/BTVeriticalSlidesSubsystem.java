package org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTSubsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.function.Supplier;

import dev.frozenmilk.dairy.core.Feature;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;

public class BTVeriticalSlidesSubsystem implements Subsystem {
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

    @Override
    public void cleanup(@NonNull Wrapper opMode) {
        Subsystem.super.cleanup(opMode);
    }

    @NonNull
    @Override
    public Feature deregister() {
        return Subsystem.super.deregister();
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
    public void preUserInitHook(@NonNull Wrapper opMode) {
        Subsystem.super.preUserInitHook(opMode);
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
}

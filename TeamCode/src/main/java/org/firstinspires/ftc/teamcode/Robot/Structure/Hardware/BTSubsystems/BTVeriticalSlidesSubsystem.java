package org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTSubsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


import java.util.function.Supplier;
import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.Feature;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;

import kotlin.annotation.MustBeDocumented;

public class BTVeriticalSlidesSubsystem implements Subsystem {
    public static final BTVeriticalSlidesSubsystem INSTANCE = new BTVeriticalSlidesSubsystem();
    public static Telemetry telemetry;
    public static DcMotor VLL, VLR;

    private BTVeriticalSlidesSubsystem() {}

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    @Override
    public boolean isActive() {
        return Subsystem.super.isActive();
    }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
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
        HardwareMap hMap = opMode.getOpMode().hardwareMap;
        telemetry = opMode.getOpMode().telemetry;
        telemetry.addData("BTVeriticalSlidesSubsystem", "preUserInitHook");

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


}

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
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;

import kotlin.annotation.MustBeDocumented;

public class BTVeriticalSlidesSubsystem implements Subsystem {
    public static final BTVeriticalSlidesSubsystem INSTANCE = new BTVeriticalSlidesSubsystem();
    public static Telemetry telemetry;
    public static DcMotor VLL, VLR;
    public static int maxPos = 800;
    public static int minPos = 0;
    public static int tolerance = 10;

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
        telemetry.addData("BTVeriticalSlidesSubsystem", "postUserInitHook");
        Subsystem.super.postUserInitHook(opMode);
    }

    @Override
    public void postUserStartHook(@NonNull Wrapper opMode) {
        telemetry.addData("BTVeriticalSlidesSubsystem", "postUserStartHook");
        Subsystem.super.postUserStartHook(opMode);
    }

    @Override
    public void postUserStopHook(@NonNull Wrapper opMode) {
        telemetry.addData("BTVeriticalSlidesSubsystem", "postUserStopHook");
        Subsystem.super.postUserStopHook(opMode);
    }

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hMap = opMode.getOpMode().hardwareMap;
        telemetry = opMode.getOpMode().telemetry;
        telemetry.addData("BTVeriticalSlidesSubsystem", "preUserInitHook");
        VLL = setupAutoLiftMotor("VLL", DcMotor.Direction.FORWARD, hMap);
        VLR = setupAutoLiftMotor("VLR", DcMotor.Direction.REVERSE, hMap);

    }


    @Override
    public void preUserStartHook(@NonNull Wrapper opMode) {
        telemetry.addData("BTVeriticalSlidesSubsystem", "preUserStartHook");
        Subsystem.super.preUserStartHook(opMode);
    }

    @Override
    public void preUserStopHook(@NonNull Wrapper opMode) {
        telemetry.addData("BTVeriticalSlidesSubsystem", "preUserStopHook");
        Subsystem.super.preUserStopHook(opMode);
    }

    public static void setPower(double power){
        telemetry.addData("BTVerticalSlidesSubsystem setPower: ", power);
        VLL.setPower(power);
        VLR.setPower(power);
    }

    public static Lambda setPowerCommand(double power){
        return new Lambda("set-power")
                .setExecute(() -> {
                    setPower(power);
                });
    }


    public static Lambda runToPosition(int pos){
        return new Lambda("set-target-pos")
                .setInterruptible(true)
                .setInit(() -> {
                    VLL.setTargetPosition(pos);
                    VLR.setTargetPosition(pos);
                });
    }

    public static Lambda waitForPos(int pos) {
        return new Lambda("wait-for-pos")
                .setFinish(() -> Math.abs(getPos() - pos) < tolerance);
    }

    public static double getPos(){
        return VLL.getCurrentPosition();
    }

    public void resetSlides() {
            VLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            VLL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            VLL.setTargetPosition(0);
            VLL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            VLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            VLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            VLR.setTargetPosition(0);
            VLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private DcMotor setupAutoLiftMotor(String deviceName, DcMotor.Direction direction, HardwareMap hMap) {
        DcMotor aMotor = hMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aMotor.setTargetPosition(0);
        aMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return aMotor;
    }

}

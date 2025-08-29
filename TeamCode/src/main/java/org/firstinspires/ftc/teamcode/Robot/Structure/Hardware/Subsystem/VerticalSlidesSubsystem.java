package org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.Subsystem;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class VerticalSlidesSubsystem extends SubsystemBase {
    public static Motor VLL;
    public static Motor VLR;

    public VerticalSlidesSubsystem(HardwareMap hMap){
        VLL = new Motor(hMap,"VLL", 90, 312); //check these, probably incorrect
        VLR = new Motor(hMap,"VLR", 90, 312); //check these, probably incorrect

        VLL.setRunMode(Motor.RunMode.PositionControl);
        VLR.setRunMode(Motor.RunMode.PositionControl);

    }

}

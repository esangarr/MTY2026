package frc.robot.Commands.MecaCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Intake.IntakeSub;

public class IntakeCommands {

    public static Command idle(IntakeSub intake){
        return Commands.run(()-> {intake.stopAll();},intake);
    }
    
}

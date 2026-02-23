package frc.robot.Commands.MecaCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Index.IndexSub;

public class IndexCommands {

    public static Command idle(IndexSub index){
        return Commands.run(()-> {index.stop();},index);
    }
    
}

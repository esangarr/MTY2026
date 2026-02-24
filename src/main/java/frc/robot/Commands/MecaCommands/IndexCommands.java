package frc.robot.Commands.MecaCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Index.IndexSub;

public class IndexCommands {

    public static Command idle(IndexSub index){
        return Commands.run(()-> {index.stop();},index);
    }

    public static Command RunRollers (IndexSub index, double speed){

        return Commands.run(()->{RunRollers(index, speed);}, index);
    }

    public static Command RunIndex (IndexSub index, double speed){

        return Commands.run(()->{RunIndex(index, speed);}, index);
    }

    public static Command Run (IndexSub index, double speed){

        return Commands.run(()->{Run(index, speed);}, index);
    }
    
}

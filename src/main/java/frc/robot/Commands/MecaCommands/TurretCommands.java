package frc.robot.Commands.MecaCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Outake.Turret.TurretSub;

public class TurretCommands {

    public static Command idle(TurretSub turret){
        return Commands.run(()-> {turret.stop();},turret);
    }
    
}

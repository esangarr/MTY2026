package frc.robot.Commands.MecaCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Climber.ClimberSub;

public class ClimberCommands {

   public Command Climb(ClimberSub climber, double speed){
        return Commands.run(()-> {climber.climb(speed);}, climber).finallyDo(()->{climber.stop();});
    }
    public Command ClimbReach(ClimberSub climber, double targetVoltage){
        return Commands.run(()-> {climber.reachVoltage(targetVoltage);}, climber);
    }
}
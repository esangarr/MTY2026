package frc.robot.Commands.MecaCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Mechanisms.Intake.IntakeSub;
import frc.robot.Mechanisms.Index.IndexSub;


public class IntakeCommands {

    public static Command idle(IntakeSub intake){
        return Commands.run(()-> {intake.stopAll();},intake);
    }
    
    public static Command moveVoltage(IntakeSub intake, double voltage){
        return Commands.run(()-> {intake.setVoltageAng(voltage);},intake).finallyDo(()-> {intake.stopAng();});
    }
    
    public static Command movePID(IntakeSub intake, double angle, IntakeSub.MODE mode){
        return Commands.run(()-> {intake.setAngle(angle, mode);},intake).finallyDo(()-> {intake.stopAng();});
    }

    public static Command moveRollers(IntakeSub intake, double speed){
        return Commands.run(()-> {intake.setRollers(speed);},intake).finallyDo(()-> {intake.stopWheelsIntake();});
    }

    public static Command moveRollersIndex(IntakeSub intake, IndexSub index, double speed){
        return Commands.run(()-> {
            intake.setRollers(speed);
            index.Run(speed);},intake, index).finallyDo(()-> {intake.stopWheelsIntake();index.stop();
        });

    }

}
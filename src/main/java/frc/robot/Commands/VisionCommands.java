package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DriveTrain.SwerveSubs.CommandSwerveDrivetrain;

public class VisionCommands {

    public static Command snapToApril(CommandSwerveDrivetrain swerve){
        return Commands.run(()-> {swerve.aproachY();},swerve);
    }
    
    public static Command snapToApril2(CommandSwerveDrivetrain swerve){
        return Commands.run(()-> {swerve.aproachXY();},swerve);
    }

    public static Command ResetQuestPose(CommandSwerveDrivetrain swerve){
        return Commands.run(()-> {swerve.setQuestPose(new Pose3d());});
    }

    
    
}

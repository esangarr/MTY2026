package frc.robot.Mechanisms.Outake.Turret;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stzteam.forgemini.io.IOSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Mechanisms.MechanismsConstants.TurretConstants;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TurretSub extends IOSubsystem{

    private final SparkMax azimuth;
    private final AbsoluteEncoder encoder;

    public TurretSub(){
        super("TurretSubsistem");

        azimuth = new SparkMax(0, MotorType.kBrushless);
        encoder = azimuth.getAbsoluteEncoder();

        motorConfig();
    }

    public void motorConfig(){
        var config = new SparkMaxConfig();
        
        var profiles = config.closedLoop;

        azimuth.setCANTimeout(250);

            try{

                profiles.pid(
                    TurretConstants.kP,
                    TurretConstants.kI,
                    TurretConstants.kD).
                    outputRange(TurretConstants.kMinOutput, TurretConstants.kMaxOutput);

                profiles.feedForward.kS(TurretConstants.kS).kV(TurretConstants.kV).kA(TurretConstants.kA);

                profiles.maxMotion.cruiseVelocity(TurretConstants.kCruiseVelocity).maxAcceleration(TurretConstants.kMaxAcc);

                config.
                    idleMode(IdleMode.kBrake).
                    inverted(TurretConstants.kMotorInverted).
                    smartCurrentLimit(TurretConstants.kCurrentLimit).
                    voltageCompensation(TurretConstants.kMaxVolts);
                
                config.softLimit.
                    forwardSoftLimit(TurretConstants.kUpperLimit).
                    forwardSoftLimitEnabled(true).
                    reverseSoftLimit(TurretConstants.kLowerLimit).
                    reverseSoftLimitEnabled(true);

                config.absoluteEncoder.
                    inverted(TurretConstants.kEncoderInverted).
                    positionConversionFactor(TurretConstants.kPositionFactor).
                    velocityConversionFactor(TurretConstants.kVelocityFactor);
        
                //Si usar encoder relativo configurarlo y poner esto
                //getMotor().getEncoder().setPosition(0);

                azimuth.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            }
            
            finally{
                azimuth.setCANTimeout(0);
            }
    }

    public void stop() {
        azimuth.stopMotor();
    }

    public void setPosition(Rotation2d angle) {
        azimuth.getClosedLoopController().setSetpoint(angle.getRotations(), ControlType.kMAXMotionPositionControl);
    }

    public double getPosition(){
        return encoder.getPosition();
    }
    
    @Override 
    public void periodicLogic() {}
    
}

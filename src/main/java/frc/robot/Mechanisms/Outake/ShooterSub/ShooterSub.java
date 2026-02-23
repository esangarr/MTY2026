package frc.robot.Mechanisms.Outake.ShooterSub;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stzteam.forgemini.io.IOSubsystem;

import frc.robot.Mechanisms.MechanismsConstants.ShooterConstants;

public class ShooterSub extends IOSubsystem{

    private final SparkMax leaderMot, followerMot;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController leaderPID;


    public ShooterSub(){
        super("ShooterSubsystem");
        leaderMot =  new SparkMax(ShooterConstants.leaderMot_ID, MotorType.kBrushless);
        followerMot = new SparkMax(ShooterConstants.followerMot_ID, MotorType.kBrushless);

        leaderPID = leaderMot.getClosedLoopController();
        encoder = leaderMot.getEncoder();


    }

    public void configureMotor(){
        var leaderConfig = new SparkMaxConfig();
        var followerConfig = new SparkMaxConfig();
        
        leaderMot.setCANTimeout(250);
        followerMot.setCANTimeout(250);

        leaderConfig.
                idleMode(IdleMode.kBrake).
                inverted(ShooterConstants.absoluteInverted).
                smartCurrentLimit(ShooterConstants.kCurrentLimit).
                voltageCompensation(ShooterConstants.kMaxVolts).
                closedLoop.pid(0,0,0).
                feedForward.kS(0).kA(0).kV(0);


        followerConfig.follow(leaderMot, ShooterConstants.followerInverted);

        leaderMot.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMot.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leaderMot.setCANTimeout(0);
        followerMot.setCANTimeout(0);

    }

    public double getVelocity(){
        return encoder.getVelocity();
    }

    public void setVelTarget(double velocityTarget){
        leaderPID.setSetpoint(velocityTarget, ControlType.kVelocity);
    }

    public void moveSpeed(double speed){
        leaderMot.set(speed);
        followerMot.set(speed);
    }

    public void stopALL(){
        leaderMot.stopMotor();
        followerMot.stopMotor();
    }

    @Override 
    public void periodicLogic() {}
    
}

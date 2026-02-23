package frc.robot.Mechanisms.Climber;

import com.stzteam.forgemini.io.IOSubsystem;
import com.stzteam.forgemini.io.Signal;

import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;


import frc.robot.Mechanisms.MechanismsConstants.ClimberConstants;


public class ClimberSub extends IOSubsystem{

    private final TalonFX climberMotor;
    private final TalonFXConfiguration climberConfig;

    private final MotorOutputConfigs motorConfigs ;
    private final CurrentLimitsConfigs limitConfigs ;

    private final VoltageOut voltageRequest;

    private static final String TABLE = "ClimberSubsystem";


    public ClimberSub(){
        super(TABLE);

        motorConfigs = new MotorOutputConfigs();
        limitConfigs = new CurrentLimitsConfigs();

        voltageRequest = new VoltageOut(0);


        motorConfigs.Inverted = ClimberConstants.invertedValue;
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
    
        limitConfigs.StatorCurrentLimit = ClimberConstants.currentLimit;
        limitConfigs.StatorCurrentLimitEnable = true;        

        climberMotor = new TalonFX(ClimberConstants.CLIMBER_ID, "Canivore");
        climberConfig = new TalonFXConfiguration();

        climberConfig.Feedback.SensorToMechanismRatio = ClimberConstants.gear_Ratio;
    

        climberMotor.getConfigurator().apply(motorConfigs);
    }

    @Signal(key="Position")
    public double getPositionAng(){
        return climberMotor.getRotorPosition().getValueAsDouble();
    }
    

    @Signal(key="Voltage")
    public double getVoltage(){
        return climberMotor.getMotorVoltage().getValueAsDouble();
    }

    public void reachVoltage(double targetVoltage){
        climberMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    public void goUp(double speed){
        climberMotor.setVoltage(speed);
    }
    public void goDown(double speed){
        climberMotor.setVoltage(-speed);
    }
    public void stop(){
        climberMotor.stopMotor();
    }


    @Override
    public void periodicLogic() {}
}


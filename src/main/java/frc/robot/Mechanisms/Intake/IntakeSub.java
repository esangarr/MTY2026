//Importations
package frc.robot.Mechanisms.Intake;

// Forgemini
import com.stzteam.forgemini.io.IOSubsystem;
import com.stzteam.forgemini.io.Signal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;


import edu.wpi.first.math.util.Units;

import frc.robot.Mechanisms.MechanismsConstants;
import frc.robot.Mechanisms.MechanismsConstants.IntakeConstants;



public class IntakeSub extends IOSubsystem {

    
    //objects
    private TalonFX intakeMotor;
    private TalonFXConfigurator intakeConfigurator;

    private TalonFX angulatorMotor;
    private TalonFXConfigurator angulatorConfigurator;
    private TalonFXConfiguration angConfigs ;

    //Elastic

    private static final String TABLE = "Intake";
    
    //MotionMagic
         
    private MotionMagicExpoVoltage angulator_request;

    public IntakeSub(){
        
        super(TABLE);
        
        intakeMotor = new TalonFX(MechanismsConstants.IntakeConstants.ROLLERS_CAN_ID);
        angulatorMotor = new TalonFX(MechanismsConstants.IntakeConstants.ANGULATOR_CAN_ID);
        

        intakeConfigurator = intakeMotor.getConfigurator();
        angulatorConfigurator = angulatorMotor.getConfigurator();

        angConfigs = new TalonFXConfiguration();

        angConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;

        angulator_request = new MotionMagicExpoVoltage(0);

        var motorConfigs = new MotorOutputConfigs();

        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 80;
        limitConfigs.StatorCurrentLimitEnable = true;

        
        intakeConfigurator.apply(limitConfigs);
        angulatorConfigurator.apply(limitConfigs);

        angulatorConfigurator.apply(angConfigs);

        intakeConfigurator.refresh(motorConfigs);
        intakeConfigurator.refresh(motorConfigs);

        angulatorConfigurator.refresh(motorConfigs);
        angulatorConfigurator.refresh(motorConfigs);

        configMotion();

    }

    

    public void configMotion (){
      
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.0; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0.0; // no output for integrated error
        slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic Expo settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0.0; // kV is around 0.12 V/rps
        motionMagicConfigs.MotionMagicExpo_kA = 0.0; // Use a slower kA of 0.1 V/(rps/s)

        angulatorMotor.getConfigurator().apply(talonFXConfigs);

        
    }

    @Signal(key="Position")
    public double getPositionAng(){
        return angulatorMotor.getRotorPosition().getValueAsDouble();
    }

    public void setVoltageAng (double voltage){
        angulatorMotor.setVoltage(voltage);
    }

    public void setRollers(double speed){
        intakeMotor.set(speed);
    }

    @Signal(key="VelocityRollers")
    public double getVelocityIntake (){
        return intakeMotor.getVelocity().getValueAsDouble();
    }

   
    public void setAngle (double angleSetpoint){
        double target = Units.degreesToRotations(angleSetpoint);
        angulatorMotor.setControl(angulator_request.withPosition(target));

    }

    @Signal(key="Setpoint")
    public double currentSetpoint() {
        return angulatorMotor.getClosedLoopReference().getValueAsDouble();
    }

    @Signal(key="AlGoal")
    public boolean atGoal() {

        return Math.abs(currentSetpoint() - getPositionAng()) < IntakeConstants.Tolerance;
    }

    public void stopAng() {
        angulatorMotor.stopMotor();
    }

    public void stopWheelsIntake() {
        intakeMotor.stopMotor();
    }

    public void stopAll(){
        angulatorMotor.stopMotor();
        intakeMotor.stopMotor();
    }
    

    @Override
    public void periodicLogic(){

    }

}
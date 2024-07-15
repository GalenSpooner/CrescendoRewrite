package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import RockinLib.MotorControllers.RockinTalon;
import RockinLib.Sensors.RockinCancoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.ShooterTarget;

public class Shooter extends SubsystemBase  {
    PivotState state = PivotState.STOW;
    PivotState prevState;
    RockinTalon topFlywheel;
    RockinTalon bottomFlywheel;
    RockinTalon topFeeder;
    RockinTalon bottomFeeder;
    RockinTalon pivot;
    static ShooterTarget target;
    boolean shooting = false;
    TalonFXConfiguration flywheelConfig;
    TalonFXConfiguration pivotConfig;
    static CommandSwerveDrivetrain drivetrain;
    MotionMagicVoltage pivotVoltage;
    VelocityVoltage flywheelVoltage;
    BooleanSupplier intaking;
    public Shooter(CommandSwerveDrivetrain drive, BooleanSupplier intaking){
        this.intaking = intaking;
        SmartDashboard.putNumber("Manual Angle", 15);
        SmartDashboard.putNumber("Manual Velocity", 15);
        target = new ShooterTarget(null, null, null);
        drivetrain = drive;
        topFlywheel = new RockinTalon(ShooterConstants.SHOOTER_TOPFLYWHEEL_ID, 50);
        bottomFlywheel = new RockinTalon(ShooterConstants.SHOOTER_BOTTOMFLYWHEEL_ID,50);
        topFeeder = new RockinTalon(ShooterConstants.SHOOTER_TOPFEEDER_ID,40);
        bottomFeeder = new RockinTalon(ShooterConstants.SHOOTER_BOTTOMFEEDER_ID,40);
        pivot = new RockinTalon(0, 40);
        flywheelConfig = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKA(0.41).withKV(0.25).withKP(0.1));
        topFlywheel.getConfigurator().refresh(flywheelConfig);
        bottomFlywheel.getConfigurator().refresh(flywheelConfig);
        topFeeder.getConfigurator().refresh(flywheelConfig);
        bottomFeeder.getConfigurator().refresh(flywheelConfig);
        pivotConfig = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKA(0).withKV(2.25).withKG(0.17).withKP(0.1))
        .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(0.75).withMotionMagicCruiseVelocity(5)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(100));
        pivot.getConfigurator().refresh(pivotConfig); 
        pivot.setPosition(0);
        flywheelVoltage = new VelocityVoltage(0);
        flywheelVoltage.Slot = 0;
        pivotVoltage = new MotionMagicVoltage(5);
        pivotVoltage.withSlot(0);
    }
    public enum PivotState{
        SPEAKER(getSpeakerAngle()),
        AMP(110),
        STOW(5),
        INTAKING(45),
        PASSING(75),
        MANUAL(SmartDashboard.getNumber("Manual Angle", 15));
        public double angle;
        private PivotState(double angle) {
            this.angle = angle;
        }
    }
    
    public Command setState(PivotState state){
        return runOnce(() -> {
            this.prevState = this.state;
            this.state = state;
            
        });
    }


    @Override
    public void periodic() {    
        if(!shooting){
            topFlywheel.setControl(flywheelVoltage.withVelocity(2));
            bottomFlywheel.setControl(flywheelVoltage.withVelocity(2));
        }
        if(intaking.getAsBoolean()){
            this.prevState = this.state;
            this.state = PivotState.INTAKING;
        }
        if(this.getState() == PivotState.INTAKING && !intaking.getAsBoolean()){
            this.state = this.prevState;
            this.prevState = PivotState.INTAKING;
        }

        pivot.setControl(pivotVoltage.withPosition(Units.degreesToRotations(state.angle)));
        SmartDashboard.putBoolean("Shooting", shooting);
        SmartDashboard.putString("Shooter Aim State", state.toString());
        SmartDashboard.putNumber("Shooter Angle", pivot.getRotorPosition().getValueAsDouble() / 3600);
        SmartDashboard.putNumber("Flywheel velocity", topFlywheel.getVelocity().getValueAsDouble());
    }
    public static double getSpeakerAngle(){
        
        return (target.calculateFromDistance(Math.sqrt((Math.pow((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? drivetrain.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED : drivetrain.getPose().getX(),2)) + (Math.pow(5.5 - drivetrain.getPose().getY(), 2))))).angle;
    }
    public double getSpeakerVelocity(){
        return (target.calculateFromDistance(Math.sqrt((Math.pow((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? drivetrain.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED : drivetrain.getPose().getX(),2)) + (Math.pow(5.5 - drivetrain.getPose().getY(), 2))))).velocity;
    }
    public Command ShootSpeaker(){
        return new SequentialCommandGroup(
            runOnce(() -> {
                shooting = true;
                flywheelVoltage.withVelocity((this.state == PivotState.MANUAL) ? SmartDashboard.getNumber("Manual Velocity", 75) : getSpeakerVelocity());
                topFlywheel.setControl(flywheelVoltage);
                bottomFlywheel.setControl(flywheelVoltage);
                }),
                new ParallelRaceGroup(Commands.waitUntil(() -> MathUtil.isNear(flywheelVoltage.Velocity,topFlywheel.getVelocity().getValueAsDouble(),5)), Commands.waitSeconds(1.5)),
            runOnce(() -> {
                topFeeder.setControl(flywheelVoltage);
                bottomFeeder.setControl(flywheelVoltage);
            }),
            Commands.waitSeconds(ShooterConstants.SHOOTER_TIME_TO_SCORE),
            runOnce(() -> {
                flywheelVoltage.Velocity = 0;
                shooting = false;
            }));
    }
    public Command ShootAmp(){
        return new SequentialCommandGroup(
            runOnce(() -> {
                shooting = true;
                flywheelVoltage.withVelocity(50);
                topFlywheel.setControl(flywheelVoltage);
                bottomFlywheel.setControl(flywheelVoltage);
                }),
                new ParallelRaceGroup(Commands.waitUntil(() -> MathUtil.isNear(flywheelVoltage.Velocity,topFlywheel.getVelocity().getValueAsDouble(),5)), Commands.waitSeconds(1.5)),
            runOnce(() -> {
                topFeeder.setControl(flywheelVoltage);
                bottomFeeder.setControl(flywheelVoltage);
            }),
            Commands.waitSeconds(ShooterConstants.SHOOTER_TIME_TO_SCORE),
            runOnce(() -> {
                flywheelVoltage.Velocity = 0;
                shooting = false;
            }));
    }
    public Command Pass(){
        return new SequentialCommandGroup(
            runOnce(() -> {
                shooting = true;
                flywheelVoltage.withVelocity(100);
                topFlywheel.setControl(flywheelVoltage);
                bottomFlywheel.setControl(flywheelVoltage);
                }),
                new ParallelRaceGroup(Commands.waitUntil(() -> MathUtil.isNear(flywheelVoltage.Velocity,topFlywheel.getVelocity().getValueAsDouble(),7.5)), Commands.waitSeconds(1.5)),
            runOnce(() -> {
                topFeeder.setControl(flywheelVoltage);
                bottomFeeder.setControl(flywheelVoltage);
            }),
            Commands.waitSeconds(ShooterConstants.SHOOTER_TIME_TO_SCORE),
            runOnce(() -> {
                flywheelVoltage.Velocity = 0;
                shooting = false;
            }));
    }
    public Command Score(){
        switch(state){
            case SPEAKER:
                return ShootSpeaker();
            case AMP:
                return ShootAmp();
            default:
                return Pass();
        }
        
    }
    public boolean isShooting(){
        return shooting;
    }
    public PivotState getState(){
        return this.state;
    }
}

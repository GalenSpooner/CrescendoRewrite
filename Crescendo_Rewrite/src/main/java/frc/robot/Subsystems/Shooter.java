package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import RockinLib.MotorControllers.RockinTalon;
import RockinLib.Sensors.RockinCancoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterTarget;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase  {
    PivotState state = PivotState.STOW;
    RockinTalon topFlywheel;
    RockinTalon bottomFlywheel;
    RockinTalon topFeeder;
    RockinTalon bottomFeeder;
    RockinTalon pivot;
    RockinCancoder encoder;
    ShooterTarget target;
    TalonFXConfiguration flywheelConfig;
    TalonFXConfiguration pivotConfig;
    CommandSwerveDrivetrain drivetrain;
    public Shooter(CommandSwerveDrivetrain drive){
        drivetrain = drive;
        topFlywheel = new RockinTalon(ShooterConstants.SHOOTER_TOPFLYWHEEL_ID, 50);
        bottomFlywheel = new RockinTalon(ShooterConstants.SHOOTER_BOTTOMFLYWHEEL_ID,50);
        topFeeder = new RockinTalon(ShooterConstants.SHOOTER_TOPFEEDER_ID,40);
        bottomFeeder = new RockinTalon(ShooterConstants.SHOOTER_BOTTOMFEEDER_ID,40);
        pivot = new RockinTalon(0, 40);
        encoder = new RockinCancoder(ShooterConstants.SHOOTER_PIVOT_ENCODER_ID, "rio", "Pivot Encoder");
        encoder.attachToTalonFX(pivot);
        flywheelConfig = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKA(0.41).withKV(0.25).withKP(0.1));
        topFlywheel.getConfigurator().apply(flywheelConfig);
        bottomFlywheel.getConfigurator().apply(flywheelConfig);
        topFeeder.getConfigurator().apply(flywheelConfig);
        bottomFeeder.getConfigurator().apply(flywheelConfig);
        pivotConfig = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKA(0).withKV(2.25).withKG(0.17).withKP(0.1));
        pivot.getConfigurator().apply(pivotConfig); 
    }
    public enum PivotState{
        SPEAKER,
        AMP,
        STOW
    }
    public Command setState(PivotState state){
        return runOnce(() -> this.state = state);
    }
    @Override
    public void periodic() {    
        switch (state) {
            case SPEAKER:
                pivot.setControl(new MotionMagicVoltage(Units.degreesToRotations(getSpeakerAngle())));
                break;
            case AMP:
                pivot.setControl(new MotionMagicVoltage(Units.degreesToRotations(110)));
                break;
            case STOW:
                pivot.setControl(new MotionMagicVoltage(Units.degreesToRotations(5)));
                break;
            default:
                break;
        }
    }
    public double getSpeakerAngle(){
        
        return (target.calculateFromDistance(Math.sqrt((Math.pow((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? drivetrain.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED : drivetrain.getPose().getX(),2)) + (Math.pow(5.5 - drivetrain.getPose().getY(), 2))))).angle;
    }
    public Command ShootSpeaker(){
        return runOnce(() -> {
            var velocity = new VelocityVoltage((target.calculateFromDistance(Math.sqrt((Math.pow((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? drivetrain.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED : drivetrain.getPose().getX(),2)) + (Math.pow(5.5 - drivetrain.getPose().getY(), 2))))).velocity);
            velocity.Slot = 0;
            topFlywheel.setControl(velocity);
            bottomFlywheel.setControl(velocity);
            Commands.waitUntil(() -> topFlywheel.getVelocity().getValueAsDouble() == velocity.Velocity);
            topFeeder.setControl(velocity);
            bottomFeeder.setControl(velocity);
            Commands.waitSeconds(1);
            velocity.Velocity = 0;
        });
    }
    public Command ShootAmp(){
        return runOnce(() -> {
            var velocity = new VelocityVoltage(40);
            velocity.Slot = 0;
            topFlywheel.setControl(velocity);
            bottomFlywheel.setControl(velocity);
            Commands.waitUntil(() -> topFlywheel.getVelocity().getValueAsDouble() == velocity.Velocity);
            topFeeder.setControl(velocity);
            bottomFeeder.setControl(velocity);
            Commands.waitSeconds(1);
            velocity.Velocity = 0;
        });
    }




}

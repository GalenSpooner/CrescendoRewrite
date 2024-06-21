package frc.robot.Subsystems;

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
    static ShooterTarget target;
    boolean shooting = false;
    TalonFXConfiguration flywheelConfig;
    TalonFXConfiguration pivotConfig;
    static CommandSwerveDrivetrain drivetrain;
    MotionMagicVoltage pivotVoltage;
    VelocityVoltage flywheelVoltage;
    public Shooter(CommandSwerveDrivetrain drive){
        drivetrain = drive;
        topFlywheel = new RockinTalon(ShooterConstants.SHOOTER_TOPFLYWHEEL_ID, 50);
        bottomFlywheel = new RockinTalon(ShooterConstants.SHOOTER_BOTTOMFLYWHEEL_ID,50);
        topFeeder = new RockinTalon(ShooterConstants.SHOOTER_TOPFEEDER_ID,40);
        bottomFeeder = new RockinTalon(ShooterConstants.SHOOTER_BOTTOMFEEDER_ID,40);
        pivot = new RockinTalon(0, 40);
        encoder = new RockinCancoder(ShooterConstants.SHOOTER_PIVOT_ENCODER_ID, "rio", "Pivot Encoder",ShooterConstants.SHOOTER_PIVOT_ENCODER_OFFSET,true);
        encoder.attachToTalonFX(pivot,1,125);
        flywheelConfig = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKA(0.41).withKV(0.25).withKP(0.1));
        topFlywheel.getConfigurator().refresh(flywheelConfig);
        bottomFlywheel.getConfigurator().refresh(flywheelConfig);
        topFeeder.getConfigurator().refresh(flywheelConfig);
        bottomFeeder.getConfigurator().refresh(flywheelConfig);
        pivotConfig = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKA(0).withKV(2.25).withKG(0.17).withKP(0.1))
        .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(0.75).withMotionMagicCruiseVelocity(5));
        pivot.getConfigurator().refresh(pivotConfig); 
        flywheelVoltage = new VelocityVoltage(0);
        flywheelVoltage.Slot = 0;
        pivotVoltage = new MotionMagicVoltage(5);
        pivotVoltage.withSlot(0);
    }
    public enum PivotState{
        SPEAKER(getSpeakerAngle()),
        AMP(110),
        STOW(5);
        public double angle;
        private PivotState(double angle) {
            this.angle = angle;
        }
    }
    
    public Command setState(PivotState state){
        return runOnce(() -> this.state = state);
    }
    @Override
    public void periodic() {    
        if(!shooting){
            topFlywheel.setControl(flywheelVoltage.withVelocity(2));
            bottomFlywheel.setControl(flywheelVoltage.withVelocity(2));
        }
        pivot.setControl(pivotVoltage.withPosition(Units.degreesToRotations(state.angle)));
        SmartDashboard.putBoolean("Shooting", shooting);
        SmartDashboard.putString("Shooter Aim State", state.toString());
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
                flywheelVoltage.withVelocity(getSpeakerVelocity());
                topFlywheel.setControl(flywheelVoltage);
                bottomFlywheel.setControl(flywheelVoltage);
                }),
                new ParallelRaceGroup(Commands.waitUntil(() -> MathUtil.isNear(flywheelVoltage.Velocity,topFlywheel.getVelocity().getValueAsDouble(),50)), Commands.waitSeconds(3)),
            runOnce(() -> {
                topFeeder.setControl(flywheelVoltage);
                bottomFeeder.setControl(flywheelVoltage);
            }),
            Commands.waitSeconds(1),
            runOnce(() -> {
                flywheelVoltage.Velocity = 0;
                shooting = false;
            }));
    }
    public Command ShootAmp(){
        return new SequentialCommandGroup(
            runOnce(() -> {
                shooting = true;
                flywheelVoltage.withVelocity(500);
                topFlywheel.setControl(flywheelVoltage);
                bottomFlywheel.setControl(flywheelVoltage);
                }),
                new ParallelRaceGroup(Commands.waitUntil(() -> MathUtil.isNear(flywheelVoltage.Velocity,topFlywheel.getVelocity().getValueAsDouble(),50)), Commands.waitSeconds(3)),
            runOnce(() -> {
                topFeeder.setControl(flywheelVoltage);
                bottomFeeder.setControl(flywheelVoltage);
            }),
            Commands.waitSeconds(1),
            runOnce(() -> {
                flywheelVoltage.Velocity = 0;
                shooting = false;
            }));
    }
    public boolean isShooting(){
        return shooting;
    }
}

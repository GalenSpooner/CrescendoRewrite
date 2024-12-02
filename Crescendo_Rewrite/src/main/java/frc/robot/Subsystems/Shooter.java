package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import RockinLib.MotorControllers.RockinTalon;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    static ShooterTarget speakerTarget;
    static ShooterTarget passTarget;
    static ShooterTarget humanTarget;
    boolean shooting = false;
    TalonFXConfiguration flywheelConfig;
    TalonFXConfiguration pivotConfig;
    static CommandSwerveDrivetrain drivetrain;
    MotionMagicVoltage pivotVoltage;
    VelocityVoltage flywheelVoltage;
    BooleanSupplier intaking;
    SwerveRequest.FieldCentricFacingAngle angle;
    
    public Shooter(CommandSwerveDrivetrain drive, BooleanSupplier intaking, Object starterState){
        this.intaking = intaking;
        SmartDashboard.putNumber("Manual Angle", 15);
        SmartDashboard.putNumber("Manual Velocity", 15);
        this.state = (starterState.toString() == "STOW") ? PivotState.STOW : PivotState.MANUAL;
        speakerTarget = new ShooterTarget(null, null, null);
        passTarget = new ShooterTarget(null, null, null);
        humanTarget = new ShooterTarget(null, null, null);
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
        .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(0.75).withMotionMagicCruiseVelocity(5)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(100)).withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(70 * 100));
        pivot.getConfigurator().refresh(pivotConfig); 
        pivot.setPosition(0);
        flywheelVoltage = new VelocityVoltage(0);
        flywheelVoltage.Slot = 0;
        pivotVoltage = new MotionMagicVoltage(0);
        pivotVoltage.withSlot(0);
        angle = new SwerveRequest.FieldCentricFacingAngle();
    }
    public static enum PivotState{
        SPEAKER(getSpeakerAngle()),
        AMP(110),
        STOW(5),
        INTAKING(45),
        PASSING(getPassAngle()),
        HUMANPASS(getHumanAngle()),
        CLIMB(90),
        MANUAL(SmartDashboard.getNumber("Manual Angle", 15));
        public double angle;
        private PivotState(double angle) {
            this.angle = angle;
        }
    }
    //command for setting shooter behavior
    public Command setState(PivotState state){
        return runOnce(() -> {
            this.prevState = this.state;
            this.state = state;
            
        });
    }


    @Override
    public void periodic() {    
        // if the shooter isnt actively shooting, set it to neutral voltage of 2 volts
        if(!shooting){
            topFlywheel.setControl(flywheelVoltage.withVelocity(2));
            bottomFlywheel.setControl(flywheelVoltage.withVelocity(2));
        }
        //if the intake is intaking snap the shooter angle to specified angle for intaking
        if(intaking.getAsBoolean()){
            this.prevState = this.state;
            this.state = PivotState.INTAKING;
        }
        //once intake is done intaking set shooter back to what it was before
        if(this.getState() == PivotState.INTAKING && !intaking.getAsBoolean()){
            this.state = this.prevState;
            this.prevState = PivotState.INTAKING;
        }

        //set the angle of the pivot with a minumum angle of 0 and a maximum angle of 65 to prevent mechanical breaks
        pivot.setControl(pivotVoltage.withPosition(Units.degreesToRotations(MathUtil.clamp(state.angle, 0, 65))));
        //logging
        SmartDashboard.putBoolean("Shooting", shooting);
        SmartDashboard.putString("Shooter Aim State", state.toString());
        DogLog.log("Shooter aim state", state);
        SmartDashboard.putNumber("Shooter Angle", pivot.getRotorPosition().getValueAsDouble() / 3600);
        DogLog.log("Shooter Angle", pivot.getRotorPosition().getValueAsDouble() / 3600);
        SmartDashboard.putNumber("Flywheel velocity", topFlywheel.getVelocity().getValueAsDouble());
        DogLog.log("Flywheel velocity", topFlywheel.getVelocity().getValueAsDouble());
    }
    //getter methods from shootertargets
    public static double getSpeakerAngle(){
        
        return (speakerTarget.calculateFromDistance(Math.sqrt((Math.pow((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? drivetrain.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED : drivetrain.getPose().getX(),2)) + (Math.pow(5.5 - drivetrain.getPose().getY(), 2))))).angle;
    }
    public double getSpeakerVelocity(){
        return (speakerTarget.calculateFromDistance(Math.sqrt((Math.pow((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? drivetrain.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED : drivetrain.getPose().getX(),2)) + (Math.pow(5.5 - drivetrain.getPose().getY(), 2))))).velocity;
    }
    public static double getPassAngle(){
         return (passTarget.calculateFromDistance(Math.sqrt((Math.pow((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? drivetrain.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED : drivetrain.getPose().getX(),2)) + (Math.pow(5.5 - drivetrain.getPose().getY(), 2))))).angle;
    }
    public static double getHumanAngle(){
        return (humanTarget.calculateFromDistance(Math.sqrt((Math.pow((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? drivetrain.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED : drivetrain.getPose().getX(),2)) + (Math.pow(5.5 - drivetrain.getPose().getY(), 2))))).angle;
    }
    public static double getHumanVelocity(){
        return (humanTarget.calculateFromDistance(Math.sqrt((Math.pow((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? drivetrain.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED : drivetrain.getPose().getX(),2)) + (Math.pow(5.5 - drivetrain.getPose().getY(), 2))))).velocity;
    }
    public double getPassVelocity(){

    return (passTarget.calculateFromDistance(Math.sqrt((Math.pow((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? drivetrain.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED : drivetrain.getPose().getX(),2)) + (Math.pow(5.5 - drivetrain.getPose().getY(), 2))))).velocity;
    }
    public double getSpeakerHeading(){
        return Math.atan((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? (drivetrain.getPose().getX() / (5.5 - drivetrain.getPose().getY())) : (drivetrain.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED / (5.5 - drivetrain.getPose().getY())));
    }

    public Command ShootSpeaker(){ 
        return new SequentialCommandGroup(
            runOnce(() -> {
                shooting = true;
                //if on manual shooting mode set flywheels to the manually specified velocity (used for populating the tables in the shootertargets), otherwise set it to the needed vlocity for the shot
                flywheelVoltage.withVelocity((this.state == PivotState.MANUAL) ? SmartDashboard.getNumber("Manual Velocity", 75) : getSpeakerVelocity());
                topFlywheel.setControl(flywheelVoltage);
                bottomFlywheel.setControl(flywheelVoltage);
                }),
                //wait until flywheels are up to speed before feeding note into them
                new ParallelRaceGroup(Commands.waitUntil(() -> MathUtil.isNear(flywheelVoltage.Velocity,topFlywheel.getVelocity().getValueAsDouble(),5)), Commands.waitSeconds(1.5)),
            runOnce(() -> {
                topFeeder.setControl(flywheelVoltage);
                bottomFeeder.setControl(flywheelVoltage);
            }),
            // wait until note is shot
            Commands.waitSeconds(ShooterConstants.SHOOTER_TIME_TO_SCORE),
            runOnce(() -> {
                //stop shooting
                flywheelVoltage.Velocity = 0;
                shooting = false;
            }));
    }
    public Command PassToHuman(){
        return new SequentialCommandGroup(
            runOnce(() -> {
                shooting = true;
                flywheelVoltage.withVelocity(getHumanVelocity());
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
                flywheelVoltage.withVelocity(getPassVelocity());
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
    public Command manualHumanPass(){
         return new SequentialCommandGroup(
            runOnce(() -> {
                shooting = true;
                flywheelVoltage.withVelocity( ShooterConstants.SHOOTER_OUTREACH_SPEED_RPM/ 60);
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
    //so that you can score in speaker, amp, and pass with one button
    public Command Score(){
        DogLog.log("Shot", "Shot");
        switch(state){
            case SPEAKER:

                return new SequentialCommandGroup(ShootSpeaker()).beforeStarting(drivetrain.applyRequest(() -> angle.withTargetDirection(Rotation2d.fromDegrees(this.getSpeakerHeading())))).alongWith(new WaitCommand(0.25));
            case AMP:
                return ShootAmp();
            case HUMANPASS:
                return PassToHuman();
            case MANUAL:
                return manualHumanPass();
            default:
                return new SequentialCommandGroup(Pass()).beforeStarting(drivetrain.applyRequest(() -> angle.withTargetDirection(Rotation2d.fromDegrees(this.getSpeakerHeading())))).alongWith(new WaitCommand(0.25));
                
        }
        
    }
    //more getters
    public boolean isShooting(){
        return shooting;
    }
    public PivotState getState(){
        return this.state;
    }
}

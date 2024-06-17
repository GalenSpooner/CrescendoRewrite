// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import RockinLib.Control.RockinJoystick;
import RockinLib.LED.RockinBlinkin;
import RockinLib.LED.RockinBlinkin.BlinkinPattern;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Shooter.PivotState;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  CommandSwerveDrivetrain drivetrain;
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; 
  private double MaxAngularRate = 1.5 * Math.PI; 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  SendableChooser chooser = new SendableChooser<>();
  Shooter shooter;
  Intake intake;
  Climber climber;
  RockinBlinkin leds;
  Trigger LEDHolding;
  Trigger LEDIntaking;
  Trigger LEDShooting;
  RockinJoystick leftJoystick;
  RockinJoystick rightJoystick;
  CommandJoystick guitar;
  PathPlannerPath fournote1 = PathPlannerPath.fromPathFile("4note1");
  PathPlannerPath fournote2 = PathPlannerPath.fromPathFile("4note2");
  PathPlannerPath fournote3 = PathPlannerPath.fromPathFile("4note3");
  PathPlannerPath fournote4 = PathPlannerPath.fromPathFile("4note4");
  PathPlannerPath fournote5 = PathPlannerPath.fromPathFile("4note5");
  PathPlannerPath fivenote1 = PathPlannerPath.fromPathFile("5note1");
  PathPlannerPath fivenote2 = PathPlannerPath.fromPathFile("5note2");
  PathPlannerPath fivenotealt = PathPlannerPath.fromPathFile("5notealt");
  PathPlannerPath fivenotealt2 = PathPlannerPath.fromPathFile("5notealt2");




  public RobotContainer() {
    drivetrain = TunerConstants.DriveTrain;
    shooter = new Shooter(drivetrain);
    intake = new Intake();
    climber = new Climber();
    leftJoystick = new RockinJoystick(0);
    rightJoystick = new RockinJoystick(1);
    guitar = new CommandJoystick(2);
    SmartDashboard.putData(chooser);
    chooser.addOption("OnePiece", 0);
    chooser.addOption("5piece" , 1);
    configureBindings();
  }

  private void configureBindings() {
    //config drivetrain
    drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() -> drive.withVelocityX(-leftJoystick.getY() * MaxSpeed) 
        .withVelocityY(-leftJoystick.getX() * MaxSpeed)
        .withRotationalRate(-rightJoystick.getX() * MaxAngularRate) 
    ));

    //config LED actions
    LEDHolding = new Trigger(() -> intake.isHolding() && !shooter.isShooting());
    LEDIntaking = new Trigger(()-> intake.isIntaking() && !shooter.isShooting());
    LEDShooting = new Trigger(() -> shooter.isShooting());
    LEDHolding.onTrue(leds.setBlinkinPattern(BlinkinPattern.AQUA));
    LEDIntaking.onTrue(leds.setBlinkinPattern(BlinkinPattern.DARK_RED));
    LEDShooting.onTrue(leds.setBlinkinPattern(BlinkinPattern.GREEN));

    //config joystick button presses - left joystick primarily intake, right joystick primarily shooter
    leftJoystick.trigger().and(() -> intake.getState() != IntakeState.HOLDING).onTrue(intake.setState(IntakeState.INTAKING));
    leftJoystick.trigger().onFalse(intake.setState(IntakeState.IDLE));
    leftJoystick.ThumbButton().onTrue(intake.setState(IntakeState.OUTTAKING));
    rightJoystick.UpperLeft().onTrue(shooter.setState(PivotState.SPEAKER));
    rightJoystick.UpperRight().onTrue(shooter.setState(PivotState.AMP));
    rightJoystick.trigger().onTrue(shooter.ShootSpeaker());
    rightJoystick.ThumbButton().onTrue(shooter.ShootAmp());
    
    //config guitar button presses (climb)
    guitar.button(0).whileTrue(climber.climbersUp());
    guitar.button(1).whileTrue(climber.climbersDown());
  }
  private Command chooseAuto(){
    switch ((int) chooser.getSelected()) {
      case 0:
        return new SequentialCommandGroup(
              shooter.setState(PivotState.SPEAKER),
              shooter.ShootSpeaker()
        );
      case 1:
        return new SequentialCommandGroup(

          shooter.setState(PivotState.SPEAKER),
          shooter.ShootSpeaker(),
          intake.setState(IntakeState.INTAKING),

          drivetrain.runPathplannerPathFile(fournote1),
          drivetrain.runPathplannerPathFile(fournote2),

          shooter.ShootSpeaker(),
          intake.setState(IntakeState.INTAKING),
          
          drivetrain.runPathplannerPathFile(fournote3),

          shooter.ShootSpeaker(),
          intake.setState(IntakeState.INTAKING),

          drivetrain.runPathplannerPathFile(fournote4),
          drivetrain.runPathplannerPathFile(fournote5),

          shooter.ShootSpeaker(),
          intake.setState(IntakeState.INTAKING),

          drivetrain.runPathplannerPathFile(fivenote1),

          //logic for if robot does not pick up a note (it will go to the next note)
          ((intake.getState() == IntakeState.HOLDING)?new InstantCommand(() ->{

            drivetrain.runPathplannerPathFile(fivenote2);
            shooter.ShootSpeaker();

          }):new InstantCommand(() -> {

            drivetrain.runPathplannerPathFile(fivenotealt);
            drivetrain.runPathplannerPathFile(fivenotealt2);
            shooter.ShootSpeaker();

          }))

        );
    
      default:
        return null;
    }
  }
  public Command getAutonomousCommand() {
    return chooseAuto();
  }
}

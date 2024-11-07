// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import RockinLib.Control.RockinGuitar;
import RockinLib.Control.RockinJoystick;
import RockinLib.LED.RockinBlinkin;
import RockinLib.LED.RockinBlinkin.BlinkinPattern;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Shooter.PivotState;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  
  CommandSwerveDrivetrain drivetrain;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; 
  private double MaxAngularRate = 1.5 * Math.PI; 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  SendableChooser chooser = new SendableChooser();
  SendableChooser modeChooser = new SendableChooser();
  Shooter shooter;
  Intake intake;
  Climber climber;
  RockinBlinkin leds;
  Trigger LEDHolding;
  Trigger LEDIntaking;
  Trigger LEDShooting;
  RockinJoystick leftJoystick;
  RockinJoystick rightJoystick;
  RockinGuitar guitarHero;
  PathPlannerPath fournote1 = PathPlannerPath.fromPathFile("4note1");
  PathPlannerPath fournote2 = PathPlannerPath.fromPathFile("4note2");
  PathPlannerPath fournote3 = PathPlannerPath.fromPathFile("4note3");
  PathPlannerPath fournote4 = PathPlannerPath.fromPathFile("4note4");
  PathPlannerPath fournote5 = PathPlannerPath.fromPathFile("4note5");
  PathPlannerPath fivenote1 = PathPlannerPath.fromPathFile("5note1");
  PathPlannerPath fivenote2 = PathPlannerPath.fromPathFile("5note2");
  PathPlannerPath fivenotealt = PathPlannerPath.fromPathFile("5notealt");
  PathPlannerPath fivenotealt2 = PathPlannerPath.fromPathFile("5notealt2");
  PathPlannerPath sixnote1 = PathPlannerPath.fromPathFile("6note1");
  PathPlannerPath sixnote2 = PathPlannerPath.fromPathFile("6note2");
  




  public RobotContainer() {
    DogLog.setPdh(new PowerDistribution(63, ModuleType.kRev));
    DogLog.setOptions(new DogLogOptions().withCaptureDs(false).withCaptureNt(false).withLogExtras(true).withNtPublish(true));
    drivetrain = TunerConstants.DriveTrain;
    intake = new Intake();
    
    //climber = new Climber();
    leds = new RockinBlinkin(3, "blinkin");
    leftJoystick = new RockinJoystick(0);
    rightJoystick = new RockinJoystick(1);
    guitarHero = new RockinGuitar(2);
    chooser.addOption("OnePiece", 0);
    chooser.addOption("5piece" , 1);
    modeChooser.addOption("Competition", PivotState.STOW);
    modeChooser.addOption("Outreach", PivotState.MANUAL);
    shooter = new Shooter(drivetrain, intake::isIntaking,modeChooser.getSelected());
    
    SmartDashboard.putData(chooser);
    
    
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
    LEDIntaking = new Trigger(()-> !intake.isHolding()  && !shooter.isShooting() && intake.isIntaking());
    LEDShooting = new Trigger(() -> shooter.isShooting());
    LEDHolding.onTrue(leds.setBlinkinPattern(BlinkinPattern.GREEN));
    LEDIntaking.onTrue(leds.setBlinkinPattern(BlinkinPattern.DARK_RED));
    LEDShooting.onTrue(leds.setBlinkinPattern(BlinkinPattern.AQUA));

    //config joystick button presses - left joystick primarily intake, right joystick primarily shooter
    leftJoystick.trigger().and(() -> intake.getState() != IntakeState.HOLDING).onTrue(intake.setState(IntakeState.INTAKING));
    leftJoystick.trigger().onFalse(intake.setState(IntakeState.IDLE));
    leftJoystick.ThumbButton().onTrue(intake.setState(IntakeState.OUTTAKING));
    rightJoystick.DpadUp().onTrue(shooter.setState(PivotState.SPEAKER));
    rightJoystick.DpadLeft().or(rightJoystick.DpadRight()).onTrue(shooter.setState(PivotState.AMP));
    rightJoystick.DpadDown().onTrue((modeChooser.getSelected() == "Outreach") ? shooter.setState(PivotState.HUMANPASS) : shooter.setState(PivotState.PASSING));
    rightJoystick.DpadNeutral().onTrue(shooter.setState(PivotState.STOW));
    rightJoystick.trigger().onTrue(shooter.Score());
    
    //config guitar button presses (climb)
    // guitarHero.green().whileTrue(climber.climbersUp());
    // guitarHero.red().whileTrue(climber.climbersDown());
  }
  // returns auto based of of selected autonomous in shuffleboard
  private Command chooseAuto(){
    switch ((int) chooser.getSelected()) {
      case 0:
        return new SequentialCommandGroup(
              shooter.setState(PivotState.SPEAKER),
              shooter.ShootSpeaker()
        );
      case 1:
       return Autos.fiveNoteAdaptable(drivetrain, shooter, intake, fournote1, fournote2, fournote3, fournote4, fournote5, fivenote1, fivenote2, fivenotealt, fivenotealt2);
      case 2:
          return Autos.sixNote(drivetrain, shooter, intake, fournote1, fournote2, fournote3, fournote4, fournote5, fivenote1, fivenote2, fivenotealt, fivenotealt2,sixnote1,sixnote2);
    
      default:
        return null;
    }
  }
  public Command getAutonomousCommand() {
    return chooseAuto();
  }
}

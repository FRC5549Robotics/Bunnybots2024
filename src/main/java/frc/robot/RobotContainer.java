// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.Auton.DumpnDrive;
import frc.robot.subsystems.DrivetrainSubsystem;
//import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shintake;
// import frc.robot.subsystems.Pivot.PivotTarget;
import frc.robot.commands.Auton.DumpnDrive;
import frc.robot.commands.DriveCommand;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_controller = new CommandXboxController(Constants.DRIVE_CONTROLLER);
  private final CommandXboxController m_controller2 = new CommandXboxController(Constants.OPERATOR_CONTROLLER);


  PathPlannerPath Simple = PathPlannerPath.fromPathFile("ThrowItBackBunny");
  

  // The robot's subsystems and commands are defined here...
  private final AHRS m_ahrs = new AHRS();
  public final DrivetrainSubsystem m_drive = new DrivetrainSubsystem(m_ahrs);
  // private final Pivot m_pivot = new Pivot(m_controller2);
  private final Shintake m_shintake = new Shintake();
  DrivetrainSubsystem m_DrivetrainSubsystem = new DrivetrainSubsystem(m_ahrs);
  // private final Limelight m_limelight = new Limelight(m_controller, m_drive);//Find Feedforward Constants );

  JoystickButton resetNavXButton = new JoystickButton(m_controller.getHID(), Constants.RESET_NAVX_BUTTON);
  JoystickButton shootPivotButton = new JoystickButton(m_controller.getHID(), Constants.SHOOT_PIVOT_BUTTON);
  JoystickButton intakePivotButton = new JoystickButton(m_controller.getHID(), Constants.INTAKE_PIVOT_BUTTON);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
      m_drive.setDefaultCommand(new DriveCommand(m_drive, m_controller));
      resetNavXButton.onTrue(new InstantCommand(m_drive::zeroGyroscope));

      m_controller2.leftTrigger(0.1).onTrue(new InstantCommand(m_shintake::shintake_intake));
      m_controller2.rightTrigger(0.1).onTrue(new InstantCommand(m_shintake::shintake_shoot));
      m_controller2.leftTrigger(0.1).negate().and(m_controller2.rightTrigger(0.1).negate()).onTrue(new InstantCommand(m_shintake::off));


      // shootPivotButton.whileTrue(new PivotIntake(m_pivot, PivotTarget.Shoot));
      // intakePivotButton.whileTrue(new PivotIntake(m_pivot, PivotTarget.Intake));
      // shootPivotButton.or(intakePivotButton).onFalse(new InstantCommand(m_pivot::off));
      // m_controller2.axisGreaterThan(Constants.PIVOT_JOYSTICK, Constants.PIVOT_DEADBAND).or(m_controller2.axisLessThan(Constants.PIVOT_JOYSTICK, -Constants.PIVOT_DEADBAND)).onTrue(new PivotAnalog(m_pivot, m_controller2)).onFalse(new InstantCommand(m_pivot::off));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    m_DrivetrainSubsystem.setAutonPose(Simple);
    return new DumpnDrive(m_shintake, Simple);
  }
}

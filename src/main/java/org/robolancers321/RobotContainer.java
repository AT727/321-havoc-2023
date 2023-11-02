/* (C) Robolancers 2024 */
package org.robolancers321;

import static org.robolancers321.Constants.OperatorConstants.*;
import static org.robolancers321.Constants.Swerve.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.robolancers321.commands.autos.Autos;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.subsystems.arm.commands.RunArm;
import org.robolancers321.subsystems.arm.commands.ManualMoveAnchor;
import org.robolancers321.subsystems.arm.commands.ManualMoveFloating;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.intake.commands.RunIntake;
import org.robolancers321.subsystems.intake.commands.RunOuttake;
import org.robolancers321.subsystems.swerve.Swerve;
import org.robolancers321.subsystems.swerve.SwerveModule;

public class RobotContainer {
  private final Field2d field = new Field2d();
  private final CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController manipulator =
      new CommandXboxController(Constants.OperatorConstants.kManipulatorControllerPort);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final Arm arm = new Arm();

  private final Swerve swerve =
      new Swerve(
          new SwerveModule(frontLeft),
          new SwerveModule(frontRight),
          new SwerveModule(backLeft),
          new SwerveModule(backRight),
          gyro,
          field);

  private final Intake intake = new Intake();
  private final Autos autoPicker = new Autos(swerve, arm, intake);

  public RobotContainer() {
    // swerve.setDefaultCommand(swerve.drive(this::getThrottle, this::getStrafe, this::getTurn,
    // true));

    this.arm.setDefaultCommand(new RunArm(arm));

    configureBindings();
  }

  private void configureBindings() {


    //manipulator arm
    manipulator.b().onTrue(arm.moveArmTogether(Constants.RawArmSetpoints.CONTRACT));
    manipulator.x().onTrue(arm.moveArmSeparate(Constants.RawArmSetpoints.MID));
    manipulator.y().onTrue(arm.moveArmSeparate(Constants.RawArmSetpoints.HIGH));
    manipulator.a().onTrue(arm.moveArmSeparate(Constants.RawArmSetpoints.SHELFCONE)); //also ground
    manipulator.povUp().onTrue(arm.moveArmSeparate(Constants.RawArmSetpoints.SHELFCUBE));

    //manipulator manual arm 
    manipulator.rightTrigger().whileTrue(new ManualMoveAnchor(arm, false));
    manipulator.rightBumper().whileTrue(new ManualMoveFloating(arm, false));
    
    manipulator.leftTrigger().whileTrue(new ManualMoveAnchor(arm, true));
    manipulator.leftBumper().whileTrue(new ManualMoveFloating(arm, true));

    //manipulator intake
    Trigger intakeSlow = new Trigger(() -> manipulator.getLeftY() > 0.2);
    Trigger outtakeSlow = new Trigger(() -> manipulator.getLeftY() < -0.2);
    Trigger intakeFast = new Trigger(() -> manipulator.getRightY() > 0.2);
    Trigger outtakeFast = new Trigger(() -> manipulator.getRightY() < -0.2);

    intakeSlow.whileTrue(new RunIntake(intake, Constants.Intake.kLowVelocity));
    outtakeSlow.whileTrue(new RunOuttake(intake, Constants.Intake.kLowVelocity));

    intakeFast.whileTrue(new RunIntake(intake, Constants.Intake.kMaxVelocity));
    outtakeFast.whileTrue(new RunOuttake(intake, Constants.Intake.kMaxVelocity));


    

    
  };

  public Command getAutonomousCommand() {
    return autoPicker.getAutoChooser().getSelected();
  }

  private double getThrottle() {
    return kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(driver.getLeftY(), kJoystickDeadband);
  }

  private double getStrafe() {
    return kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(driver.getLeftX(), kJoystickDeadband);
  }

  private double getTurn() {
    return kMaxOmegaRadiansPerSecond
        * MathUtil.applyDeadband(driver.getRightX(), kJoystickDeadband);
  }
}

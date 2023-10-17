/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.robolancers321.Constants.Arm.ArmSetpoints;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.subsystems.arm.commands.MoveToSetpoint;
import org.robolancers321.subsystems.arm.commands.RunArm;

public class RobotContainer {
  private Arm arm = new Arm();

  private final CommandXboxController controller =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    this.arm.setDefaultCommand(new RunArm(arm));

    controller.y().whileTrue(new MoveToSetpoint(arm, ArmSetpoints.TEST));

    configureBindings();

    // try {
    //   String versionData = new VersionLoader().getVersionData().toString();
    //   System.out.println(versionData);
    //   SmartDashboard.putString("VERSION_DATA", versionData);

    // } catch (FileNotFoundException ex) {
    //   System.out.println("VERSIONING FILE NOT FOUND");
    //   SmartDashboard.putString("VERSION_DATA", "VERSIONING FILE NOT FOUND");
    // }
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}

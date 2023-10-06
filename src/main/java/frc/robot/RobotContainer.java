/* (C) Robolancers 2024 */
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.MoveToSetpoint;
import frc.robot.subsystems.arm.commands.RunArm;
import frc.robot.util.Controller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private Arm arm = new Arm();
  private Controller controller = new Controller(0);

  public RobotContainer() {
    this.arm.setDefaultCommand(new RunArm(arm));
    configureBindings();
  }

  private void configureBindings() {
    Controller.onPress(controller.A, new MoveToSetpoint(arm));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

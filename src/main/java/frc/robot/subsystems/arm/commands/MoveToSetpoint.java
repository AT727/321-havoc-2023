/* (C) Robolancers 2024 */
package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.InverseArmKinematics;
import frc.robot.util.RelativePlane;

public class MoveToSetpoint extends CommandBase {

  private double anchorPosSetpoint;
  private double floatingPosSetpoint;
  Arm arm;

  public MoveToSetpoint(Arm arm
      // double setpoint
      ) {
    // this.anchorPosSetpoint = setpoint;
    // this.floatingPosSetpoint = floatingSetpoint.position;
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    // arm.periodicIO.anchorPosSetpoint = anchorPosSetpoint;
    InverseArmKinematics testSetpoint =
        new InverseArmKinematics(new RelativePlane("test", 0), 22, 35); // inches
    InverseArmKinematics.Angles angles = testSetpoint.getAngles();
    arm.periodicIO.anchorPosSetpoint = angles.getAnchorAngle();
    arm.periodicIO.floatingPosSetpoint = angles.getFloatingAngle();
  }
}

/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.Constants;
import org.robolancers321.Constants.Arm.ArmSetpoints;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.util.MathUtils;

public class MoveToSetpoint extends CommandBase {
  private Arm arm;

  private double anchorPosSetpoint;
  private double floatingPosSetpoint;

  public MoveToSetpoint(Arm arm, ArmSetpoints setpoint) {
    this.arm = arm;

    this.anchorPosSetpoint = setpoint.getAnchor();
    this.floatingPosSetpoint = setpoint.getFloating();
  }

  @Override
  public void initialize() {
    // arm.anchorSetpoint = anchorPosSetpoint;
    // arm.floatingSetpoint = floatingPosSetpoint;


    SmartDashboard.putNumber("anchorIK", anchorPosSetpoint);
    SmartDashboard.putNumber("floatingIK", floatingPosSetpoint);

    

   
  }

  @Override
  public boolean isFinished() {
    return (MathUtils.epsilonEquals(
            anchorPosSetpoint, arm.getAnchorAngle(), Constants.Arm.Anchor.kTolerance)
        && MathUtils.epsilonEquals(
            floatingPosSetpoint, arm.getFloatingAngle(), Constants.Arm.Floating.kTolerance));
  }
}

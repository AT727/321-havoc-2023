/* (C) Robolancers 2024 */
package frc.robot.subsystems.arm.commands;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

public class RunArm extends CommandBase {

  Arm arm;

  public RunArm(Arm arm) {
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    // PID
    // // cal FF
    arm.periodicIO.anchorFF =
        Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD.calculate(
            Math.toRadians(arm.periodicIO.anchorPosSetpoint), 0);
    arm.periodicIO.floatingFF =
        Constants.Arm.Floating.FF.FLOATING_FEEDFORWARD.calculate(
            Math.toRadians(arm.periodicIO.floatingPosSetpoint), 0);

    // set FF and setpoint
    arm.anchorPIDController.setReference(
        arm.periodicIO.anchorPosSetpoint,
        ControlType.kPosition,
        Constants.Arm.Anchor.PID.kSlot,
        arm.periodicIO.anchorFF,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
    arm.floatingPIDController.setReference(
        arm.periodicIO.floatingPosSetpoint,
        ControlType.kPosition,
        Constants.Arm.Floating.PID.kSlot,
        arm.periodicIO.floatingFF,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
  }
}

/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.util.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.robolancers321.Constants;

public class Arm extends SubsystemBase {
  private CANSparkMax anchorMotor;
  private CANSparkMax floatingMotor;

  private RelativeEncoder anchorEncoder;
  private RelativeEncoder floatingEncoder;

  private SparkMaxPIDController anchorPIDController;
  private SparkMaxPIDController floatingPIDController;

  // TODO: potentially should use explicit getters and setters
  public double anchorSetpoint = Constants.Arm.Anchor.kZeroPosition;
  public double floatingSetpoint = Constants.Arm.Floating.kZeroPosition;

  public TrapezoidProfile.State anchorState =
      new TrapezoidProfile.State(Constants.Arm.Anchor.kZeroPosition, 0);
  public TrapezoidProfile.State floatingState =
      new TrapezoidProfile.State(Constants.Arm.Floating.kZeroPosition, 0);

  public Arm() {
    this.anchorMotor = new CANSparkMax(Constants.Arm.Anchor.kAnchorPort, MotorType.kBrushless);
    this.anchorEncoder = anchorMotor.getEncoder();
    this.anchorPIDController = this.anchorMotor.getPIDController();

    this.floatingMotor =
        new CANSparkMax(Constants.Arm.Floating.kFloatingPort, MotorType.kBrushless);
    this.floatingEncoder = floatingMotor.getEncoder();
    this.floatingPIDController = this.floatingMotor.getPIDController();

    configureMotors();
    configureEncoders();
    configureControllers();

    initTuneControllers();
  }

  private void configureMotors() {
    anchorMotor.setInverted(Constants.Arm.Anchor.kInverted);
    anchorMotor.setIdleMode(IdleMode.kBrake);
    anchorMotor.setSmartCurrentLimit(Constants.Arm.Anchor.kCurrentLimit);
    anchorMotor.enableVoltageCompensation(12.0);
    anchorMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.Anchor.kMinAngle);
    anchorMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.Anchor.kMaxAngle);
    anchorMotor.enableSoftLimit(SoftLimitDirection.kReverse, Constants.Arm.Anchor.kEnableSoftLimit);
    anchorMotor.enableSoftLimit(SoftLimitDirection.kForward, Constants.Arm.Anchor.kEnableSoftLimit);

    floatingMotor.setInverted(Constants.Arm.Floating.kInverted);
    floatingMotor.setIdleMode(IdleMode.kBrake);
    floatingMotor.setSmartCurrentLimit(Constants.Arm.Floating.kCurrentLimit);
    floatingMotor.enableVoltageCompensation(12.0);
    floatingMotor.setSoftLimit(
        SoftLimitDirection.kReverse, (float) Constants.Arm.Floating.kMinAngle);
    floatingMotor.setSoftLimit(
        SoftLimitDirection.kForward, (float) Constants.Arm.Floating.kMaxAngle);
    floatingMotor.enableSoftLimit(
        SoftLimitDirection.kReverse, Constants.Arm.Floating.kEnableSoftLimit);
    floatingMotor.enableSoftLimit(
        SoftLimitDirection.kForward, Constants.Arm.Floating.kEnableSoftLimit);
  }

  private void configureEncoders() {
    anchorEncoder.setPositionConversionFactor(Constants.Arm.Anchor.Conversions.kDegPerRot);
    anchorEncoder.setPosition(Constants.Arm.Anchor.kZeroPosition);

    floatingEncoder.setPositionConversionFactor(Constants.Arm.Floating.Conversions.kDegPerRot);
    floatingEncoder.setPosition(Constants.Arm.Floating.kZeroPosition);
  }

  private void configureControllers() {
    anchorPIDController.setP(Constants.Arm.Anchor.PID.kP);
    anchorPIDController.setI(Constants.Arm.Anchor.PID.kI);
    anchorPIDController.setD(Constants.Arm.Anchor.PID.kD);
    anchorPIDController.setOutputRange(
        Constants.Arm.Anchor.kMinOutput, Constants.Arm.Anchor.kMaxOutput);

    floatingPIDController.setP(Constants.Arm.Floating.PID.kP);
    floatingPIDController.setI(Constants.Arm.Floating.PID.kI);
    floatingPIDController.setD(Constants.Arm.Floating.PID.kD);
    floatingPIDController.setOutputRange(
        Constants.Arm.Floating.kMinOutput, Constants.Arm.Floating.kMaxOutput);
  }

  public double getAnchorAngle() {
    return anchorEncoder.getPosition();
  }

  public double getFloatingAngle() {
    return floatingEncoder.getPosition();
  }

  public double getAnchorVelocity() {
    return anchorEncoder.getVelocity();
  }

  public double getFloatingVelocity() {
    return floatingEncoder.getVelocity();
  }

  public void setAnchorSpeed(double speed) {
    anchorMotor.set(speed);
  }

  public void setFloatingSpeed(double speed) {
    floatingMotor.set(speed);
  }

  public void setAnchorState(double position, double velocity) {
    this.anchorState = new TrapezoidProfile.State(position, velocity);
  }

  public TrapezoidProfile.State getFloatingState() {
    return this.floatingState = new TrapezoidProfile.State(this.getFloatingAngle(), 0);
  }

  public void setFloatingState(double position, double velocity) {
    this.floatingState = new TrapezoidProfile.State(position, velocity);
  }

  public void setAnchorControllerReference(double reference, double ff) {
    anchorPIDController.setReference(
        reference, ControlType.kPosition, Constants.Arm.Anchor.PID.kSlot, ff, ArbFFUnits.kVoltage);
  }

  public void setFloatingControllerReference(double reference, double ff) {
    floatingPIDController.setReference(
        reference,
        ControlType.kPosition,
        Constants.Arm.Floating.PID.kSlot,
        ff,
        ArbFFUnits.kVoltage);
  }



  private void initTuneControllers() {
    SmartDashboard.putNumber(
        "anchorKP", SmartDashboard.getNumber("anchorKP", Constants.Arm.Anchor.PID.kP));
    SmartDashboard.putNumber(
        "anchorKI", SmartDashboard.getNumber("anchorKI", Constants.Arm.Anchor.PID.kI));
    SmartDashboard.putNumber(
        "anchorKD", SmartDashboard.getNumber("anchorKD", Constants.Arm.Anchor.PID.kD));
    SmartDashboard.putNumber(
        "anchorKG", SmartDashboard.getNumber("anchorKG", Constants.Arm.Anchor.FF.kG));
    SmartDashboard.putNumber(
        "anchorKS", SmartDashboard.getNumber("anchorKS", Constants.Arm.Anchor.FF.kS));

    SmartDashboard.putNumber(
        "floatingKP", SmartDashboard.getNumber("floatingKP", Constants.Arm.Floating.PID.kP));
    SmartDashboard.putNumber(
        "floatingKI", SmartDashboard.getNumber("floatingKI", Constants.Arm.Floating.PID.kI));
    SmartDashboard.putNumber(
        "floatingKD", SmartDashboard.getNumber("floatingKD", Constants.Arm.Floating.PID.kD));
    
    SmartDashboard.putNumber("anchorSetpoint", anchorSetpoint);
    SmartDashboard.putNumber("floatingSetpoint", floatingSetpoint);
  }

  private void tuneControllers() {
    double anchorSetpoint = SmartDashboard.getEntry("anchorSetpoint").getDouble(0);
    double floatingSetpoint = SmartDashboard.getEntry("floatingSetpoint").getDouble(0);

    double anchorKP = SmartDashboard.getEntry("anchorKP").getDouble(0);
    double anchorKI = SmartDashboard.getEntry("anchorKI").getDouble(0);
    double anchorKD = SmartDashboard.getEntry("anchorKD").getDouble(0);
    double anchorKG = SmartDashboard.getEntry("anchorKG").getDouble(0);
    double anchorKS = SmartDashboard.getEntry("anchorKS").getDouble(0);

    double floatingKP = SmartDashboard.getEntry("floatingKP").getDouble(0);
    double floatingKI = SmartDashboard.getEntry("floatingKI").getDouble(0);
    double floatingKD = SmartDashboard.getEntry("floatingKD").getDouble(0);

    SmartDashboard.putNumber("currentAnchorPos", this.getAnchorAngle());
    SmartDashboard.putNumber("anchorOutput", this.anchorMotor.getAppliedOutput());
    SmartDashboard.putNumber("currentFloatingPos", this.getFloatingAngle());
    SmartDashboard.putNumber("floatingOutput", this.floatingMotor.getAppliedOutput());

    // this.anchorPIDController.setP(anchorKP);
    // this.anchorPIDController.setI(anchorKI);
    // this.anchorPIDController.setD(anchorKD);

    this.floatingPIDController.setP(MathUtil.clamp(floatingKP, 0.0, 3.0));
    this.floatingPIDController.setI(floatingKI);
    this.floatingPIDController.setD(floatingKD);

    this.anchorSetpoint = MathUtil.clamp(anchorSetpoint, 16, 95);
    this.floatingSetpoint = MathUtil.clamp(floatingSetpoint, 22, 180);

    Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD = new ArmFeedforward(anchorKS, anchorKG, 0);


    // double setpoint = SmartDashboard.getEntry("setpointPos").getDouble(0);
    // double floatingKP = SmartDashboard.getEntry("floating KP").getDouble(0);
    // double floatingKI = SmartDashboard.getEntry("floatingKI").getDouble(0);
    // double floatingKD = SmartDashboard.getEntry("floatingKD").getDouble(0);
    // double floatingKS = SmartDashboard.getEntry("floatingKS").getDouble(0);

    // SmartDashboard.putNumber("Pos", this.getFloatingAngle());
    // SmartDashboard.putNumber("Output", this.floatingMotor.getAppliedOutput());

    // Constants.Arm.Floating.FF.kS = floatingKS;
  }


  @Override
  public void periodic() {
    tuneControllers();
  }
}

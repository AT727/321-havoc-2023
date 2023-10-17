/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
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

    // this.periodicIO = new PeriodicIO();
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

    // only need for Motion Profile
    anchorEncoder.setVelocityConversionFactor(Constants.Arm.Anchor.Conversions.kDegPerRotVEL);
    // floatingEncoder.setVelocityConversionFactor(Constants.Arm.Floating.Conversions.kDistPerRot);
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

  public TrapezoidProfile.State getAnchorState() {
    return this.anchorState = new TrapezoidProfile.State(this.getAnchorAngle(), 0);
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

  private void initTuneControllers() {
    SmartDashboard.putNumber(
        "custom arm kG", SmartDashboard.getNumber("custom arm kG", Constants.Arm.kG));
    SmartDashboard.putNumber(
        "anchorSetpoint", SmartDashboard.getNumber("anchorSetpointPos", getAnchorAngle()));
    SmartDashboard.putNumber(
        "floatingSetpoint", SmartDashboard.getNumber("floatingSetpointPos", getFloatingAngle()));
    SmartDashboard.putNumber(
        "anchorMaxVel", SmartDashboard.getNumber("anchorMaxVel", Constants.Arm.Anchor.MP.maxVel));
    SmartDashboard.putNumber(
        "anchorMaxAccel",
        SmartDashboard.getNumber("anchorMaxAccel", Constants.Arm.Anchor.MP.maxAccel));

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
    SmartDashboard.putNumber(
        "floatingKS", SmartDashboard.getNumber("flotingKS", Constants.Arm.Anchor.FF.kS));
  }

  private void tuneControllers() {
    Constants.Arm.kG = SmartDashboard.getNumber("custom arm kG", 0.0);

    double anchorSetpoint = SmartDashboard.getEntry("anchorSetpoint").getDouble(0);
    double anchorMaxVel = SmartDashboard.getEntry("anchorMaxVel").getDouble(0);
    double anchorMaxAccel = SmartDashboard.getEntry("anchorMaxAccel").getDouble(0);

    double anchorKP = SmartDashboard.getEntry("anchorKP").getDouble(0);
    double anchorKI = SmartDashboard.getEntry("anchorKI").getDouble(0);
    double anchorKD = SmartDashboard.getEntry("anchorKD").getDouble(0);
    double anchorKG = SmartDashboard.getEntry("anchorKG").getDouble(0);
    double anchorKS = SmartDashboard.getEntry("anchorKS").getDouble(0);

    SmartDashboard.putNumber("anchorAngle", this.getAnchorAngle());
    SmartDashboard.putNumber("Output", this.anchorMotor.getAppliedOutput());
    SmartDashboard.putNumber("anchorVelMP", this.anchorState.velocity);
    SmartDashboard.putNumber("anchorVelREAL", this.getAnchorVelocity());

    this.anchorPIDController.setP(anchorKP);
    this.anchorPIDController.setI(anchorKI);
    this.anchorPIDController.setD(anchorKD);

    // periodicIO.anchorPosSetpoint = setpoint;
    this.anchorSetpoint =
        MathUtil.clamp(
            anchorSetpoint, Constants.Arm.Anchor.kMinAngle, Constants.Arm.Anchor.kMaxAngle);

    Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD = new ArmFeedforward(0, anchorKG, 0);

    Constants.Arm.Anchor.MP.ANCHOR_CONSTRAINTS = new TrapezoidProfile.Constraints(anchorMaxVel, anchorMaxAccel);

    double floatingSetpoint = SmartDashboard.getEntry("floatingSetpoint").getDouble(0);
    double floatingKP = SmartDashboard.getEntry("floatingKP").getDouble(0);
    double floatingKI = SmartDashboard.getEntry("floatingKI").getDouble(0);
    double floatingKD = SmartDashboard.getEntry("floatingKD").getDouble(0);

    SmartDashboard.putNumber("floatingAngle", getFloatingAngle());
    // SmartDashboard.putNumber("Output", this.floatingMotor.getAppliedOutput());

    this.floatingPIDController.setP(floatingKP);
    this.floatingPIDController.setI(floatingKI);
    this.floatingPIDController.setD(floatingKD);
    this.floatingSetpoint = MathUtil.clamp(floatingSetpoint, Constants.Arm.Floating.kMinAngle, Constants.Arm.Floating.kMaxAngle);
  }

  public double calculateCustomAnchorFeedforward() {
    double alpha = this.getAnchorAngle() * Math.PI / 180.0;
    double beta = this.getFloatingAngle() * Math.PI / 180.0;

    return Constants.Arm.kG
        * (Constants.Arm.Anchor.kAnchorLength
                * Math.cos(alpha)
                * (0.5 + Constants.Arm.kFloatingToAnchorMassRatio)
            + 0.5
                * Constants.Arm.kFloatingToAnchorMassRatio
                * Constants.Arm.Floating.kFloatingLength
                * Math.cos(alpha - beta));
  }

  public double calculateCustomFloatingFeedforward() {
    double alpha = this.getAnchorAngle() * Math.PI / 180.0;
    double beta = this.getFloatingAngle() * Math.PI / 180.0;

    return Constants.Arm.kG
        * (0.5
            * Constants.Arm.kFloatingToAnchorMassRatio
            * Constants.Arm.Floating.kFloatingLength
            * Math.cos(alpha - beta));
  }

  @Override
  public void periodic() {
    tuneControllers();
  }
}

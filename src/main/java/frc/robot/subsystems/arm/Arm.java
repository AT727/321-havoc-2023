/* (C) Robolancers 2024 */
package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public CANSparkMax anchorMotor;
  public CANSparkMax floatingMotor;

  public RelativeEncoder anchorEncoder;
  public RelativeEncoder floatingEncoder;

  public SparkMaxPIDController anchorPIDController;
  public SparkMaxPIDController floatingPIDController;

  public PeriodicIO periodicIO;

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

    this.periodicIO = new PeriodicIO();
    initTuneControllers();
  }

  public void configureMotors() {
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

  public void configureEncoders() {
    anchorEncoder.setPositionConversionFactor(Constants.Arm.Anchor.Conversions.kRatio);
    anchorEncoder.setPosition(Constants.Arm.Anchor.kZeroPosition);

    floatingEncoder.setPositionConversionFactor(Constants.Arm.Floating.Conversions.kRatio);
    floatingEncoder.setPosition(Constants.Arm.Floating.kZeroPosition);

    // determine velocity by delta(x)/delta(t)
  }

  public void configureControllers() {
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

  public void initTuneControllers() {
    SmartDashboard.putNumber(
        "anchorKP", SmartDashboard.getNumber("anchorKP", Constants.Arm.Anchor.PID.kP));
    SmartDashboard.putNumber(
        "anchorKI", SmartDashboard.getNumber("anchorKI", Constants.Arm.Anchor.PID.kI));
    SmartDashboard.putNumber(
        "anchorKD", SmartDashboard.getNumber("anchorKD", Constants.Arm.Anchor.PID.kD));
    SmartDashboard.putNumber(
        "anchorKG", SmartDashboard.getNumber("anchorKG", Constants.Arm.Anchor.FF.kg));

    SmartDashboard.putNumber(
        "floatingKP", SmartDashboard.getNumber("floatingKP", Constants.Arm.Floating.PID.kP));
    SmartDashboard.putNumber(
        "floatingKI", SmartDashboard.getNumber("floatingKI", Constants.Arm.Floating.PID.kI));
    SmartDashboard.putNumber(
        "floatingKD", SmartDashboard.getNumber("floatingKD", Constants.Arm.Floating.PID.kD));
    SmartDashboard.putNumber(
        "floatingKG", SmartDashboard.getNumber("floatingKG", Constants.Arm.Floating.FF.kg));

    SmartDashboard.putNumber(
        "setpointPos", SmartDashboard.getNumber("setpointPos", periodicIO.anchorPosSetpoint));
  }

  public void tuneControllers() {
    // double floatingKP = SmartDashboard.getEntry("floating KP").getDouble(0);
    // double floatingKI = SmartDashboard.getEntry("floatingKI").getDouble(0);
    // double floatingKD = SmartDashboard.getEntry("floatingKD").getDouble(0);
    // double floatingKFF = SmartDashboard.getEntry("floatingKFF").getDouble(0);

    // this.floatingPIDController.setP(floatingKP);
    // this.floatingPIDController.setI(floatingKI);
    // this.floatingPIDController.setD(floatingKD);
    // this.floatingPIDController.setFF(floatingKFF);

    double setpoint = SmartDashboard.getEntry("setpointPos").getDouble(0);
    double anchorKP = SmartDashboard.getEntry("anchorKP").getDouble(0);
    double anchorKI = SmartDashboard.getEntry("anchorKI").getDouble(0);
    double anchorKD = SmartDashboard.getEntry("anchorKD").getDouble(0);
    double anchorKG = SmartDashboard.getEntry("anchorKG").getDouble(0);

    SmartDashboard.putNumber("Pos", SmartDashboard.getNumber("Pos", getAnchorAngle()));
    SmartDashboard.putNumber("Vel", SmartDashboard.getNumber("Vel", getAnchorVelocity()));
    SmartDashboard.putNumber(
        "Output", SmartDashboard.getNumber("Output", anchorMotor.getAppliedOutput()));
    SmartDashboard.putNumber(
        "Current", SmartDashboard.getNumber("Current", anchorMotor.getOutputCurrent()));

    this.anchorPIDController.setP(anchorKP);
    this.anchorPIDController.setI(anchorKI);
    this.anchorPIDController.setD(anchorKD);
    periodicIO.anchorPosSetpoint = setpoint;
    Constants.Arm.Anchor.FF.kg = anchorKG;
  }

  public static class PeriodicIO {
    public double anchorPosSetpoint = Constants.Arm.Anchor.kZeroPosition;
    public double floatingPosSetpoint = Constants.Arm.Floating.kZeroPosition;
    public double anchorVelSetpoint = 0.0;
    public double floatingVelSetpoint = 0.0;
    public double anchorFF = 0.0;
    public double floatingFF = 0.0;

    // MOTION PROFILE
    // public TrapezoidProfile anchorProfile = new
    // TrapezoidProfile(Constants.Arm.Anchor.MP.ANCHOR_CONSTRAINTS, new TrapezoidProfile.State());
    // public TrapezoidProfile floatingProfile = new
    // TrapezoidProfile(Constants.Arm.Floating.MP.FLOATING_CONSTRAINTS, new
    // TrapezoidProfile.State());
    // public double anchorProfileStartTime = 0.0;
    // public double floatingProfileStartTime = 0.0;
  }

  @Override
  public void periodic() {
    tuneControllers();

    // MOTION PROFILE

    // //cal Pos & Vel at time t of MP
    // TrapezoidProfile.State anchorProfileState =
    // periodicIO.anchorProfile.calculate(Timer.getFPGATimestamp() -
    // periodicIO.anchorProfileStartTime);
    // TrapezoidProfile.State floatingProfileState =
    // periodicIO.floatingProfile.calculate(Timer.getFPGATimestamp() -
    // periodicIO.floatingProfileStartTime);

    // //cal FF using Pos & Vel above
    // periodicIO.anchorFF =
    // Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD.calculate(anchorProfileState.position,
    // anchorProfileState.velocity);
    // periodicIO.floatingFF =
    // Constants.Arm.Floating.FF.FLOATING_FEEDFORWARD.calculate(floatingProfileState.position,
    // floatingProfileState.velocity);

    // //set PIDControllers to MP setpoint
    // floatingPIDController.setReference(
    //   floatingProfileState.position,
    //   ControlType.kPosition,
    //   Constants.Arm.Floating.PID.kSlot,
    //   periodicIO.floatingFF,
    //   SparkMaxPIDController.ArbFFUnits.kVoltage);
    // anchorPIDController.setReference(
    //   anchorProfileState.position,
    //   ControlType.kPosition,
    //   Constants.Arm.Anchor.PID.kSlot,
    //   periodicIO.anchorFF,
    //   SparkMaxPIDController.ArbFFUnits.kVoltage);

  }
}

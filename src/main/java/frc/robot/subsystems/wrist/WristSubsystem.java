package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Simulation
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;

public class WristSubsystem extends SubsystemBase {

  private final PWMSparkMax motor = new PWMSparkMax(0);

  // Absolute encoder (through-bore on DIO). Returns [0,1) turns.
  private final DutyCycleEncoder abs = new DutyCycleEncoder(0);

  // Optional relative encoder on the motor (DIO 1/2) for velocity
  // You can omit this and estimate velocity from finite differences of abs angle.
  private final Encoder rel = new Encoder(1, 2);

  public static final class Constants {
    // Gear & geometry
    public static final double GEAR_RATIO = 100.0;        // motor rotations per wrist rotation (example)
    public static final double ARM_LENGTH_M = 0.20;       // wrist COM distance from joint (m), for sim
    public static final double MOI_KG_M2 = 0.015;         // moment of inertia, for sim (approx.)
    public static final double MIN_ANGLE_RAD = Units.degreesToRadians(-100.0);
    public static final double MAX_ANGLE_RAD = Units.degreesToRadians(+120.0);

    // Encoder zeroing
    public static final double ABSOLUTE_ZERO_OFFSET_RAD = Units.degreesToRadians(12.0); 
    // physical offset to make abs.getAbsolutePosition()==0 align with your chosen zero

    // Feedforward (from characterization)
    public static final double kS = 0.25;   // V
    public static final double kG = 0.85;   // V (gravity term magnitude)
    public static final double kV = 1.2;    // V per (rad/s)
    public static final double kA = 0.03;   // V per (rad/s^2)

    // PID
    public static final double kP = 3.0;
    public static final double kI = 0.0;
    public static final double kD = 0.15;

    // Motion profiling constraints
    public static final double MAX_VEL_RAD_PER_S   = Units.degreesToRadians(220.0);
    public static final double MAX_ACC_RAD_PER_S2  = Units.degreesToRadians(900.0);

    // Voltage limits
    public static final double MAX_APPLIED_VOLTAGE = 10.0; // keep margin from 12V to reduce brownouts
  }

  private final ArmFeedforward ff = new ArmFeedforward(
      Constants.kS, Constants.kG, Constants.kV, Constants.kA);

  private final ProfiledPIDController pid = new ProfiledPIDController(
      Constants.kP, Constants.kI, Constants.kD,
      new TrapezoidProfile.Constraints(
          Constants.MAX_VEL_RAD_PER_S, Constants.MAX_ACC_RAD_PER_S2));

  private boolean initialized = false;
  private double goalRad = 0.0; // current target (radians)

  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
      DCMotor.getNEO(1),                   // motor model
      Constants.GEAR_RATIO,
      Constants.MOI_KG_M2,
      Constants.ARM_LENGTH_M,
      Constants.MIN_ANGLE_RAD,
      Constants.MAX_ANGLE_RAD,
      true,                                // simulate gravity
      0.0                                  // starting angle
  );

  private final DutyCycleEncoderSim absSim = new DutyCycleEncoderSim(abs);

  public WristSubsystem() {
    // Make motor neutral behavior friendlier for holding
    motor.setInverted(false);
    // Configure PID tolerances for "atGoal"
    pid.setTolerance(Units.degreesToRadians(1.0), Units.degreesToRadians(5.0));
  }

  /** Set a wrist goal in radians; will be followed with motion profiling. */
  public void setGoalRadians(double radians) {
    radians = MathUtil.clamp(radians, Constants.MIN_ANGLE_RAD, Constants.MAX_ANGLE_RAD);
    goalRad = radians;
    pid.setGoal(goalRad);
  }

  public void setGoalDegrees(double deg) {
    setGoalRadians(Units.degreesToRadians(deg));
  }

  /** Nudge goal by small delta (deg) for driver trim. */
  public void nudgeDegrees(double deltaDeg) {
    setGoalRadians(goalRad + Units.degreesToRadians(deltaDeg));
  }

  /** Is the profiled PID on target? */
  public boolean atGoal() {
    return pid.atGoal();
  }

  /** Current wrist angle (radians) from absolute encoder with offset. */
  public double getAngleRad() {
    // abs returns [0,1) turns; convert to radians and apply offset
    double turns = abs.getAbsolutePosition(); // 0..1
    double angle = turns * (2.0 * Math.PI) + Constants.ABSOLUTE_ZERO_OFFSET_RAD;
    // Wrap to [-pi, pi] (or any consistent wrapping)
    angle = MathUtil.angleModulus(angle);
    return angle;
  }

  /** Estimated wrist velocity (rad/s). Use rel encoder if geared and configured, else finite diff. */
  public double getVelocityRadPerSec() {
    // If rel encoder is on the motor shaft, convert to joint rad/s
    double motorRadPerSec = rel.getRate() * 2.0 * Math.PI; // rel rate in rotations/s → rad/s
    return motorRadPerSec / Constants.GEAR_RATIO;
  }

  @Override
  public void periodic() {
    // Initialize PID reference once we have a valid angle
    double angle = getAngleRad();
    if (!initialized) {
      pid.reset(angle);
      pid.setGoal(angle);
      goalRad = angle;
      initialized = true;
    }

    // Profiled PID calculates desired setpoint (pos/vel) toward goal
    double pidOutput = pid.calculate(angle);

    // Pull the profiled setpoint for FF (best practice!)
    var setpoint = pid.getSetpoint(); // contains position & velocity
    double desiredPos = setpoint.position;
    double desiredVel = setpoint.velocity;

    // Estimate desired acceleration from profile slope (finite diff would also work if needed)
    // ProfiledPIDController internally computes a TrapezoidProfile; we can approximate accel:
    double desiredAccel = (pid.getSetpoint().velocity - desiredVel) / 0.02; // simple placeholder

    // Gravity-aware feedforward
    double ffVolts = ff.calculate(desiredPos, desiredVel, desiredAccel);

    // Combine & clamp
    double volts = MathUtil.clamp(ffVolts + pidOutput,
        -Constants.MAX_APPLIED_VOLTAGE, Constants.MAX_APPLIED_VOLTAGE);

    // Enforce soft limits by reducing output that would drive deeper into a limit
    if ((angle <= Constants.MIN_ANGLE_RAD && volts < 0) ||
        (angle >= Constants.MAX_ANGLE_RAD && volts > 0)) {
      volts = 0.0;
    }

    motor.setVoltage(volts);
  }

  @Override
  public void simulationPeriodic() {
    // Feed commanded voltage into sim
    double appliedVolts = motor.get() * RobotController.getBatteryVoltage(); // for PWM SparkMax
    // If you use setVoltage above (we do), better reflect that directly:
    appliedVolts = motor.getVoltage(); // WPILib motors expose this; otherwise keep prior line.

    armSim.setInputVoltage(appliedVolts);
    armSim.update(0.02);

    // Update absolute encoder sim from the joint angle
    double simAngle = armSim.getAngleRads(); // in radians
    double simTurns = (MathUtil.angleModulus(simAngle) - Constants.ABSOLUTE_ZERO_OFFSET_RAD) / (2.0 * Math.PI);
    // Make sure turns is [0,1)
    simTurns = (simTurns % 1.0 + 1.0) % 1.0;
    absSim.setAbsolutePosition(simTurns);

    // If you use rel encoder on motor: update by motor-side position
    double motorRotations = armSim.getPositionRadians() * Constants.GEAR_RATIO / (2.0 * Math.PI);
    rel.setDistance(motorRotations);
    // WPILib Encoder's rate requires setSamplesToAverage & encoding scale; skipping for brevity.
  }
}

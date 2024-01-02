package frc.quixlib.swerve;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.devices.QuixPigeon2;
import frc.quixlib.devices.QuixStatusSignal;
import frc.quixlib.localization.QuixSwerveLocalizer;
import frc.quixlib.localization.SwerveDriveOdometryMeasurement;
import frc.quixlib.math.MathUtils;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.PipelineVisionPacket;
import frc.quixlib.vision.QuixVisionCamera;
import frc.quixlib.vision.QuixVisionSim;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public abstract class QuixSwerve extends SubsystemBase {
  private final QuixPigeon2 m_imu;
  private final ArrayList<QuixVisionCamera> m_cameras;
  private final QuixSwerveModule[] m_modules;
  private final SwerveDriveKinematics m_kinematics;
  private final QuixSwerveModuleSetpointGenerator m_setpointGenerator;
  private SwerveSetpoint m_prevSetpoint;

  private static final SwerveSetpointGenerator.KinematicLimits m_kinematicLimits =
      new SwerveSetpointGenerator.KinematicLimits();
  private final SwerveSetpointGenerator m_setpointGenerator254;
  private SwerveSetpoint m_prevSetpoint254;
  private SwerveModuleState[] m_prevStates;

  private final ConrolThread m_controlThread;
  protected final ReadWriteLock m_controlThreadLock = new ReentrantReadWriteLock();
  private final double m_dt = 1.0 / Constants.Test.kSwerveControlFrequency;
  private double m_averageControlLoopTime = 0.0;

  private final QuixSwerveLocalizer m_localizer;
  private final double m_maxDriveSpeed;
  private final QuixSwerveController m_driveController;

  private final QuixVisionSim m_visionSim;

  private final Field2d m_fieldViz;
  private final Mechanism2d m_viz = new Mechanism2d(100, 100);
  private final MechanismRoot2d[] m_vizModuleRoots;
  private final MechanismLigament2d[] m_vizModuleCurrentState;
  private final MechanismLigament2d[] m_vizModuleTargetState;

  // Swerve control is executed in a separate thread at a higher frequency.
  // These variables store the control request from the main robot loop.
  // Note that |m_controlThreadLock| must be held when reading/writing to these variables.
  private ControlMode m_controlMode = ControlMode.DRIVE;
  private double m_xVel = 0.0; // m/s
  private double m_yVel = 0.0; // m/s
  private double m_thetaVel = 0.0; // rad/s
  private boolean m_fieldRelative = false;
  private double m_allowedScrub = Double.POSITIVE_INFINITY; // m/s
  private boolean m_closedLoopVelocity = false;
  private Pose2d m_targetPose = new Pose2d();

  private enum ControlMode {
    DRIVE,
    DRIVE_TO_POSE,
    STOP_WITH_X,
  }

  /**
   * Swerve drive class that handles all swerve-related functions, including kinematics, control,
   * and localization.
   */
  public QuixSwerve(
      final QuixPigeon2 imu,
      final ArrayList<QuixVisionCamera> cameras,
      final double maxDriveSpeed,
      final double maxModuleAcceleration,
      final double maxModuleSteeringRate,
      final QuixSwerveController driveController,
      final Fiducial[] targets,
      final QuixVisionSim visionSim,
      final Field2d fieldViz) {
    m_imu = imu;
    m_cameras = cameras;
    m_modules = createModules();
    m_kinematics = new SwerveDriveKinematics(getModulePositions());
    m_setpointGenerator =
        new QuixSwerveModuleSetpointGenerator(
            m_kinematics, maxDriveSpeed, maxModuleAcceleration, maxModuleSteeringRate, m_dt);
    m_prevSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());

    m_kinematicLimits.kMaxDriveVelocity = maxDriveSpeed;
    m_kinematicLimits.kMaxDriveAcceleration = maxModuleAcceleration;
    m_kinematicLimits.kMaxSteeringVelocity = maxModuleSteeringRate;
    m_setpointGenerator254 = new SwerveSetpointGenerator(m_kinematics, getModulePositions());
    m_prevSetpoint254 = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());
    m_prevStates = getModuleStates();

    m_localizer =
        new QuixSwerveLocalizer(
            m_kinematics,
            new Rotation2d(m_imu.getContinuousYaw()),
            getModulePositionStates(),
            new Pose2d(),
            targets);
    m_maxDriveSpeed = maxDriveSpeed;
    m_driveController = driveController;
    zeroModuleEncoders();

    m_targetPose = getPose();
    m_controlThread = new ConrolThread(m_modules);
    m_controlThread.start();

    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    m_visionSim = visionSim;

    // Setup Viz
    m_fieldViz = fieldViz;
    SmartDashboard.putData("Swerve Viz", m_viz);
    m_vizModuleRoots = new MechanismRoot2d[m_modules.length];
    m_vizModuleCurrentState = new MechanismLigament2d[m_modules.length];
    m_vizModuleTargetState = new MechanismLigament2d[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      m_vizModuleRoots[i] =
          m_viz.getRoot(
              "Swerve Module ID: " + i,
              50 + m_modules[i].getPosition().getX() * 100,
              50 + m_modules[i].getPosition().getY() * 100);
      m_vizModuleCurrentState[i] =
          m_vizModuleRoots[i].append(
              new MechanismLigament2d("Current State", 0, 0, 10, new Color8Bit(Color.kRed)));
      m_vizModuleTargetState[i] =
          m_vizModuleRoots[i].append(
              new MechanismLigament2d("Target State", 0, 0, 5, new Color8Bit(Color.kGreen)));
    }
  }

  /**
   * Returns an array of QuixSwerveModules that define the individual module configurations. Called
   * on construction.
   */
  protected abstract QuixSwerveModule[] createModules();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    for (var module : m_modules) {
      module.m_driveMotor.checkFaultsAndReconfigureIfNecessary();
      module.m_steeringMotor.checkFaultsAndReconfigureIfNecessary();
      // TODO: Handle reconfiguration if necessary.
    }

    // Update localization only when enabled.
    if (DriverStation.isEnabled()) {
      final ArrayList<PipelineVisionPacket> visionMeasurements = new ArrayList<>();
      for (var camera : m_cameras) {
        visionMeasurements.add(camera.getLatestMeasurement());
      }

      m_controlThreadLock.writeLock().lock();
      m_localizer.addVision(visionMeasurements);
      m_localizer.publishImmutableEntries();
      m_controlThreadLock.writeLock().unlock();
    }

    // Plot
    updateModuleViz();
    m_controlThreadLock.readLock().lock();
    m_fieldViz.getObject("Odometry").setPose(m_localizer.getOdometryPose());
    m_fieldViz.getObject("Localizer Raw").setPose(m_localizer.getRawPose());
    m_fieldViz.getObject("Localizer").setPose(m_localizer.getPose());
    m_controlThreadLock.readLock().unlock();

    SmartDashboard.putNumber("IMU: Roll", m_imu.getRoll());
    SmartDashboard.putNumber("IMU: Pitch", m_imu.getPitch());
    SmartDashboard.putNumber("IMU: Yaw", m_imu.getContinuousYaw() / (2.0 * Math.PI));
    SmartDashboard.putNumber("IMU: Roll rate", m_imu.getRollRate());
    SmartDashboard.putNumber("IMU: Pitch rate", m_imu.getPitchRate());
    SmartDashboard.putNumber("IMU: Yaw rate", m_imu.getYawRate());
  }

  /** Sets the IMU and localizer to the given pose. */
  public void resetPose(final Pose2d pose) {
    m_imu.setContinuousYaw(pose.getRotation().getRadians());
    m_controlThreadLock.writeLock().lock();
    m_localizer.resetPose(
        new Rotation2d(m_imu.getContinuousYaw()), getModulePositionStates(), pose);
    m_controlThreadLock.writeLock().unlock();
  }

  /**
   * Drive with open loop module velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void driveOpenLoop(
      final double xVel,
      final double yVel,
      final double thetaVel,
      final boolean fieldRelative,
      final double allowedScrub) {
    m_controlThreadLock.writeLock().lock();
    m_controlMode = ControlMode.DRIVE;
    m_xVel = xVel;
    m_yVel = yVel;
    m_thetaVel = thetaVel;
    m_fieldRelative = fieldRelative;
    m_allowedScrub = allowedScrub;
    m_closedLoopVelocity = false;
    m_controlThreadLock.writeLock().unlock();
  }

  /**
   * Drive with closed loop module velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void driveClosedLoop(
      final double xVel,
      final double yVel,
      final double thetaVel,
      final boolean fieldRelative,
      final double allowedScrub) {
    m_controlThreadLock.writeLock().lock();
    m_controlMode = ControlMode.DRIVE;
    m_xVel = xVel;
    m_yVel = yVel;
    m_thetaVel = thetaVel;
    m_fieldRelative = fieldRelative;
    m_allowedScrub = allowedScrub;
    m_closedLoopVelocity = true;
    m_controlThreadLock.writeLock().unlock();
  }

  /**
   * Drive with the given translation and rotation velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether the provided velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   * @param isClosedLoop Whether to use closed-loop module velocities
   */
  private void drive(
      final double xVel,
      final double yVel,
      final double thetaVel,
      final boolean fieldRelative,
      final double allowedScrub,
      final boolean isClosedLoop) {
    // TODO: Maybe latency compensated IMU helps?
    final ChassisSpeeds desiredChassisSpeeds =
        correctForDynamics(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVel, yVel, thetaVel, new Rotation2d(m_imu.getContinuousYaw()))
                : new ChassisSpeeds(xVel, yVel, thetaVel),
            m_dt);

    switch (Constants.Test.antiScrubMode) {
      case 604:
        {
          m_prevSetpoint =
              m_setpointGenerator.getFeasibleSetpoint(
                  m_prevSetpoint, desiredChassisSpeeds, allowedScrub);
          setDesiredModuleStates(m_prevSetpoint.moduleStates, isClosedLoop);
          break;
        }
      case 254:
        {
          var setpoint254 =
              m_setpointGenerator254.generateSetpoint(
                  m_kinematicLimits, m_prevSetpoint254, desiredChassisSpeeds, m_dt);
          m_prevSetpoint254 = setpoint254;

          // Unwrap states
          var curStates = getModuleStates();
          var unwrappedStates = new SwerveModuleState[setpoint254.moduleStates.length];
          for (int i = 0; i < setpoint254.moduleStates.length; i++) {
            unwrappedStates[i] = new SwerveModuleState();
            unwrappedStates[i].speedMetersPerSecond =
                setpoint254.moduleStates[i].speedMetersPerSecond;
            unwrappedStates[i].angle =
                new Rotation2d(
                    MathUtils.placeInScope(
                        setpoint254.moduleStates[i].angle.getRadians(),
                        curStates[i].angle.getRadians()));
          }
          setDesiredModuleStates(unwrappedStates, isClosedLoop);
          break;
        }
      default:
        {
          var newStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);

          // Unwrap states
          var curStates = getModuleStates();
          for (int i = 0; i < newStates.length; i++) {
            newStates[i] =
                QuixSwerveModuleSetpointGenerator.optimizeModule(newStates[i], curStates[i]);
          }
          m_prevStates = newStates;
          setDesiredModuleStates(m_prevStates, isClosedLoop);
          break;
        }
    }
  }

  /**
   * Approximation to correct for for swerve second order dynamics issue. Inspired by 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(
      final ChassisSpeeds originalSpeeds, final double dt) {

    final Pose2d robotPoseVel =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * dt,
            originalSpeeds.vyMetersPerSecond * dt,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * dt));
    final Twist2d twistVel = MathUtils.log(robotPoseVel);
    return new ChassisSpeeds(twistVel.dx / dt, twistVel.dy / dt, twistVel.dtheta / dt);
  }

  /**
   * Drives the swerve to the target pose.
   *
   * @param targetPose The target pose in field-frame
   * @param xVelocityRef Field-relative x-velocity feed-forward
   * @param yVelocityRef Field-relative y-velocity feed-forward
   * @param thetaVelocityRef Theta-velocity feed-forward
   * @param allowedScrub Allowable module scrub in m/s
   * @return the error between the current pose and the target pose
   */
  public Pose2d driveToPose(
      final Pose2d targetPose,
      final double xVelocityRef,
      final double yVelocityRef,
      final double thetaVelocityRef,
      final double allowedScrub) {
    m_controlThreadLock.writeLock().lock();
    m_controlMode = ControlMode.DRIVE_TO_POSE;
    m_targetPose = targetPose;
    m_xVel = xVelocityRef;
    m_yVel = yVelocityRef;
    m_thetaVel = thetaVelocityRef;
    m_allowedScrub = allowedScrub;
    m_controlThreadLock.writeLock().unlock();

    final Pose2d currentPose = getPose();
    final Pose2d poseError =
        new Pose2d(
            targetPose.getX() - currentPose.getX(),
            targetPose.getY() - currentPose.getY(),
            targetPose.getRotation().minus(currentPose.getRotation()));

    m_fieldViz.getObject("Target Pose").setPose(targetPose);
    SmartDashboard.putNumber("Chassis x error", poseError.getX());
    SmartDashboard.putNumber("Chassis y error", poseError.getY());
    SmartDashboard.putNumber("Chassis theta error (degrees)", poseError.getRotation().getDegrees());
    return poseError;
  }

  /**
   * Turns the swerve in-place to the target angle.
   *
   * @param angle The target angle in field-frame
   * @param thetaVelocityRef Theta-velocity feed-forward
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void turnToAngle(
      final Rotation2d angle, final double thetaVelocityRef, final double allowedScrub) {
    final Pose2d targetPose = new Pose2d(getPose().getTranslation(), angle);
    final ChassisSpeeds fieldRelativeChassisSpeeds =
        m_driveController.calculate(getPose(), targetPose, 0.0, 0.0, thetaVelocityRef);

    m_controlThreadLock.writeLock().lock();
    m_controlMode = ControlMode.DRIVE_TO_POSE;
    m_targetPose = targetPose;
    m_xVel = 0.0;
    m_yVel = 0.0;
    m_thetaVel = fieldRelativeChassisSpeeds.omegaRadiansPerSecond;
    m_allowedScrub = allowedScrub;
    m_closedLoopVelocity = false;
    m_controlThreadLock.writeLock().unlock();
  }

  /** Drive open loop with zero velocity. */
  public void stop() {
    m_controlThreadLock.writeLock().lock();
    m_controlMode = ControlMode.DRIVE;
    m_xVel = 0.0;
    m_yVel = 0.0;
    m_thetaVel = 0.0;
    m_allowedScrub = Double.POSITIVE_INFINITY;
    m_fieldRelative = false;
    m_closedLoopVelocity = false;
    m_controlThreadLock.writeLock().unlock();
  }

  /** Turns modules to form an X pointing at the origin with zero velocity. */
  public void stopWithX() {
    m_controlThreadLock.writeLock().lock();
    m_controlMode = ControlMode.STOP_WITH_X;
    m_controlThreadLock.writeLock().unlock();
  }

  private ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public FieldSpeeds getFieldSpeeds() {
    final Rotation2d rotation = getPose().getRotation();
    final ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    final double vxField =
        chassisSpeeds.vxMetersPerSecond * rotation.getCos()
            - chassisSpeeds.vyMetersPerSecond * rotation.getSin();
    final double vyField =
        chassisSpeeds.vxMetersPerSecond * rotation.getSin()
            + chassisSpeeds.vyMetersPerSecond * rotation.getCos();
    return new FieldSpeeds(vxField, vyField, chassisSpeeds.omegaRadiansPerSecond);
  }

  private Translation2d[] getModulePositions() {
    final Translation2d[] positions = new Translation2d[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getPosition();
    }
    return positions;
  }

  private SwerveModuleState[] getModuleStates() {
    final SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositionStates() {
    return getModulePositionStates(true);
  }

  private SwerveModulePosition[] getModulePositionStates(final boolean refresh) {
    final SwerveModulePosition[] states = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getPositionState(refresh);
    }
    return states;
  }

  public Pose2d getPose() {
    try {
      m_controlThreadLock.readLock().lock();
      return m_localizer.getPose();
    } finally {
      m_controlThreadLock.readLock().unlock();
    }
  }

  public void setContinuousYaw(double yaw) {
    m_imu.setContinuousYaw(yaw);
  }

  private void zeroModuleEncoders() {
    for (var module : m_modules) {
      module.zeroToAbsPosition();
    }
  }

  private void setDesiredModuleStates(
      final SwerveModuleState[] desiredStates, final boolean isClosedLoop) {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(desiredStates[i], isClosedLoop);
    }
  }

  private void updateModuleViz() {
    final var moduleScrubs = m_setpointGenerator.computeModuleScrubs(getModuleStates());
    final double kSpeedVizScalar = 5.0;
    for (int i = 0; i < m_modules.length; i++) {
      var currentState = m_modules[i].getState();
      m_vizModuleCurrentState[i].setAngle(currentState.angle);
      m_vizModuleCurrentState[i].setLength(kSpeedVizScalar * currentState.speedMetersPerSecond);
      m_vizModuleCurrentState[i].setLineWeight(5 + 100 * (moduleScrubs[i] / m_maxDriveSpeed));

      var targetState = m_modules[i].getLastCommandedState();
      m_vizModuleTargetState[i].setAngle(targetState.angle);
      m_vizModuleTargetState[i].setLength(kSpeedVizScalar * targetState.speedMetersPerSecond);

      SmartDashboard.putNumber("Module Scrub " + i, moduleScrubs[i]);
    }
  }

  public class ConrolThread extends Thread {
    private static final int kThreadPriority = 1;
    private final QuixStatusSignal[] m_allSignals;

    // Track thread timing
    private final MedianFilter m_peakRemover = new MedianFilter(3);
    private final LinearFilter m_lowPass = LinearFilter.movingAverage(5);
    private double m_lastTime = 0.0;

    public ConrolThread(final QuixSwerveModule[] modules) {
      super();
      setDaemon(true);

      // 4 signals for each module + 2 for Pigeon2.
      m_allSignals = new QuixStatusSignal[(modules.length * 4) + 2];
      for (int i = 0; i < modules.length; i++) {
        m_allSignals[i * 4] = modules[i].m_driveMotor.sensorPositionSignal();
        m_allSignals[(i * 4) + 1] = modules[i].m_driveMotor.sensorVelocitySignal();
        m_allSignals[(i * 4) + 2] = modules[i].m_steeringMotor.sensorPositionSignal();
        m_allSignals[(i * 4) + 3] = modules[i].m_steeringMotor.sensorVelocitySignal();
      }
      m_allSignals[m_allSignals.length - 2] = m_imu.continuousYawSignal();
      m_allSignals[m_allSignals.length - 1] = m_imu.yawRateSignal();
    }

    @Override
    public void run() {
      Threads.setCurrentThreadPriority(true, kThreadPriority);

      // Run as fast as possible, the blocking for the signals will control the timing.
      while (true) {
        // Synchronously wait for all signals, up to twice the period of the update frequency.
        var status =
            QuixStatusSignal.waitForAll(2.0 / Constants.Test.kSwerveControlFrequency, m_allSignals);
        if (status != StatusCode.OK) {
          DriverStation.reportWarning("Odometry waitForAll error: " + status, false);
        }

        m_controlThreadLock.writeLock().lock();

        // Compute loop stats
        final double currentTime = Timer.getFPGATimestamp();
        m_averageControlLoopTime =
            m_lowPass.calculate(m_peakRemover.calculate(currentTime - m_lastTime));
        m_lastTime = currentTime;

        // Only update localizer when enabled
        if (DriverStation.isEnabled()) {
          final double yaw =
              QuixStatusSignal.getLatencyCompensatedValue(
                  m_imu.continuousYawSignal(), m_imu.yawRateSignal());
          final var odometryMeasurement =
              new SwerveDriveOdometryMeasurement(
                  new Rotation2d(yaw), getModulePositionStates(false));
          m_localizer.updateOdom(odometryMeasurement);
        }

        // Apply synchronous controls
        switch (m_controlMode) {
          case DRIVE_TO_POSE:
            {
              final ChassisSpeeds fieldRelativeChassisSpeeds =
                  m_driveController.calculate(
                      m_localizer.getPose(), m_targetPose, m_xVel, m_yVel, m_thetaVel);
              drive(
                  fieldRelativeChassisSpeeds.vxMetersPerSecond,
                  fieldRelativeChassisSpeeds.vyMetersPerSecond,
                  fieldRelativeChassisSpeeds.omegaRadiansPerSecond,
                  false,
                  m_allowedScrub,
                  m_closedLoopVelocity);
              break;
            }
          case DRIVE:
            {
              drive(
                  m_xVel,
                  m_yVel,
                  m_thetaVel,
                  m_fieldRelative,
                  m_allowedScrub,
                  m_closedLoopVelocity);
              break;
            }
          case STOP_WITH_X:
            {
              final var moduleStates = getModuleStates();
              final SwerveModuleState[] desiredStates = new SwerveModuleState[moduleStates.length];
              for (int i = 0; i < moduleStates.length; i++) {
                final double angle =
                    Math.atan2(
                        m_modules[i].getPosition().getY(), m_modules[i].getPosition().getX());
                desiredStates[i] =
                    new SwerveModuleState(
                        0.0,
                        new Rotation2d(
                            MathUtils.placeInScope(angle, moduleStates[i].angle.getRadians())));
              }
              setDesiredModuleStates(desiredStates, false);
              break;
            }
          default:
            {
              throw new RuntimeException("Invalid swerve control mode: " + m_controlMode);
            }
        }
        m_controlThreadLock.writeLock().unlock();
      }
    }
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private Pose2d m_simPose = new Pose2d();

  /** Sets the simulated pose to the given pose. */
  public void resetSimPose(final Pose2d pose) {
    m_simPose = pose;
    m_visionSim.resetSimPose(pose);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // Update vision sim with sim pose.
    m_visionSim.updatePose(m_simPose);

    // TODO: Sim jointly as one system.
    for (var module : m_modules) {
      module.updateSimPeriodic();
    }

    // Update pose by integrating ChassisSpeeds.
    final ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    m_simPose =
        m_simPose.transformBy(
            new Transform2d(
                new Translation2d(
                    chassisSpeeds.vxMetersPerSecond * TimedRobot.kDefaultPeriod,
                    chassisSpeeds.vyMetersPerSecond * TimedRobot.kDefaultPeriod),
                new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * TimedRobot.kDefaultPeriod)));
    m_fieldViz.setRobotPose(m_simPose);

    // Update IMU based on sim pose.
    m_imu.setSimContinuousYaw(m_simPose.getRotation().getRadians());
  }
  // --- END STUFF FOR SIMULATION ---
}

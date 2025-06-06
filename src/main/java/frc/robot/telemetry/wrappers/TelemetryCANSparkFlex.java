package frc.robot.telemetry.wrappers;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

public class TelemetryCANSparkFlex extends SparkFlex {
  private final DoubleTelemetryEntry outputAmpsEntry;
  private final DoubleTelemetryEntry outputPercentEntry;
  private final DoubleTelemetryEntry temperatureEntry;
  private final BooleanTelemetryEntry inBrakeModeEntry;
  private final DoubleTelemetryEntry positionEntry;
  private final DoubleTelemetryEntry velocityEntry;
  SparkFlexConfig config = new SparkFlexConfig();

  /**
   * Create a new object to control a SPARK Flex motor Controller
   *
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected
   *     to their matching colors and the hall sensor must be plugged in. Brushed motors must be
   *     connected to the Red and Black terminals only.
   */
  public TelemetryCANSparkFlex(
      int deviceId, MotorType type, String telemetryPath, boolean tuningMode) {
    super(deviceId, type);

    telemetryPath += "/";

    outputAmpsEntry = new DoubleTelemetryEntry(telemetryPath + "outputAmps", tuningMode);
    outputPercentEntry = new DoubleTelemetryEntry(telemetryPath + "outputPercent", tuningMode);
    temperatureEntry = new DoubleTelemetryEntry(telemetryPath + "temperature", tuningMode);
    inBrakeModeEntry = new BooleanTelemetryEntry(telemetryPath + "inBrakeMode", tuningMode);
    positionEntry = new DoubleTelemetryEntry(telemetryPath + "position", tuningMode);
    velocityEntry = new DoubleTelemetryEntry(telemetryPath + "velocity", tuningMode);
  }

  public SparkFlexConfig IdleMode(IdleMode mode) {
    inBrakeModeEntry.append(mode == IdleMode.kBrake);

    return IdleMode(mode);
  }

  public REVLibError burnFlashWithDelay() {
    Timer.delay(0.1);
    REVLibError error =
        configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Timer.delay(0.1);
    return error;
  }

  public void logValues() {
    outputAmpsEntry.append(super.getOutputCurrent());
    outputPercentEntry.append(super.getAppliedOutput());
    temperatureEntry.append(super.getMotorTemperature());
    positionEntry.append(super.getEncoder().getPosition());
    velocityEntry.append(super.getEncoder().getVelocity());
  }
}

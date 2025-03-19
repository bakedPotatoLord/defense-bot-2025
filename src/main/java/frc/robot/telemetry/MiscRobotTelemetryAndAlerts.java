package frc.robot.telemetry;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.StringTelemetryEntry;
import frc.robot.telemetry.types.StructTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.RaiderStructs;

public class MiscRobotTelemetryAndAlerts {
  private static final String tableName = "/robot/";

  private final Alert highRIOCanUsageAlert = new Alert("High RIO CAN Usage :(", AlertType.WARNING);
  private final LinearFilter highRIOCanUsageFilter = LinearFilter.movingAverage(50);


  private final DoubleTelemetryEntry inputVoltageEntry =
      new DoubleTelemetryEntry(tableName + "inputVoltage", MiscConstants.TUNING_MODE);
  private final DoubleTelemetryEntry inputCurrentEntry =
      new DoubleTelemetryEntry(tableName + "inputCurrent", MiscConstants.TUNING_MODE);
  private final StructTelemetryEntry<CANStatus> canStatusEntry =
      new StructTelemetryEntry<>(
          tableName + "rioCANStatus", RaiderStructs.CANStatusStruct, MiscConstants.TUNING_MODE);

  private final StringTelemetryEntry startCommandsEntry =
      new StringTelemetryEntry(tableName + "startCommands", false);
  private final StringTelemetryEntry endCommandsEntry =
      new StringTelemetryEntry(tableName + "endCommands", false);

  public MiscRobotTelemetryAndAlerts() {
    

    if (MiscConstants.TUNING_MODE) {
      Alert tuningModeAlert = new Alert("Tuning Mode is Enabled :)", AlertType.INFO);
      tuningModeAlert.set(true);
    }

    CommandScheduler.getInstance()
        .onCommandInitialize((command) -> startCommandsEntry.append(getCommandID(command)));
    CommandScheduler.getInstance()
        .onCommandFinish((command) -> endCommandsEntry.append(getCommandID(command)));
    CommandScheduler.getInstance()
        .onCommandInterrupt((command) -> endCommandsEntry.append(getCommandID(command)));


  }

  private String getCommandID(Command command) {
    return command.getName() + "(" + command.hashCode() + ")";
  }

  public void logValues() {
    inputVoltageEntry.append(RobotController.getInputVoltage());
    inputCurrentEntry.append(RobotController.getInputCurrent());

    // RIO CAN Usage
    CANStatus canStatus = RobotController.getCANStatus();
    canStatusEntry.append(canStatus);
    double percentBusUsage = canStatus.percentBusUtilization;
    double filtered = highRIOCanUsageFilter.calculate(percentBusUsage);
    highRIOCanUsageAlert.set(filtered >= 0.9);
  }
}

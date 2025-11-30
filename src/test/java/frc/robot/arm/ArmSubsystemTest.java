package frc.robot.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.cancoder.CANcoderIO;
import frc.robot.pivot.PivotIO;
import frc.robot.pivot.PivotIOSim;
import frc.robot.roller.RollerIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ArmSubsystemTest {
  ArmSubsystem arm;
  RollerIOSim rollerIO;
  PivotIO pivotIO;
  CANcoderIO cancoderIO;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    TalonFXConfiguration armPivotConfig =
        Robot.createPivotConfig(
                InvertedValue.Clockwise_Positive,
                ArmSubsystem.SUPPLY_CURRENT_LIMIT,
                ArmSubsystem.STATOR_CURRENT_LIMIT,
                ArmSubsystem.SENSOR_TO_MECH_RATIO,
                ArmSubsystem.KV,
                ArmSubsystem.KG,
                ArmSubsystem.KS,
                ArmSubsystem.KP,
                ArmSubsystem.KI,
                ArmSubsystem.KD)
            // this is what lets us wrap it from -180 to 180 at the bottom
            // basically think of it like the swerve turn motor
            // it's not actually fused with the cancoder, which means it shows up weird in
            // the log
            // but that's true for the swerve as well and both seem to be fine so we'll just
            // roll with
            // it
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true));
    rollerIO =
        new RollerIOSim(
            ArmSubsystem.jKgMetersSquared,
            ArmSubsystem.PIVOT_RATIO,
            new SimpleMotorFeedforward(ArmSubsystem.KS, ArmSubsystem.KV),
            new ProfiledPIDController(
                ArmSubsystem.KP,
                ArmSubsystem.KI,
                ArmSubsystem.KD,
                new TrapezoidProfile.Constraints(
                    ArmSubsystem.MAX_VELOCITY, ArmSubsystem.MAX_ACCELERATION)));
    pivotIO =
        new PivotIOSim(
            ArmSubsystem.MIN_ANGLE.getRadians(),
            ArmSubsystem.MAX_ANGLE.getRadians(),
            ArmSubsystem.LENGTH_METERS,
            ArmSubsystem.MAX_VELOCITY,
            ArmSubsystem.MAX_ACCELERATION,
            armPivotConfig);

    cancoderIO =
        new CANcoderIO() {

          @Override
          public void updateInputs(CANcoderIOInputs inputs) {
            // pass
          }
        };

    arm = new ArmSubsystem(rollerIO, pivotIO, cancoderIO, "test RollerPivot");
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void testIntakeCoral() {
    // Verify initial conditions: no coral present, no voltage applied to the
    // roller.
    assertFalse(arm.hasCoral);
    assertEquals(0.0, arm.getRollerVoltage());

    // Schedule the Command under test: intakeCoral()
    Command c = arm.intakeCoral();
    c.schedule();

    // This test just executes line by line; there is no event loop.
    // Thus we have to simulate the event loop by calling
    // CommandScheduler.getInstance().run() ourselves. Each call corresponds to one
    // time step, mirroring the call in Robot.robotPeriodic().

    // Two time steps are needed for the Command to execute and for the roller
    // voltage to be updated.
    CommandScheduler.getInstance().run();
    CommandScheduler.getInstance().run();

    // Confirm that the roller voltage has been updated.
    assertEquals(ArmSubsystem.CORAL_INTAKE_VOLTAGE, arm.getRollerVoltage());

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    // Sanity-check: no coral yet!
    assertFalse(arm.hasCoral);

    // Simulate having a coral in the roller, which causes the motor to stall,
    // increasing its current draw.
    rollerIO.setRollerStall(arm.getRollerIOInputs());

    // Some time passes-- enough for the stator amps moving average to catch up.
    for (int i = 0; i < 5; i++) {
      CommandScheduler.getInstance().run();
    }

    // Ensure we wait 250ms of real time, to allow the debounced trigger to fire.
    try {
      Thread.sleep(250);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    // One more time step for the rest of the Command to play out.
    CommandScheduler.getInstance().run();

    // We've reached the third phase of the Command, which records detecting the
    // coral.
    assertTrue(arm.hasCoral);

    // Sanity-check: Once set, the roller voltage never changes.
    assertEquals(ArmSubsystem.CORAL_INTAKE_VOLTAGE, arm.getRollerVoltage());
  }
}

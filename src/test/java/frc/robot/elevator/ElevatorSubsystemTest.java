package frc.robot.elevator;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ElevatorSubsystemTest {
  ElevatorSubsystem elevator;
  ElevatorIOSim elevatorIO;

  @BeforeEach // this method will run before each test
  void setup() {

    // Reset singleton CommandScheduler
    Field instance;
    try {
      instance = CommandScheduler.class.getDeclaredField("instance");
      instance.setAccessible(true);
      instance.set(null, null); // Reset the instance to null
    } catch (Exception e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    elevatorIO = new ElevatorIOSim();

    elevator = new ElevatorSubsystem(elevatorIO);
  }

  @AfterEach
  void tearDown() {}

  @Test
  void testSetExtensionMeters() {
    // Verify initial conditions: elevator at zero
    assertTrue(elevator.atExtension(0));

    // Schedule the Command under test: setExtensionMeters()
    Command c = elevator.setExtensionMeters(() -> 0.5);
    c.schedule();

    // This test just executes line by line; there is no event loop.
    // Thus we have to simulate the event loop by calling
    // CommandScheduler.getInstance().run() ourselves. Each call corresponds to one
    // time step, mirroring the call in Robot.robotPeriodic().

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertTrue(elevator.atExtension(0.5));
  }

  @Test
  void testSetExtensionMetersZero() {
    // Verify initial conditions: elevator at zero
    assertTrue(elevator.atExtension(0));

    // Set up new initial conditions: elevator at 0.5
    Command c = elevator.setExtensionMeters(() -> 0.5);
    c.schedule();

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertTrue(elevator.atExtension(0.5));

    // Schedule the Command under test: setExtensionMeters(0)
    Command d = elevator.setExtensionMeters(() -> 0);
    d.schedule();

    // Some time passes...
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
    }

    assertTrue(elevator.atExtension(0.5));
  }
}

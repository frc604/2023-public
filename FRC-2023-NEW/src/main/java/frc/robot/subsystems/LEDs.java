// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private final AddressableLED leds = new AddressableLED(Constants.LEDs.ledPort);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LEDs.ledCount);

  private final Intake m_intakeSubsystem;
  private final ScoringSelector m_scoringSelector;

  public LEDs(Intake intakeSubsystem, ScoringSelector scoringSelector) {
    leds.setLength(ledBuffer.getLength());
    leds.setData(ledBuffer);
    leds.start();

    m_intakeSubsystem = intakeSubsystem;
    m_scoringSelector = scoringSelector;
  }

  private void turnOff() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
  }

  private void yellow() { // Want cone/yellow piece
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 200, 0);
    }
  }

  private void purple() { // Want cube/purple piece
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 150, 0, 255);
    }
  }

  private void green() { // Intook sucessfully
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
  }

  @Override
  public void periodic() {
    // If intook mark green
    if (m_intakeSubsystem.isRollerStalled()) {
      green();
    } else { // If not intaken mark based on color
      switch (m_scoringSelector.getNodeID()) {
          // Bottom Row
        case 0:
          {
            yellow();
            break;
          }
        case 1:
          {
            yellow();
            break;
          }
        case 2:
          {
            yellow();
            break;
          }
          // Middle Row
        case 3:
          {
            yellow();
            break;
          }
        case 4:
          {
            purple();
            break;
          }
        case 5:
          {
            yellow();
            break;
          }
          // Top Row
        case 6:
          {
            yellow();
            break;
          }
        case 7:
          {
            purple();
            break;
          }
        case 8:
          {
            yellow();
            break;
          }
        default:
          {
            turnOff();
            break;
          }
      }
    }

    leds.setData(ledBuffer);
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  // --- END STUFF FOR SIMULATION ---
}

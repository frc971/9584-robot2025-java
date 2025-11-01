package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private static final int kPWMPort = 1; 
    private static final int kLength = 30;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    private Pattern m_currentPattern = Pattern.OFF;
    private Color m_primaryColor = new Color(0, 0, 0);
    private Color m_secondaryColor = new Color(0, 0, 0);
    private double m_animationCounter = 0.0;
    private double m_blinkFrequency = 1.0;

    private enum Pattern {
        SOLID,
        RAINBOW,
        CHASE,
        BLINK,
        ALTERNATING,
        BREATHE,
        OFF
    }

    public LEDSubsystem() {
        m_led = new AddressableLED(kPWMPort);
        m_ledBuffer = new AddressableLEDBuffer(kLength);
        
        m_led.setLength(kLength);
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        updateLEDs();
        m_led.setData(m_ledBuffer);
        m_animationCounter += 0.02;
    }

    public void setSolidColor(Color color) {
        m_currentPattern = Pattern.SOLID;
        m_primaryColor = color;
    }

    public void setRainbow() {
        m_currentPattern = Pattern.RAINBOW;
    }

    public void setChase(Color color) {
        m_currentPattern = Pattern.CHASE;
        m_primaryColor = color;
    }

    public void setBlink(Color color, double frequency) {
        m_currentPattern = Pattern.BLINK;
        m_primaryColor = color;
        m_blinkFrequency = frequency;
    }

    public void setBlink(Color color) {
        setBlink(color, 1.0);
    }

    public void setAlternating(Color color1, Color color2) {
        m_currentPattern = Pattern.ALTERNATING;
        m_primaryColor = color1;
        m_secondaryColor = color2;
    }

    public void setBreathe(Color color) {
        m_currentPattern = Pattern.BREATHE;
        m_primaryColor = color;
    }

    public void off() {
        m_currentPattern = Pattern.OFF;
        for (int i = 0; i < kLength; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    public void setDisabledMode() {
        setRainbow();
    }

    public void setTeleopMode() {
        setAllianceColor();
    }

    public void setAutonomousMode() {
        setChase(Color.kGreen);
    }

    public void setAllianceColor() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                setSolidColor(Color.kGreen);
            } else {
                setSolidColor(Color.kBlue);
            }
        } else {
            setSolidColor(Color.kWhite);
        }
    }

    private void updateLEDs() {
        switch (m_currentPattern) {
            case SOLID:
                updateSolid();
                break;
            case RAINBOW:
                updateRainbow();
                break;
            case CHASE:
                updateChase();
                break;
            case BLINK:
                updateBlink();
                break;
            case ALTERNATING:
                updateAlternating();
                break;
            case BREATHE:
                updateBreathe();
                break;
            case OFF:
                break;
        }
    }

    private void updateSolid() {
        for (int i = 0; i < kLength; i++) {
            m_ledBuffer.setLED(i, m_primaryColor);
        }
    }

    private void updateRainbow() {
        int rainbowFirstPixelHue = (int)(m_animationCounter * 90) % 180;

        for (int i = 0; i < kLength; i++) {
            int hue = (rainbowFirstPixelHue + (i * 180 / kLength)) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
    }

    private void updateChase() {
        int position = (int)(m_animationCounter * 20) % kLength;
        int chaseLength = 5;

        for (int i = 0; i < kLength; i++) {
            if (Math.abs(i - position) < chaseLength) {
                m_ledBuffer.setLED(i, m_primaryColor);
            } else {
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
    }

    private void updateBlink() {
        double blinkPhase = (m_animationCounter * m_blinkFrequency) % 1.0;
        boolean isOn = blinkPhase < 0.5;

        for (int i = 0; i < kLength; i++) {
            if (isOn) {
                m_ledBuffer.setLED(i, m_primaryColor);
            } else {
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
    }

    private void updateAlternating() {
        for (int i = 0; i < kLength; i++) {
            if (i % 2 == 0) {
                m_ledBuffer.setLED(i, m_primaryColor);
            } else {
                m_ledBuffer.setLED(i, m_secondaryColor);
            }
        }
    }

    private void updateBreathe() {
        double brightness = (Math.sin(m_animationCounter * 2.0) + 1.0) / 2.0;

        for (int i = 0; i < kLength; i++) {
            m_ledBuffer.setRGB(i,
                (int)(m_primaryColor.red * 255 * brightness),
                (int)(m_primaryColor.green * 255 * brightness),
                (int)(m_primaryColor.blue * 255 * brightness));
        }
    }
}
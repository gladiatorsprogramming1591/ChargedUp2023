package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

        // 120 degrees is Purple
        // 110-115 degrees is Yellow
        //90 degrees is Blue
    private static Servo m_LEDs;

    private double m_degrees = 90; // Starts at blue

    public final double BLUE = 90;
    public final double YELLOW = 110;
    public final double PURPLE = 120;

    boolean called = false;
    double cycleState = 40;
    double count = 0;


    public LEDs(){
        m_LEDs = new Servo(0);
        // SmartDashboard.putNumber("LED Degrees", m_Degrees);
    }

    @Override
    public void periodic(){
        m_LEDs.setAngle(m_degrees);
        // m_LEDs.setAngle(SmartDashboard.getNumber("LED Degrees", m_Degrees));  // Use SmartDashboard not ShuffleBoard
        // SmartDashboard.putNumber("LED Degrees Get", SmartDashboard.getNumber("LED Degrees", m_Degrees));
    }

    public void setColor(double color){
        m_degrees = color;
    }

    public void setPiece(){
        if (called) {
            called = false;
            setColor(PURPLE);
        } else {
            called = true;
            setColor(YELLOW);
        }
    }

    public void cycle(){
        setColor(cycleState);
        cycleState = cycleState + 20;
        if (cycleState >= 140) cycleState = 40;
        if (++count %25 == 0) cycle();  // cycles every half second
    }

}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

        // PWM Microseconds = 0.09(degrees) - 45
        // Degrees          = 100/9(PWM) + 500
    private static Servo m_LEDs;

    private double m_degrees = -1; // Starts at Arduino default

    public final double BLUE = 90;
    public final double YELLOW = 110;
    public final double PURPLE = 120;

    public final double WHITE = 40.6; // minimum color  (below is not defined)
    public final double TEAL = 130.4; // maximum color (above is off)
    public final double OFF = 140;
    double cycleState = WHITE;
    double flashState;
    boolean blink = false;
    double count = 0;

    boolean called = false;



    public LEDs(){
        m_LEDs = new Servo(0);
        // SmartDashboard.putNumber("LED Degrees", m_Degrees);
    }

    @Override
    public void periodic(){
        if (m_degrees >= 0) m_LEDs.setAngle(m_degrees);
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
        // cycles every quarter second
        if (++count %5 == 0) {      // 3%
            setColor(cycleState);
            cycleState = cycleState + 9;
            if (cycleState >= TEAL) cycleState = WHITE;
        }
    }

    public void off(){
        setColor(OFF);
        called = false;
    }

    public void flashing(){
        if (++count %5 == 0) {      // 3%
            if (called) {           // Previously Purple
                if (!blink){
                    blink = true;
                    setColor(PURPLE);
                } else {
                    blink = false;
                    setColor(OFF);    
                }
            } else {
                if (!blink){
                    blink = true;
                    setColor(YELLOW);
                } else {
                    blink = false;
                    setColor(OFF);    
                }
            }

            setColor(cycleState);
            cycleState = cycleState + 9;
            if (cycleState >= TEAL) cycleState = WHITE;
        }
        // if (++count %3 == 0) {
        //     if (called){
        //     //     if (flashStatePurple == PURPLE) flashStatePurple = OFF;
        //     //     else flashStatePurple = PURPLE;
        //     // }
        //     // } else {
        //     //     if (flashStateYellow == YELLOW) flashStateYellow = OFF;
        //     //     else flashStateYellow = YELLOW;
        // }
    }

}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

    private static PWM m_LEDs;

    private int count = 0;

    // private double m_Pos = 0.5;

    private double m_Speed = -1.0;

    // private int m_Raw = 128;

    public LEDs(){
        m_LEDs = new PWM(0);

        // SmartDashboard.putNumber("LED Pos", m_Pos);
        // SmartDashboard.putNumber("LED Speed", m_Speed);
        // SmartDashboard.putNumber("LED Raw", m_Raw);
    }

    @Override
    public void periodic(){
        if (++count%250 == 0) {
            m_Speed = m_Speed + .05;
            m_LEDs.setSpeed(m_Speed);
            System.out.println("Speed is " + m_Speed);
        }
        if (m_Speed >= 1) m_Speed = -1.0;
    //     double pos = m_Pos;
    //     double speed = m_Speed;
    //     double raw = m_Raw;
    //     pos = SmartDashboard.getNumber("LED Pos", m_Pos);
    //     speed = SmartDashboard.getNumber("LED Speed", m_Speed);
    //     raw = SmartDashboard.getNumber("LED Raw", m_Raw);

    //     if (pos != m_Pos) {
    //         m_Pos = pos;
    //         m_LEDs.setPosition(pos);
    //         System.out.println("Setting pos = " + pos);
    //     }


    //     System.out.println("Setting speed = " + speed);
    //     // if (speed != m_Speed) {
    //     //     m_Speed = speed;
    //         m_LEDs.setSpeed(speed);
    //     // }

    //     if (raw != m_Raw) {
    //         m_Raw = (int) raw;
    //         m_LEDs.setRaw(m_Raw);
    //         System.out.println("Setting raw = " + raw);
    //     }
    }

    public void setColor(double color){
        m_LEDs.setSpeed(color);
    }

}
